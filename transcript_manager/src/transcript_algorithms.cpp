#include "transcript_manager/transcript.hpp"

namespace whisper {

void Transcript::merge_one(const std::vector<Segment> &other) {
  if ( !other.empty() ) {
    // Set the stale segment based on incoming timestamp
    set_stale_segment(other[0].get_start());
  }

  if ( empty() ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "[LCS] First Words Added to Transcript");
    push_back(other);
    return;
  } // else segments_.size() > 0

  // Get comparable strings for fuzzy lcs matching
  //  hash lcs_id -> (seg_id, word_id)
  std::vector<index> hash_t, hash_o;
  std::vector<std::string> comp_str_t, comp_str_o;
  for (size_t seg_id = stale_segment_; seg_id < segments_.size(); ++seg_id) {
    for (size_t word_id = 0; word_id < segments_[seg_id].words_.size(); ++word_id) {
      std::string comp_word = segments_[seg_id].words_[word_id].get_comparable();
      if ( !comp_word.empty() ) {
        comp_str_t.push_back(comp_word);
        hash_t.push_back({static_cast<int>(seg_id), static_cast<int>(word_id)});
      }
    }
  }
  for (size_t seg_id = 0; seg_id < other.size(); ++seg_id) {
    for (size_t word_id = 0; word_id < other[seg_id].words_.size(); ++word_id) {
      std::string comp_word = other[seg_id].words_[word_id].get_comparable();
      if ( !comp_word.empty() ) {
        comp_str_o.push_back(comp_word);
        hash_o.push_back({static_cast<int>(seg_id), static_cast<int>(word_id)});
      }
    }
  }

  // Longest Common Substring with Gaps.
  auto [indicies_t, indicies_o] = lcs_indicies_(comp_str_t, comp_str_o, allowed_gaps_);
  if ( indicies_t.empty() ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "[LCS] - No overlap in substring");
    push_back(other);
    return;
  }

  // 
  // Merge words and segments
  // 
  Transcript::Operations pending_ops;

  // Helper lambda function
  auto check_segment = [](const index &id, const std::vector<Segment> &segs) -> bool {
    return static_cast<size_t>(id.second) >= segs[id.first].words_.size();
  };
  auto get_word = [](const index &id, const std::vector<Segment> &segs) -> const Word& {
    return segs[id.first].words_[id.second];
  };

  // Start at first matched word
  index cur_t = hash_t[indicies_t[0]], cur_o = hash_o[indicies_o[0]];
  for(size_t i = 1; i <= indicies_t.size(); ++i) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "[LCS]\tMatch:   '%s' -- and -- '%s'", 
                                            get_word(cur_t, segments_).get().c_str(),
                                            get_word(cur_o, other).get().c_str());
    pending_ops.push_back({Transcript::OperationType::CONFLICT, cur_t, cur_o});
    pending_ops.push_back({Transcript::OperationType::INCREMENT, cur_t});

    // Current index "i" may not be valid
    index next_t, next_o;
    if ( i == indicies_t.size() ) {
      // Next index is one past final word
      next_t = {segments_.size()-1, segments_[segments_.size()-1].words_.size()};
      next_o = {other.size()-1, other[other.size()-1].words_.size()};
    } else {
      next_t = hash_t[indicies_t[i]]; next_o = hash_o[indicies_o[i]];
    }

    // Move to next word
    if ( cur_t != next_t ) { ++cur_t.second; };
    if ( cur_o != next_o ) { ++cur_o.second; };

    RCLCPP_DEBUG(node_ptr_->get_logger(), 
                "[LCS]\tNext Transcript Index:   (%d, %d).  Next Other Index:  (%d, %d)", 
                next_t.first, next_t.second, next_o.first, next_o.second);
    while ( cur_t != next_t || cur_o != next_o ) {
      RCLCPP_DEBUG(node_ptr_->get_logger(), 
                "[LCS]\t\tCurrent Transcript Index:   (%d, %d).  Current Other Index:  (%d, %d)", 
                cur_t.first, cur_t.second, cur_o.first, cur_o.second);

      // Check if we encountered segment boundary
      bool seg_next_t = cur_t != next_t && check_segment(cur_t, segments_);
      bool seg_next_o = cur_o != next_o && check_segment(cur_o, other);
      if ( seg_next_t && seg_next_o ) {
        ++cur_t.first; ++cur_o.first;
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
          "[LCS]\t\t\tMatching Segment boundary --- %s (:existing) v.s. (new:) %s", 
          segments_[cur_t.first].as_timestamp_str().c_str(), 
          other[cur_o.first].as_timestamp_str().c_str());

        pending_ops.push_back({Transcript::OperationType::MERGE_SEG, cur_t, cur_o});
        cur_t.second = 0; cur_o.second = 0;
        continue;
      }
      else if ( seg_next_t && !seg_next_o ) {
        ++cur_t.first;
        cur_t.second = 0;
        // Segment boundary in transcript but not in new words.  Decrease likelihood of segment
        pending_ops.push_back({Transcript::OperationType::DEC_SEG, cur_t});
        RCLCPP_DEBUG(node_ptr_->get_logger(), "[LCS]\t\t\tExtra Segment boundary --- %s", 
                                              segments_[cur_t.first].as_timestamp_str().c_str());
        continue;
      }
      else if ( !seg_next_t && seg_next_o ) {
        // Segment boundary in update missing in transcript.  Insert new segment
        ++cur_o.first;
        cur_o.second = 0;
        pending_ops.push_back(
            {Transcript::OperationType::INSERT_SEG, cur_t, cur_o});
        RCLCPP_DEBUG(node_ptr_->get_logger(), "[LCS]\t\t\tMissing Segment boundary --- %s", 
                                                  other[cur_o.first].as_timestamp_str().c_str());
        continue;
      }
      // 
      // Custom Merge Rules
      //    Handle inserting/decrementing/conflicting gaps in the lcs
      // 
      // 1.  Encourage over-writing punctuation in the transcript (if the update is a word)
      if ( cur_t != next_t && cur_o != next_o && 
            get_word(cur_t, segments_).is_punct() && !get_word(cur_o, other).is_punct() ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                          "[LCS]\t\t\tOverwrite Punctuation:   '%s' (:overwrite) v.s. (new:) '%s'", 
                          get_word(cur_t, segments_).get().c_str(),
                          get_word(cur_o, other).get().c_str());

        pending_ops.push_back({Transcript::OperationType::DECREMENT, cur_t});
        pending_ops.push_back({Transcript::OperationType::CONFLICT, cur_t, cur_o});
      }
      // 2.  Conflict when there is a gap because of missmatched words in the LCS
      else if ( cur_t != next_t && cur_o != next_o ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                          "[LCS]\t\t\tConflict:   '%s' (:existing) v.s. (new:) '%s'", 
                          get_word(cur_t, segments_).get().c_str(),
                          get_word(cur_o, other).get().c_str());

        pending_ops.push_back({Transcript::OperationType::CONFLICT, cur_t, cur_o});
      }
      // 3.  Words appear in the audio steam (update) which are not part of the transcript
      else if ( cur_o != next_o ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                          "[LCS]\t\t\tInsert:  '%s'", 
                          get_word(cur_o, other).get().c_str());

        pending_ops.push_back({Transcript::OperationType::INSERT, cur_t, cur_o});
      }
      // 4.  Words in the transcript are missing from the update
      else {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                          "[LCS]\t\t\tDecrement:  '%s'", 
                          get_word(cur_t, segments_).get().c_str());

        pending_ops.push_back({Transcript::OperationType::DECREMENT, cur_t});
      }
      // Custom Merge Rules Finished
      // 
      // Move to next word
      if ( cur_t != next_t ) { ++cur_t.second; };
      if ( cur_o != next_o ) { ++cur_o.second; };
    }
    // Prep for next loop. 
    // cur_o and cur_t should already be equal to the next values.
  }

  run(pending_ops, other);
  clear_mistakes(-1);
}


std::tuple<std::vector<int>, std::vector<int>> Transcript::lcs_indicies_(
                                                  const std::vector<std::string>& textA,
                                                  const std::vector<std::string>& textB,
                                                  int allowedGaps) {
  int nA = textA.size();
  int nB = textB.size();

  // 2D DP table initialized to DPEntry(0, 0)
  std::vector<std::vector<DPEntry>> dp(nA + 1, std::vector<DPEntry>(nB + 1, {0, 0}));
  std::vector<std::vector<std::pair<int, int>>> 
                      prev(nA + 1, std::vector<std::pair<int, int>>(nB + 1, {-1, -1}));

  int maxLength = 0;
  int endIndexA = -1, endIndexB = -1;

  // Fill DP table
  for (int i = 1; i <= nA; ++i) {
    for (int j = 1; j <= nB; ++j) {
      if (textA[i-1] == textB[j-1]) {
        dp[i][j] = {dp[i-1][j-1].length + 1, 0};
        prev[i][j] = {i-1, j-1};
      } else {
        // Case 1: skip one element from textA
        if (dp[i-1][j].gaps < allowedGaps && dp[i][j].length < dp[i-1][j].length) {  
          dp[i][j] = {dp[i-1][j].length, dp[i-1][j].gaps + 1};
          prev[i][j] = prev[i-1][j];
        }

        // Case 2: skip one element from textB
        if (dp[i][j-1].gaps < allowedGaps && dp[i][j].length < dp[i][j-1].length) {
          dp[i][j] = {dp[i][j-1].length, dp[i][j-1].gaps + 1};
          prev[i][j] = prev[i][j-1];
        }

        // Case 3: skip one element from textA AND textB
        if (dp[i-1][j-1].gaps < allowedGaps && dp[i][j].length < dp[i-1][j-1].length) {
          dp[i][j] = {dp[i-1][j-1].length, dp[i-1][j-1].gaps + 1};
          prev[i][j] = prev[i-1][j-1];
        }
      }

      // Track the maximum length
      if (dp[i][j].length >= maxLength) {
        maxLength = dp[i][j].length;
        endIndexA = i;
        endIndexB = j;
      }
    }
  }

  if (maxLength == 0) {
    return {{}, {}};
  }
  
  // Backtrack to find the longest matching subsequence
  std::vector<int> resultA, resultB;
  std::tie(endIndexA, endIndexB) = prev[endIndexA][endIndexB];
  while (endIndexA != -1 && endIndexB != -1) {
    resultA.push_back(endIndexA);  
    resultB.push_back(endIndexB);
    std::tie(endIndexA, endIndexB) = prev[endIndexA][endIndexB];
  }

  std::reverse(resultA.begin(), resultA.end());
  std::reverse(resultB.begin(), resultB.end());

  return {resultA, resultB};
}

} // end of namespace whisper

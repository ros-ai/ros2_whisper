#include "transcript_manager/transcript.hpp"

namespace whisper {

void Transcript::merge_one_(const std::vector<Word> &new_words) {
  auto stale_id = get_stale_word_id();
  
  std::string tmp_print_str_1_;
  std::string tmp_print_str_2_;

  if (empty()) {
    push_back(new_words);
    RCLCPP_DEBUG(node_ptr_->get_logger(), "First Words Added");
    return;
  }

  // Get comparable strings for fuzzy lcs matching
  auto old_words = get_words_splice();
  std::vector<std::string> comp_words_old, comp_words_new;
  std::vector<int> skipped_ids_old, skipped_ids_new;
  int skipped_so_far = 0;
  for (const auto &word : old_words) {
    const auto &comp_word = word.get_comparable();
    if ( comp_word.empty() ) {
      skipped_so_far++;
    } else {
      comp_words_old.push_back(comp_word);  
      skipped_ids_old.push_back(skipped_so_far);
      tmp_print_str_1_ += "'" + comp_words_old[comp_words_old.size() - 1] + "', ";
    }
  }
  skipped_so_far = 0;
  for (size_t i = 0; i < new_words.size(); ++i) {
    const auto &comp_word = new_words[i].get_comparable();
    if ( comp_word.empty() ) {
      skipped_so_far++;
    } else {
      comp_words_new.push_back(comp_word);  
      skipped_ids_new.push_back(skipped_so_far);
      tmp_print_str_2_ += "'" + comp_words_new[comp_words_new.size() - 1] + "', ";
    }
  }
  RCLCPP_DEBUG(node_ptr_->get_logger(), " ");
  RCLCPP_DEBUG(node_ptr_->get_logger(), "Comp Against:  %s", tmp_print_str_1_.c_str());
  RCLCPP_DEBUG(node_ptr_->get_logger(), "   New Words:  %s", tmp_print_str_2_.c_str());

  // Longest Common Substring with Gaps.
  //   A:  Old words (already in Transcript), B:  New words recieved from live feed
  auto [indiciesA, indiciesB] = lcs_indicies_(comp_words_old, comp_words_new, allowed_gaps_);
  if ( indiciesA.empty() ) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "  ---No overlap");
    push_back(new_words);
    return;
  }
  
  // Merge words and segments
  Transcript::Operations pending_ops;

  auto prevA = indiciesA[0], prevB = indiciesB[0];
  for(size_t i = 1; i <= indiciesA.size(); ++i) {
    // Include the offsets from skipped words
    auto prevA_id = prevA + skipped_ids_old[prevA];
    auto prevB_id = prevB + skipped_ids_new[prevB];
    RCLCPP_DEBUG(node_ptr_->get_logger(), "\tPrevA: %d,  PrevB:  %d:   %s (%f\\%d)", 
                                            prevA_id, prevB_id, 
                                            old_words[prevA_id].get().c_str(),
                                            old_words[prevA_id].get_prob(),
                                            old_words[prevA_id].get_occurrences());

    pending_ops.push_back({Transcript::OperationType::MERGE, prevA_id, prevB_id});
    pending_ops.push_back({Transcript::OperationType::INCREMENT, prevA_id});

    // Current index "i" may not be valid
    int curA_id = prevA_id + 1, curB_id = prevB_id + 1;
    int nextA_id, nextB_id;
    if ( i == indiciesA.size() ) {
      // The following merge rules will run to the end of the new_words and old_words array.
      // Most commonly, new words that do not exist in the transcript are inserted at the end.
      // TODO:  Apply these rules to the begining (before first LCS Match)
      nextA_id = old_words.size(), nextB_id = new_words.size();
    } else {
      nextA_id = indiciesA[i] + skipped_ids_old[indiciesA[i]];
      nextB_id = indiciesB[i] + skipped_ids_new[indiciesB[i]];
    }
    // RCLCPP_INFO(node_ptr_->get_logger(), "\tNextA: %d,  NextB:  %d", nextA_id, nextB_id);
    while (curA_id != nextA_id || curB_id != nextB_id) {
      // RCLCPP_INFO(node_ptr_->get_logger(), "\t\tCurA: %d,  CurB:  %d", curA_id, curB_id);
      // 
      // Custom Merge Rules
      // 
      // 0:  Handle Segments
      bool is_a_seg = curA_id != nextA_id && old_words[curA_id].is_segment();
      bool is_b_seg = curB_id != nextB_id && new_words[curB_id].is_segment();
      if ( is_a_seg && is_b_seg ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tMerging segments '%s' and '%s' at %d",
                                            old_words[curA_id].as_timestamp_str().c_str(), 
                                            new_words[curB_id].as_timestamp_str().c_str(), 
                                            curA_id);

        pending_ops.push_back({Transcript::OperationType::MERGE, curA_id, curB_id});
        ++curA_id; ++curB_id;
        continue;
      }
      else if ( is_a_seg && !is_b_seg ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tReducing likelihood of segment '%s' at %d",
                                            old_words[curA_id].as_timestamp_str().c_str(), 
                                            curA_id);

        pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id});
        ++curA_id;
        continue;
      }
      else if ( !is_a_seg && is_b_seg ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tInserting segment '%s' at '%d'",
                                            new_words[curB_id].as_timestamp_str().c_str(), 
                                            curA_id);

        pending_ops.push_back({Transcript::OperationType::INSERT, curA_id, curB_id});
        ++curB_id;
        continue;
      }
      // Neither are segments, continue loop

      // 1.  Encourage over-writing punctuation in the transcript (if the update is a word)
      if ( curA_id != nextA_id && curB_id != nextB_id && 
            old_words[curA_id].is_punct() && ! new_words[curB_id].is_punct() ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
          "\t\tWord Conflict Transcript (punct) vs update (word).  '%s' (%f\\->%d) --> '%s'",
                                            old_words[curA_id].get().c_str(),
                                            old_words[curA_id].get_prob(),
                                            old_words[curA_id].get_occurrences()-1,
                                            new_words[curB_id].get().c_str());

        pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id});
        pending_ops.push_back({Transcript::OperationType::CONFLICT, curA_id, curB_id});
        curA_id++; curB_id++;
      }
      // 1.2  Conflict when there is a gap because of missmatched words in the LCS
      else if ( curA_id != nextA_id && curB_id != nextB_id)  {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                          "\t\tResolve Conflict Between '%s'(%f\\%d) and '%s'(%f\\%d)",
                                                  old_words[curA_id].get().c_str(), 
                                                  old_words[curA_id].get_prob(), 
                                                  old_words[curA_id].get_occurrences(), 
                                                  new_words[curB_id].get().c_str(), 
                                                  new_words[curB_id].get_prob(),
                                                  new_words[curB_id].get_occurrences());

        // If we have a conflict, the word's likely-hood could be decreased.
        //    - Removed:  This causes some issues with words that sound the same 
        //            i.e. are constantly in conflict.
        // pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id}); // Removed 
        pending_ops.push_back({Transcript::OperationType::CONFLICT, curA_id, curB_id});
        curA_id++; curB_id++;
      }
      // 1.3  Words appear in the audio steam (update) which are not part of the transcript
      else if ( curB_id != nextB_id ) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "\t\tInserting word '%s' -- Between '%s' and '%s'",
                                            new_words[curB_id].get().c_str(), 
                                            old_words[curA_id-1].get().c_str(), 
                                            curA_id == static_cast<int>(old_words.size()) ? 
                                                        "END" : old_words[curA_id].get().c_str());

        pending_ops.push_back({Transcript::OperationType::INSERT, curA_id, curB_id});
        curB_id++;
      }
      // 1.4 Words in the transcript are missing from the update
      else {
        RCLCPP_DEBUG(node_ptr_->get_logger(), 
                                  "\t\tDecreasing Likelihood of word:  '%s' (%f\\%d->%d)", 
                                                old_words[curA_id].get().c_str(),
                                                old_words[curA_id].get_prob(),
                                                old_words[curA_id].get_occurrences(),
                                                old_words[curA_id].get_occurrences() - 1);

        pending_ops.push_back({Transcript::OperationType::DECREMENT, curA_id, -1});
        curA_id++;
      }
    }
    // Prep for next loop.  Move prevA and prevB to the next matching word
    prevA = indiciesA[i]; prevB = indiciesB[i];
  }

  run(pending_ops, new_words);
  clear_mistakes(-1);

  auto stale_id_new = std::max(stale_id, stale_id + indiciesA[0] - indiciesB[0]);
  RCLCPP_DEBUG(node_ptr_->get_logger(), "Stale id update %d -> %d", stale_id, stale_id_new );
  set_stale_word_id(stale_id_new);
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
    // RCLCPP_INFO(node_ptr_->get_logger(), "Word i: %s", textA[i-1].c_str());
    for (int j = 1; j <= nB; ++j) {
      // RCLCPP_INFO(node_ptr_->get_logger(), "\tWord j: %s", textB[j-1].c_str());
      if (textA[i-1] == textB[j-1]) {
        // RCLCPP_INFO(node_ptr_->get_logger(), "\t\tMatch");
        dp[i][j] = {dp[i-1][j-1].length + 1, 0};
        prev[i][j] = {i-1, j-1};
      } else {
        // Case 1: skip one element from textA
        if (dp[i-1][j].gaps < allowedGaps && dp[i][j].length < dp[i-1][j].length) {  
          // RCLCPP_INFO(node_ptr_->get_logger(), "\t\t Drop word from A");
          dp[i][j] = {dp[i-1][j].length, dp[i-1][j].gaps + 1};
          prev[i][j] = prev[i-1][j];
        }

        // Case 2: skip one element from textB
        if (dp[i][j-1].gaps < allowedGaps && dp[i][j].length < dp[i][j-1].length) {
          // RCLCPP_INFO(node_ptr_->get_logger(), "\t\t Drop word from B");
          dp[i][j] = {dp[i][j-1].length, dp[i][j-1].gaps + 1};
          prev[i][j] = prev[i][j-1];
        }

        // Case 3: skip one element from textA AND textB
        if (dp[i-1][j-1].gaps < allowedGaps && dp[i][j].length < dp[i-1][j-1].length) {
          // RCLCPP_INFO(node_ptr_->get_logger(), "\t\t Drop word from A AND B");
          dp[i][j] = {dp[i-1][j-1].length, dp[i-1][j-1].gaps + 1};
          prev[i][j] = prev[i-1][j-1];
        }
      }
      // RCLCPP_INFO(node_ptr_->get_logger(), "\t\tDP[%d][%d] Length: %d  Gap: %d", 
      //             i, j, dp[i][j].length, dp[i][j].gaps);

      // Track the maximum length
      if (dp[i][j].length >= maxLength) {
        maxLength = dp[i][j].length;
        endIndexA = i;
        endIndexB = j;
        // RCLCPP_INFO(node_ptr_->get_logger(), " ---- New Best Length: %d  endpoint A: %d   endpoint B: %d", 
        //             maxLength, endIndexA, endIndexB);
      }
    }
  }

  if (maxLength == 0) {
    return {{}, {}};
  }
  
  // Backtrack to find the longest matching subsequence
  std::string print_str = "Backtrack pairs: ";
  std::vector<int> resultA, resultB;
  std::tie(endIndexA, endIndexB) = prev[endIndexA][endIndexB];
  while (endIndexA != -1 && endIndexB != -1) {
    print_str += "(" + std::to_string(endIndexA) + "," + std::to_string(endIndexB) + "), ";
    resultA.push_back(endIndexA);  
    resultB.push_back(endIndexB);
    std::tie(endIndexA, endIndexB) = prev[endIndexA][endIndexB];
  }
  // RCLCPP_INFO(node_ptr_->get_logger(), "%s", print_str.c_str());

  std::reverse(resultA.begin(), resultA.end());
  std::reverse(resultB.begin(), resultB.end());

  return {resultA, resultB};
}

} // end of namespace whisper

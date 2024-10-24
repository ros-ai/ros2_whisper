#include "whisper_util/whisper.hpp"

namespace whisper {
Whisper::Whisper() { wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY); }

Whisper::Whisper(const std::string &model_path) { initialize(model_path); }

Whisper::~Whisper() { whisper_free(ctx); }

void Whisper::initialize(const std::string &model_path) {
  ctx = whisper_init_from_file_with_params(model_path.c_str(), cparams);
}

std::string Whisper::forward(const std::vector<float> &input) {
  if (whisper_full(ctx, wparams, input.data(), input.size()) != 0) {
    return {};
  }
  std::vector<std::string> segments;
  int n_segments = whisper_full_n_segments(ctx);
  for (int i = 0; i < n_segments; ++i) {
    segments.push_back(whisper_full_get_segment_text(ctx, i));
  }
  return std::accumulate(segments.begin(), segments.end(), std::string());
}

std::vector<whisper_token> Whisper::tokens() {
  std::vector<whisper_token> tokens;
  const int n_segments = whisper_full_n_segments(ctx);
  for (int i = 0; i < n_segments; ++i) {
    const int token_count = whisper_full_n_tokens(ctx, i);
    for (int j = 0; j < token_count; ++j) {
      tokens.push_back(whisper_full_get_token_id(ctx, i, j));
    }
  }
  return tokens;
}


void Whisper::forward_tokenize(
                  const std::vector<float> &input,
                  std::vector<int> &token_ids,
                  std::vector<std::string> &token_texts,
                  std::vector<float> &token_probs,
                  std::vector<int> &segment_start_token_idx,
                  std::vector<int64_t> &segment_start_timestamp,
                  std::vector<int64_t> &segment_end_timestamp
                ) {
  // Perform whisper inference
  if (whisper_full(ctx, wparams, input.data(), input.size()) != 0) {
    return;
  }

  // Calculate the total number of tokens across all segments
  int total_tokens = 0;
  int n_segments = whisper_full_n_segments(ctx);
  for (int i = 0; i < n_segments; ++i) {
    total_tokens += whisper_full_n_tokens(ctx, i);
  }

  // Reserve memory for vectors to avoid reallocations
  token_ids.reserve(total_tokens);
  token_texts.reserve(total_tokens);
  token_probs.reserve(total_tokens);
  segment_start_token_idx.reserve(n_segments);
  segment_start_timestamp.reserve(n_segments);
  segment_end_timestamp.reserve(n_segments);

  // Load data
  int segment_start_token_counter = 0;
  for (int i = 0; i < n_segments; ++i) {
    segment_start_token_idx.push_back(segment_start_token_counter);
    segment_start_timestamp.push_back(whisper_full_get_segment_t0(ctx, i));
    segment_end_timestamp.push_back(whisper_full_get_segment_t1(ctx, i));

    // Get token level data
    const int token_count = whisper_full_n_tokens(ctx, i);
    for (int j = 0; j < token_count; ++j) {
      token_ids.push_back(whisper_full_get_token_id(ctx, i, j));
      token_texts.push_back(whisper_full_get_token_text(ctx, i, j));
      token_probs.push_back(whisper_full_get_token_p(ctx, i, j));
      segment_start_token_counter++;
    }
  }
}



} // end of namespace whisper

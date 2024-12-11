#ifndef TRANSCRIPT_MANAGER__TOKENS_HPP_
#define TRANSCRIPT_MANAGER__TOKENS_HPP_

#include <string>
#include <stdexcept>

namespace whisper {

/**
 * @brief Deserialized data of the token from the WhisperTokens.msg
 */
class SingleToken {
private:
  std::string data_;
  float prob_;

public:
  std::string get_data() const {
    return data_;
  }

  float get_prob() const {
    return prob_;
  }

  SingleToken(const std::string& data_, float prob_)
        : data_(data_), prob_(prob_) {};
};

} // end of namespace whisper
#endif // TRANSCRIPT_MANAGER__TOKENS_HPP_

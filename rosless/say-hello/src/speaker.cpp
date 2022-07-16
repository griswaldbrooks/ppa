#include "say-hello/speaker.hpp" // For speaker
#include <string>      // For string

namespace sh {


speaker::speaker(std::string const& phrase):sentence_{
  "I want " + phrase
}{}

  std::string const& speaker::operator()() const noexcept {
    return sentence_;
  }


}  // namespace sh

#pragma once

#include <string>      // For string

namespace sh {

struct speaker {
  /// \brief Decorates a phrase.
  /// \param phrase to be decorated.
  explicit speaker(std::string const& phrase);
  /// \returns the decorated phrase.
  std::string const& operator()() const noexcept;
 private:
  /// \brief Decorated phrase.
  std::string sentence_;
};

}  // namespace sh

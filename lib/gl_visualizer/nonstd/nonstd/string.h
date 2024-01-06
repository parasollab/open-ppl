#ifndef NONSTD_STRING_H_
#define NONSTD_STRING_H_

#include <string>
#include <vector>

namespace nonstd {

  ///@name String
  ///@{

  /// Trim any whitespace at the front or back of a string.
  /// @param[in] _s The string to trim.
  /// @return       A trimmed copy of the input string.
  std::string trim(const std::string& _s);

  /// Break up a string into tokens separated by one or more delimiters.
  /// @param[in] _s The string to tokenize.
  /// @param[in] _d A string of concatenated delimiters. Default is ", ".
  /// @param[in] _t Trim tokens after splitting?
  /// @return A vector holding the resulting tokens.
  std::vector<std::string> tokenize(std::string _s, const std::string& _d = ", ",
      const bool _t = 1);

  ///@}

}

#endif

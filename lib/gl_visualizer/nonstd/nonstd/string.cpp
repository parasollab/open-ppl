#include <regex>

#include "nonstd/string.h"

using namespace std;

namespace nonstd {

  string
  trim(const string& _s)
  {
    // Create a regex for the trim pattern.
    static const regex r_t(R"(^\s*|\s*$)");
    return regex_replace(_s, r_t, string(""));
  }


  vector<string>
  tokenize(string _s, const string& _d, const bool _t)
  {
    // Create a regex for the delimiter pattern.
    regex r_d("[^" + _d + "]+");

    // Tokenize the string by grouping all characters separated by any character
    // in the delimiter string.
    vector<string> tokens;
    smatch m;

    while(regex_search(_s, m, r_d)) {
      tokens.push_back(m.str());
      if(_t) tokens.back() = trim(tokens.back());
      _s = m.suffix().str();
    }

    return tokens;
  }

}

#include <fstream>

#include "nonstd/io.h"
#include "nonstd/runtime.h"

using namespace std;

namespace nonstd {

  vector<string>
  read_file_lines(
      const string& _filename
  ) {
    // Open the file.
    ifstream file(_filename);
    if(!file.is_open())
      throw nonstd::exception(WHERE) << "Could not read file '" << _filename
                                     << "'.";

    // Read the file.
    string line;
    vector<string> lines;
    while(getline(file, line))
      lines.push_back(line);

    // Close the file and return the contents.
    file.close();
    return lines;
  }


  std::string
  read_file(
      const std::string& _filename
  ) {
    // Open the file.
    std::ifstream file(_filename);
    if(!file.is_open())
      throw nonstd::exception(WHERE) << "Could not read file '" << _filename
                                     << "'.";

    // Reserve space. ifstream insists on creating a fake extra character for
    // the 'end of file', but only when the file contains at least one
    // character. Handle this stupidity here.
    std::string contents;
    file.seekg(0, std::ios::end);
    const size_t num_chars = size_t(file.tellg()) - 1;
    if(num_chars == size_t(-1))
      return {};
    contents.reserve(num_chars);
    file.seekg(0, std::ios::beg);

    auto iter = std::istreambuf_iterator<char>(file);
    for(size_t i = 0; i < num_chars; ++i, ++iter)
      contents.push_back(*iter);

    return contents;
  }


  char
  matching_bracket(
      const char _bracket
  ) {
    switch(_bracket)
    {
      case '(':
        return ')';
      case ')':
        return '(';
      case '[':
        return ']';
      case ']':
        return '[';
      case '{':
        return '}';
      case '}':
        return '{';
      case '<':
        return '>';
      case '>':
        return '<';
      default:
        throw nonstd::exception(WHERE) << "Unrecognized bracket '" << _bracket
                                       << "', choices are (){}[]<>.";
    }
  }

}

#include <algorithm>
#include <iostream>
#include <regex>

#include "nonstd/options_manager.h"
#include "nonstd/runtime.h"

using namespace std;

namespace nonstd {

  /*--------------------------- Construction ---------------------------------*/

  options_manager::
  options_manager(int _argc, char* _argv[])
  {
    // Tokenize the argument list, skipping the first token which is the
    // executable name.
    vector<string> tokens(_argc - 1);
    for(int i = 1; i < _argc; ++i)
      tokens[i - 1] = _argv[i];

    // Parse the tokens into flags and arguments.
    if(!tokens.empty())
      parse_options(move(tokens));
  }


  void
  options_manager::
  initialize()
  {
    validate_options();
  }

  /*-------------------------- Public Interface ------------------------------*/

  bool
  options_manager::
  has_flag(const std::string& _flag) const
  {
    return m_parsed_options.count(_flag);
  }


  const vector<string>
  options_manager::
  get_flags() const
  {
    vector<string> flags;
    for(const auto& opt : m_parsed_options)
      flags.push_back(opt.first);
    return flags;
  }


  const string&
  options_manager::
  get_arg(const string& _flag) const
  {
    auto opt = m_parsed_options.find(_flag);
    assert_msg(opt != m_parsed_options.end(), "nonstd::options_manager error: "
        "could not find requested flag '" + _flag + "'!");

    return opt->second;
  }

  /*----------------------- Configuration Functions --------------------------*/

  void
  options_manager::
  add_flag(const string& _flag)
  {
    add_flag(_flag, vector<string>());
  }


  void
  options_manager::
  add_flag(const string& _flag, const string& _pattern)
  {
    add_flag(_flag, vector<string>{_pattern});
  }


  void
  options_manager::
  add_flag(const string& _flag, vector<string>&& _allowed)
  {
    assert_msg(m_supported_options.count(_flag) == 0, "nonstd::options_manager "
        "error: tried to add existing flag '" + _flag + "'!");
    assert_msg(is_flag(_flag), "nonstd::options_manager error: tried to add "
        "invalid flag '" + _flag + "'!");

    m_supported_options[_flag] = _allowed;
  }

  /*------------------------- Parsing Functions ------------------------------*/

  bool
  options_manager::
  is_flag(const string& _s) const
  {
    /// A token is considered a flag if:
    /// \arg 1. it has the form '-x', where x is any alpha-numeric character.
    /// \arg 2. it has the form '--x...', where x... is any sequence of
    ///         alpha-numeric characters.
    return _s.size() && _s[0] == '-' &&
        (
          _s.size() == 2 && isalnum(_s[1]) ||
          _s[1] == '-' && all_of(_s.begin() + 2, _s.end(),
                                 [](char c){return isalnum(c);})
        );
  }


  void
  options_manager::
  parse_options(vector<string>&& _opts)
  {
    // Validate that the first token is a flag. Otherwise, the entire option list
    // is unrecognized.
    auto i = _opts.begin();
    assert_msg(is_flag(*i), "nonstd::options_manager error: first argument "
        "token '" + *i + "' is not a valid flag!");

    // Parse the tokens. Each token is mapped to the closest preceding flag.
    string* current = nullptr;
    for(; i != _opts.end(); ++i) {
      if(is_flag(*i))
        current = &m_parsed_options[*i];
      else
        *current = *i;
    }
  }


  void
  options_manager::
  validate_options() const
  {
    // Check each parsed option.
    for(const auto& opt : m_parsed_options) {
      const auto& flag = opt.first;
      const auto& arg  = opt.second;

      string err = "nonstd::options_manager error: flag '" + flag + "' ";

      // Ensure this flag is supported.
      assert_msg(m_supported_options.count(flag), err + "not recognized!");

      // Validate the arguments.
      const auto& allowed = m_supported_options.at(flag);
      switch(allowed.size()) {
        case 0:
          assert_msg(arg == string(), err + "does not support arguments, "
              "but '" + arg + "' was provided!");
          break;
        case 1:
          assert_msg(regex_match(arg, regex(allowed[0])), err + "only "
              "supports arguments that match the regex '" + allowed[0] + "'!");
          break;
        default:
          assert_msg(find(allowed.begin(), allowed.end(), arg) != allowed.end(),
              err + "does not support the argument value '" + arg + "'!");
      }
    }
  }

  /*--------------------------------------------------------------------------*/

}

/*--------------------------------- Display ----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const nonstd::options_manager& _om)
{
  _os << "Parsed options:" << std::endl;
  for(const auto& flag : _om.get_flags())
    _os << "\t" << flag << " " << _om.get_arg(flag) << std::endl;
  return _os;
}

/*----------------------------------------------------------------------------*/

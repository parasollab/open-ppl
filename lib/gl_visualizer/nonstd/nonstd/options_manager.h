#ifndef NONSTD_OPTIONS_MANAGER_H_
#define NONSTD_OPTIONS_MANAGER_H_

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// Parses and manages command-line arguments.
  //////////////////////////////////////////////////////////////////////////////
  class options_manager {

    ///@name Internal State
    ///@{

    std::map<std::string, std::vector<std::string>> m_supported_options;
    std::map<std::string, std::string> m_parsed_options;

    ///@}

    public:

      ///@name Construction
      ///@{

      options_manager(int _argc, char* _argv[]);
      virtual ~options_manager() = default;

      ///@}
      ///@name Flag Interface
      ///@{

      /// Check if a given flag was provided.
      /// @param[in] _flag The flag to check.
      /// @return True if the flag was specified on the command line.
      bool has_flag(const std::string& _flag) const;

      /// Get the parsed flags.
      /// @return A listing of all flags that were specified on the command line.
      const std::vector<std::string> get_flags() const;

      /// Get the argument passed with a given flag.
      /// @param[in] _flag The flag of interest.
      /// @return The the command line argument provided for _flag.
      const std::string& get_arg(const std::string& _flag) const;

      ///@}

    protected:

      ///@name Configuration Functions
      ///@{
      /// These functions allow derived classes to configure the allowed options
      /// and validate them during construction.

      /// Finalize the construction of this object. This function should be
      /// called at the end of each derived class's constructor.
      void initialize();

      /// Add a flag for an option with no arguments to the supported options.
      /// @param[in] _flag The flag token for this option.
      void add_flag(const std::string& _flag);

      /// Add a flag for an option with a single, pattern-matched argument value
      /// to the supported options.
      /// @param[in] _flag The flag token for this option.
      /// @param[in] _pattern The pattern to match the argument against.
      void add_flag(const std::string& _flag, const std::string& _pattern);

      /// Add a flag for an option with an enumerated set of allowed argument
      /// values to the supported options.
      /// @param[in] _flag The flag token for this option.
      /// @param[in] _allowed The argument type for this option.
      void add_flag(const std::string& _flag,
          std::vector<std::string>&& _allowed);

      ///@}

    private:

      ///@name Parsing Functions
      ///@{

      /// Test whether a given token is a posix-style flag.
      /// @param[in] _s The token to check.
      /// @return True if _s is a posix-style flag.
      bool is_flag(const std::string& _s) const;

      /// Parse the tokenized options into flag/argument pairs.
      /// @param[in] _opts The tokenized argument list.
      void parse_options(std::vector<std::string>&& _opts);

      /// Assert that the parsed options are valid instances of the supported
      /// options.
      void validate_options() const;

      ///@}
  };

}

/*--------------------------------- Display ----------------------------------*/

std::ostream& operator<<(std::ostream& _os, const nonstd::options_manager& _om);

/*----------------------------------------------------------------------------*/

#endif

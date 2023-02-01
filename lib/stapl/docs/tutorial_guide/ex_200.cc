#include <iostream> // ## 1
#include <fstream>
#include <string>

#include <stapl/utility/do_once.hpp> // ## 2
#include <stapl/runtime.hpp> // ## 3

using namespace std; // ## 4

struct msg {
private:
  string m_txt;
public:
  msg(string text)  // ## 5
    : m_txt(text)
  { }

  typedef void result_type;
  result_type operator() () { 
    cout << m_txt << endl; // ## 6
  }
  void define_type(stapl::typer& t) 
  {
    t.member(m_txt);
  }
};

template<typename Value>
struct msg_val {
private:
  string m_txt;
  Value m_val;
public:
  msg_val(string text, Value val) // ## 7
    : m_txt(text), m_val(val)
  { }

  typedef void result_type;
  result_type operator() () {
    cout << m_txt << " " << m_val << endl; // ## 8
  }
  void define_type(stapl::typer& t) 
  {
    t.member(m_txt);
    t.member(m_val);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) { // ## 9
  stapl::do_once( msg( "Example 200" ) ); // ## 10

  stapl::do_once( msg_val<int>( "The answer is ", 42 ) ); // ## 11

  cout << "Hello World\n"; // ## 12

  return EXIT_SUCCESS;
}

/*
  // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
  // component of the Texas A&M University System.

  // All rights reserved.

  // The information and source code contained herein is the exclusive
  // property of TEES and may not be disclosed, examined or reproduced
  // in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <string>
#include <stapl/paragraph/paragraph.hpp>
#include <boost/tokenizer.hpp>
#include <stapl/algorithms/MapReduce/MapReduce.hpp>

using namespace std;
using namespace stapl;
using namespace boost;

struct myTask : mr_task<string, int>
{
  void setup(string infile, string outfile, int split_size, string delimiter)
  {
    set_input_file(infile);
    set_output_file(outfile);
    set_split_size(split_size);
    set_delimiter(delimiter);
  }

  void parse(string const& text)
  {
    char_separator<char> sep(
      "/\r?\n/ ,./<>?;\':\"\\[]{}|-`~!@#$%^&*()_+= \t \r"
    );
    typedef tokenizer<char_separator<char> > tokenizer;
    tokenizer tokens(text, sep);
    for (tokenizer::iterator tok_iter = tokens.begin();
         tok_iter != tokens.end(); ++tok_iter)
      parser_emit(*tok_iter);
  }

  pair<string, int> map(string const& elem)
  {
    return make_pair(elem, 1);
  }

  pair<string, int> reduce(pair<string, int> p_old, pair<string, int> p_new)
  {
    return make_pair(p_old.first, p_old.second + p_new.second);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (get_location_id() == 0)
    cout << "testing wordcount on STAPL MapReduce...\t";

  string infname = string(argv[1]);
  string outfname = "results";
  myTask task;
  task.setup(infname, outfname, 20, " ");
  task.run();

  if (stapl::get_location_id() == 0) {
    std::map<string, int> lib_results;
    std::map<string, int> local_results;
    ifstream ifs;

    for (unsigned int i = 0; i < get_num_locations(); ++i) {
      stringstream ss;
      ss << outfname;
      ss << ".";
      ss << i;

      ifs.open(ss.str().c_str());
      pair<string, int> p;
      while (!ifs.eof()) {
        ifs >> p.first;
        ifs >> p.second;
        lib_results.insert(p);
      }
      ifs.close();
    }

    ifs.open(infname.c_str());
    size_t begin_of_file = ifs.tellg();
    ifs.seekg(0, std::ios::end);
    size_t fsize = size_t(ifs.tellg()) - begin_of_file;
    ifs.seekg(0, std::ios::beg);
    string tmp;
    tmp.resize(fsize);
    ifs.read(&tmp[0], fsize);
    // tmp[fsize] = '\0';
    ifs.close();

    char_separator<char> sep(
      "/\r?\n/ ,./<>?;\':\"\\[]{}|-`~!@#$%^&*()_+= \t \r"
    );
    typedef tokenizer<char_separator<char> > tokenizer;
    tokenizer tokens(tmp, sep);
    for (tokenizer::iterator tok_iter = tokens.begin();
         tok_iter != tokens.end(); ++tok_iter) {
      //insert_to local_results
      std::pair<string, int> p = make_pair(*tok_iter, 1);
      std::pair<std::map<string, int>::iterator, bool> result;
      result = local_results.insert(p);
      if (!result.second) {
        std::pair<string, int> newp = task.reduce((*(result.first)), p);
        result.first->second = newp.second;
      }
    }

#ifdef SHOW_LIB_RESULTS
    cout << "lib_results:" << endl;
    for (std::map<string, int>::iterator it = lib_results.begin();
         it != lib_results.end(); ++it) {
      cout << (*it).first << " " << (*it).second << endl;
    }

    cout << "local_results:" << endl;
    for (std::map<string, int>::iterator it = local_results.begin();
         it != local_results.end(); ++it) {
      cout << (*it).first << " " << (*it).second << endl;
    }
#endif

    if (lib_results == local_results)
      cout << "[passed]" << endl;
    else
      cout << "[FAILED]" << endl;

  }

  return EXIT_SUCCESS;
}

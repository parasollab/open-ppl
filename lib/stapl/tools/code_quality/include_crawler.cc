/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
//////////////////////////////////////////////////////////////////////
/// @file include_crawler.cc
/// @brief IncludeCrawler: Checks for cyclic includes. Hacked from Recursor.
/// Basefolder is the root folder that contains "stapl" and "include".
/// -v1 and -v2 add a verbosity level.
/// 1 = crawling messages, 2 = full status as run.
///
/// Compile:
///  g++ -lboost_system -lboost_filesystem -o IncludeCrawler IncludeCrawler.cc
///
/// Execute
///  ./IncludeCrawler <baseFolder> <verbosity>
///  ./IncludeCrawler ./ -v1
///
/// Output:
///  CyclesFile.txt
///    Lists all the found include cycles, showing both the full chain that led
///    to that cycle and the cycle itself. A summary is listed at the end of
///    the file. File is replaced every time the crawler is run.
///  MissingFiles.txt
///      Lists all the files listed in includes that the crawler could not open.
///      The include it for is given, as well as some other places it tried
///      to find the file. It also gives the full include chain that led to
///      the missing file so you can track down the responsible file. A
///      summary is listed at the end of the file. File is replaced every time
///      the crawler is run.
///
/// Notes:
///  Matchlevel 1: show all matches by file name,
///  2: Show all matches by file and path,
///  3 & 4: Terse 1 and 2.
///  Matchelevel is fixed at 4 because spam.
///  I hacked this from the recursor program I wrote, so it's a little hacky
///  and slow in its crawling.
///  
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <fstream>

#include <boost/filesystem.hpp>

using namespace std;
namespace fs = boost::filesystem;

string get_include(string, bool*);
void parse_include(string, string, string, bool, int, string*, string*);
void split_name(string, string*, string*);
int file_crawl(string filename, string* skip_files, string* bad_chains);
bool directory_crawl(string d, string* skip_files, string* bad_chains);

// Global variables
int verbose; // 0 = report only, 1 = crawling messages, 2 = full status as run
string missingFiles;
string cycles; // Not light cycles, alas.
string root_dir;
ofstream mFilesFile;
ofstream cyclesFile;

int main(int argc, char** argv)
{
  string temp;
  string bad_chains = "^";
  string skip_files = "|";

  if (argc > 2)
  {
    if (argv[2][2] == '1')
      verbose = 1;
    else
      if (argv[2][2] == '2')
        verbose = 2;
      else
        verbose = 0;
  }
  else
    verbose = 0;

  mFilesFile.open("MissingFiles.txt", std::fstream::trunc);
  if (!mFilesFile.is_open())
  {
    cout << "Unable to open MissingFiles.txt for writing.\n";
    return 1;
  }
  cyclesFile.open("CyclesFile.txt", std::fstream::trunc);
  if (!cyclesFile.is_open())
  {
    cout << "Unable to open CyclesFile.txt for writing.\n";
    return 1;
  }

  // Crawl the stapl folder.
  root_dir = argv[1];
  temp = root_dir + "stapl";
  if (verbose > 0)
    cout << "Crawling to " << temp << endl;
  if (!directory_crawl(temp, &skip_files, &bad_chains))
    cout << "FAILURE: UNABLE TO OPEN STAPL DIRECTORY!\n";

  // Crawl include folder
  temp = root_dir;
  temp += "include";
  if (verbose > 0)
    cout << "Crawling to " << temp << endl;
  if (!directory_crawl(temp, &skip_files, &bad_chains))
    cout << "FAILURE: UNABLE TO OPEN INCLUDE DIRECTORY!\n";

  // report!
  cout << "Missing Files: \n";
  if (missingFiles.length() < 1)
    cout << "None" << endl;
  else
    cout << missingFiles << endl;

  cout << "Include Cycles: \n";
  if (cycles.length() < 1)
    cout << "None" << endl;
  else
    cout << cycles << endl;

  // Report to file
  mFilesFile << "\nMissing Files: \n";
  if (missingFiles.length() < 1)
    mFilesFile << "None" << endl;
  else
    mFilesFile << missingFiles << endl;

  cyclesFile << "\nInclude Cycles: \n";
  if (cycles.length() < 1)
    cyclesFile << "None" << endl;
  else
    cyclesFile << cycles << endl;

  mFilesFile.close();
  cyclesFile.close();

}

bool directory_crawl(string d, string* skip_files, string* bad_chains)
{
  fs::path thisDir(d);
  fs::directory_iterator theEnd;

  if (fs::exists(thisDir))
  {
    for (fs::directory_iterator dir_iter(thisDir);
    dir_iter != theEnd; ++dir_iter)
    {
      if (fs::is_directory(dir_iter->status()))
      {
        if (verbose > 0)
          cout << "Crawling to " << dir_iter->path().string() << endl;
        directory_crawl(dir_iter->path().string(), skip_files, bad_chains);
      }
      else if (dir_iter->path().string().find(".h") != string::npos) //Also hpp
      {
        file_crawl(dir_iter->path().string(), skip_files, bad_chains);
      }
    }
  }
  else
    return false;

  return true;
}

int file_crawl(string filename, string* skip_files, string* bad_chains)
{
  ifstream infile;
  string line;
  string include_file;
  string file;
  string path;
  string file_list;
  bool angle_bracket = false;
  int match_level = 1;

  match_level = 4;
  // Open file
  infile.open(filename.c_str());

  split_name(filename,&path, &file);

  while (getline(infile,line))
  {
    include_file = get_include(line, &angle_bracket);

    if (include_file != "No")
    {
      parse_include(include_file, path, "|"+path+"/"+file+"|", angle_bracket,
          match_level, skip_files, bad_chains);
    }
  }

  infile.close();
  return 0;
}

string get_include(string s, bool* angle_bracket)
{ //#include <stapl/containers/list/list.hpp>
  bool staplBracket = false;
  string retval = "No";
  size_t i, j;
  // find #
  i = s.find("#");
  if (i == string::npos )
    return retval;
 
  // find include
  i = s.find("include \"",i+1);
  if (i == string::npos )
  {
    i = s.find("include <stapl",i+1);
    if (i == string::npos )
      return retval;
    staplBracket = true;
  }
  i += 9;
  // find end of include
  if (staplBracket)
    j = s.find(">",i+1);
  else
    j = s.find("\"",i+1);

  if (j == string::npos )
    return retval;
  // return include string
  *angle_bracket = staplBracket;

  return s.substr(i,j-i);
}

void split_name(string s, string* path, string* file)
{
  size_t i;
  i = s.find_last_of("/\\");
  if (i == string::npos)
  {
    *path = "";
    *file = s;
  }
  else
  {
    *path = s.substr(0,i);
    *file = s.substr(i+1);
  }
  return;
}

void parse_include(string s, string old_path, string file_list,
    bool angle_bracket, int m_level, string* skip_files,
    string* bad_chains)
{
  string line;
  string path;
  string file;
  string filename;
  string include_file;
  string new_path;
  ifstream infile;
  bool local_angles = false;
  int match_level = 0;
  size_t fileLoc=0;

  split_name(s,&path, &file);

  if ((*skip_files).find("|"+s+"|") == string::npos)
  {
    if (file_list.find("/"+file+"|") != string::npos)
      match_level = 1 + (2*(m_level>2));
    if (file_list.find("|"+path+"/"+file+"|") != string::npos)
      match_level = 2+ (2*(m_level>2));
    if (path == "" && file_list.find("|"+file+"|") != string::npos)
      match_level = 2+ (2*(m_level>2));

    if (match_level >= m_level)
    {
      if (match_level > 2)
      {
        // Find s in file_list
        fileLoc = file_list.find("|"+s);
        if (fileLoc == string::npos)
          fileLoc = 0;
      }
      if (match_level < 3 || (*bad_chains).find("^"+s+"^") == string::npos)
      {
        (*bad_chains) += s + "^";
        if (verbose == 2)
          cout << "\n INCLUDE CYCLE \n\t Chain: " << file_list
            << "\n\t Cycle: " << file_list.substr(fileLoc)
            << "|" << s << endl;
        cyclesFile << "\n INCLUDE CYCLE \n\t Chain: " << file_list
          << "\n\t Cycle: " << file_list.substr(fileLoc) << "|" << s << endl;
        cycles += file_list.substr(fileLoc) + s + "\n";
      }
    }
    else
    {
      if (angle_bracket)
        new_path = s;
      else
        new_path = old_path + "/" + s;

      infile.open(new_path.c_str());

      if (angle_bracket)
        new_path = path;
      else
      {
        if (path == "")
          new_path = old_path;
        else
          new_path = old_path + "/" + path;
      }

      if (!infile.is_open())
      {
        string failed_paths = new_path + "\n";
        new_path = root_dir + "stapl/" + s;
        infile.open(new_path.c_str());

        failed_paths += new_path + "\n";
        new_path = root_dir + "stapl/" + path;
        if (!infile.is_open())
        {
          new_path = root_dir + "include/" + s;
          infile.open(new_path.c_str());
          failed_paths += new_path + "\n";
          new_path = root_dir + "include/" + path;
          if (!infile.is_open())
          {
            (*skip_files) += s + "|";
            if (verbose == 2)
              cout << "\n FAILED TO OPEN/FIND: " << s
                << "\n  Pulled from chain: "<< file_list
                << "\n  Tried:\n" << failed_paths << endl;
            mFilesFile << "\n FAILED TO OPEN/FIND: " << s
              << "\n  Pulled from chain: "<< file_list
              << "\n  Tried:\n" << failed_paths << endl;
            missingFiles += s + "\n";
          }
        }
      }
      while (getline(infile,line))
      {
        include_file = get_include(line, &local_angles);
        if (include_file != "No")
        {
          parse_include(include_file, new_path, file_list + s + "|",
            local_angles, m_level, skip_files, bad_chains);
        }
      }
    }
  }
  return;
}

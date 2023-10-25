//////////////////////////////////////////////////////////////////////
/// @file input_generator.cc
///
/// Generates two sequences of the specified size.  Each sequence is a
/// random sequence of lower-case characters from the English alphabet.
//////////////////////////////////////////////////////////////////////

#include <cstdlib>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv)
{
  const size_t input_size = atoi(argv[1]);
  const int sza = input_size;
  const int szb = input_size;

  ofstream myfile;

  myfile.open ("lcs_input.txt");

  srand(109500);

  myfile << 1 << endl;

  for (int i = 0;i < sza;i++)
    myfile << (char)('a'+rand() % 26);

  myfile << endl;

  for (int i = 0;i < szb;i++)
    myfile << (char)('a'+rand() % 26);

  myfile << endl;

  myfile.close();

  return 0;
}

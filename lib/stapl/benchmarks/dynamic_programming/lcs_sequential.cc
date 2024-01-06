//////////////////////////////////////////////////////////////////////
/// @file lcs_sequential.cc
///
/// A LCS implementation that works with different containers implementing []
/// operator Those containers can contain any elements implementing the
/// equality operator == They can be accessed by indices or iterators.
//////////////////////////////////////////////////////////////////////


#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <time.h>

//////////////////////////////////////////////////////////////////////
/// @brief Simple timer class that mimics the interface of stapl::counter.
//////////////////////////////////////////////////////////////////////
class clock_gettime_timer
{
private:
  struct timespec m_ts;

public:
  void start(void)
  {
    if (clock_gettime(CLOCK_REALTIME, &m_ts) != 0)
      printf("clock_gettime_timer failed\n");
  }

  double stop(void) const
  {
    struct timespec nt;
    if (clock_gettime(CLOCK_REALTIME, &nt) != 0)
      printf("clock_gettime_timer failedi\n");
    return (double(nt.tv_sec-m_ts.tv_sec) +
            double(nt.tv_nsec-m_ts.tv_nsec)/1.0E9);
  }

};

#define max(a,b) (((a) > (b)) ? (a) : (b))

using namespace std;

//////////////////////////////////////////////////////////////////////
/// @brief Computes the length of the LCS between the subsequencs represented
///   by (seq1 [0...p.first-1] and seq2 [0...p.second-1])
///
/// @param p a substructure  pointing to the range of the subproblem, should
/// have .first and .second (e.g. a pair of iterators or ints)
///
/// @param seq1,seq2 The sequences of elements representing the inputs to the
///   problem.
///
/// @param table the DP table that will hold the results of the subproblems
//////////////////////////////////////////////////////////////////////
template<typename C, typename T>
int getLengthLCS(const T& p, const C& seq1, const C& seq2,
                 vector <vector<int> >& table)
{
  if (p.first == 0 || p.second == 0)
      return table [p.first][p.second] = 0;

  if (table [p.first][p.second] != -1)
      return table [p.first][p.second];

  int p1 = getLengthLCS(T(p.first-1, p.second-1), seq1, seq2, table) +
           (seq1 [p.first] == seq2 [p.second] ? 1 : 0);

  int p2 = getLengthLCS(T(p.first-1, p.second), seq1, seq2, table);

  int p3 = getLengthLCS(T(p.first, p.second-1), seq1, seq2, table);

  table [p.first][p.second] = max(p1, max(p2, p3));

  // store all the best choices
  //
  return table [p.first][p.second];
}

//////////////////////////////////////////////////////////////////////
/// @brief Initialize the 2D table that will hold the solution of the
///   subproblems with a sentinel value.
//////////////////////////////////////////////////////////////////////
void initialize(vector<vector<int> >& table,
                int size1, int size2, int sentinel)
{
  table.resize(size1);

  for (vector<vector<int> >::size_type i = 0; i < table.size(); i++)
  {
      table [i].assign(size2, sentinel);
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Time the execution of the LCS algorithm on two input sequences.
///
/// The driver calls this function for each pair of sequences provided in the
/// input file.
//////////////////////////////////////////////////////////////////////
template<typename C>
void callLCS(const C& seq1, const C& seq2, int* dur)
{
  int startTime = time(NULL);

  clock_gettime_timer timer;

  timer.start();

  pair <int, int> initial(seq1.size()-1, seq2.size()-1);
  vector< vector<int> > table;

  initialize(table, seq1.size(), seq2.size(), -1);

  int lcs_length = getLengthLCS(initial, seq1, seq2, table);

  double elapsed = timer.stop();

  int tempDur = time(NULL)-startTime;

  cout << "Length of LCS is " << lcs_length
       << " computed in " << elapsed << endl;

  *dur += tempDur;
}


int main(int argc, char** argv)
{
  // Sequences are represented from elem_1 ... elem_(n-1) as elem_0
  //  is reserved as a special character
  //
  int cases;
  int  accumDur = 0;

  ifstream myfile;

  myfile.open (argv [1]);
  myfile >> cases;

  string s1, s2;

  for (int cs=0; cs < cases; ++cs)
  {
    myfile >> s1 >> s2;

    vector <char> a(s1.size()+1);
    vector <char> b(s2.size()+1);

    for (vector<char>::size_type i = 1; i <= s1.size(); i++)
      a [i] =  s1 [i-1];

    for (vector<char>::size_type i = 1; i <= s2.size(); i++)
      b [i] = s2 [i-1];

    callLCS(a, b, &accumDur);
  }

  myfile.close();

  cout << "Duration is: " << accumDur << endl;

  return 0;
}

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>

using namespace std;

int main(int argc, char **argv) {
  if ( argc < 2 ) {
    cerr << "Must specify name of input file on command line" << endl;
    cerr << endl;
    exit(1);
  }
  ifstream in_str;
  in_str.open( argv[1], ios::in|ios::binary );
  if ( !in_str.is_open() ) {
    cerr << "Unable to open output file: " << argv[3] << endl;
    cerr << endl;
    exit(1);
  }

  if ( argc < 3 ) {
    cerr << "Must specify name of output file on command line" << endl;
    exit(1);
  }
  ofstream out_str;
  out_str.open(argv[2], ios::out);
  if ( !out_str.is_open() ) {
    cerr << "Unable to open output file: " << argv[3] << endl;
    exit(1);
  }

  if ( argc < 4 ) {
    cerr << "Must specify number of dimensions on command line" << endl;
    exit(1);
  }
  int dims = atoi(argv[3]);
  if ( dims < 1 ) {
    cerr << "Number of dimensions must be > 0" << endl;
    exit(1);
  }

  int count = 0;
  int buf_len = 32 * dims;
  double *val_buf = new double [dims];

  in_str.read( (char *)val_buf, sizeof(double)*dims );
  while ( !in_str.eof() ) {
    for ( int ndx = 0; ndx<dims; ndx++ ) {
      if ( ndx > 0 )
         out_str << " ";
      out_str << val_buf[ndx];
    }
    out_str << endl;
    ++count;

    in_str.read( (char *)val_buf, sizeof(double)*dims );
  }
  cout << "Record count: " << count << endl;
  in_str.close();
  out_str.close();
}

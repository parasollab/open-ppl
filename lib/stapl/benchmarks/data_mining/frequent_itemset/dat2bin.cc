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
  in_str.open( argv[1], ios::in );
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
  out_str.open(argv[2], ios::out|ios::binary);
  if ( !out_str.is_open() ) {
    cerr << "Unable to open output file: " << argv[3] << endl;
    exit(1);
  }

  int val_buf_len = 4096;
  int buf_len = 32 * val_buf_len;
  char *line_buf = new char [buf_len];
  char *word_buf = new char [buf_len];
  long *val_buf = new long [val_buf_len];

  line_buf[0]= '\0';
  in_str.getline(line_buf, buf_len);
  int decl_cnt = atoi(line_buf);
  out_str.write( (char *)&decl_cnt, sizeof(long) );

  int actual_cnt = 0;
  int record_size = 0;
  line_buf[0]= '\0';
  in_str.getline(line_buf, buf_len);
  while ( !in_str.eof() ) {
    int ndx = 0;
    char *ptr;
    ptr = strtok( line_buf, " " );
    while ( ptr != NULL ) {
      strcpy( word_buf, ptr );
      long value = atoi(word_buf);
      val_buf[ndx++] = value;
      ptr = strtok( NULL, " " );
    }
    record_size = (ndx > record_size) ? ndx : record_size;
    out_str.write( (char *)&ndx, sizeof(long) );
    out_str.write( (char *)val_buf, sizeof(long)*ndx );
    ++actual_cnt;

    line_buf[0]= '\0';
    in_str.getline(line_buf, buf_len);
  }
  cout << "Declared record count: " << decl_cnt << endl;
  cout << "Actual record count: " << actual_cnt << endl;
  cout << "Max record size: " << record_size << endl;
  in_str.close();
  out_str.close();
}

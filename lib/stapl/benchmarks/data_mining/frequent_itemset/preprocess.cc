#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <map>

using namespace std;
typedef unsigned long ulong;

int main(int argc, char **argv) {
  if ( argc < 2 ) {
    cerr << "Must specify name of input file on command line" << endl;
    cerr << endl;
    exit(1);
  }
  ifstream in_str;
  in_str.open( argv[1], ios::in );
  if ( !in_str.is_open() ) {
    cerr << "Unable to open output file: " << argv[1] << endl;
    cerr << endl;
    exit(1);
  }

  if ( argc < 3 ) {
    cerr << "Must specify name of output meta file on command line" << endl;
    exit(1);
  }
  ofstream out_meta_str;
  out_meta_str.open(argv[2], ios::out|ios::binary);
  if ( !out_meta_str.is_open() ) {
    cerr << "Unable to open output file: " << argv[2] << endl;
    exit(1);
  }

  if ( argc < 4 ) {
    cerr << "Must specify name of output data file on command line" << endl;
    exit(1);
  }
  ofstream out_data_str;
  out_data_str.open(argv[3], ios::out|ios::binary);
  if ( !out_data_str.is_open() ) {
    cerr << "Unable to open output data file: " << argv[3] << endl;
    exit(1);
  }

  vector<ulong> rec_len;

  ulong val_buf_len = 4096;
  ulong buf_len = 32 * val_buf_len;

  char *line_buf = new char [buf_len];
  char *word_buf = new char [buf_len];
  ulong *val_buf = new ulong [val_buf_len];
  map<int,int> item_cnt;

  ulong max_sz = 0;
  ulong trans_cnt = 0;
  line_buf[0]= '\0';
  in_str.getline(line_buf, buf_len);

  while ( !in_str.eof() ) {

    ulong ndx = 0;
    char *ptr;
    ptr = strtok( line_buf, " " );
    while ( ptr != NULL ) {
      strcpy( word_buf, ptr );
      ulong value = atoi(word_buf);
      val_buf[ndx++] = value;

      ulong count = item_cnt[value];
      item_cnt[value] = count+1;
      ptr = strtok( NULL, " " );
    }
    out_data_str.write( (char *)val_buf, sizeof(ulong)*ndx );
    rec_len.push_back(ndx);

    max_sz = (ndx > max_sz) ? ndx : max_sz;
    ++trans_cnt;
    line_buf[0]= '\0';
    in_str.getline(line_buf, buf_len);
  }
  ulong tot_items = item_cnt.size();

  out_meta_str.write( (char *)&trans_cnt, sizeof(ulong) ); //
  out_meta_str.write( (char *)&max_sz, sizeof(ulong) ); //
  out_meta_str.write( (char *)&tot_items, sizeof(ulong) ); //

  map<int,int>::iterator iter;
  for( iter= item_cnt.begin(); iter != item_cnt.end(); ++iter ) {
    ulong key = iter->first;
    ulong val = iter->second;
    cerr << "Item: " << key << " " << val << endl;
    out_meta_str.write( (char *)&key, sizeof(ulong) );
    out_meta_str.write( (char *)&val, sizeof(ulong) );
  }
  cerr << "Item count: " << item_cnt.size() << endl;
  out_meta_str.write( (char *)rec_len.data(), sizeof(ulong)*trans_cnt );

  if( item_cnt.size() > 65535 ) {
    cerr << "Too many items to store in 16 bits" << endl;
    cerr << "Use 32-bit version of FIM" << endl;
  }

  cerr << "Transaction count: " << trans_cnt << endl;
  cerr << "Max transaction size: " << max_sz << endl;

  

  in_str.close();
  out_meta_str.close();
  out_data_str.close();
}

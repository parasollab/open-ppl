#include "freqmine.hpp"
#include <time.h>

#include "confint.hpp"

using namespace std;

stapl::exit_code experiment( string, string, string,
                 double, double, char, bool, double );

bool freq_mine( string, string, string,
                double, double, char, double );

#define DBG_STEP 1

ulong first_mask_list [] = {
  0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFC, 0xFFFFFFF8,
  0xFFFFFFF0, 0xFFFFFFE0, 0xFFFFFFC0, 0xFFFFFF80,
  0xFFFFFF00, 0xFFFFFE00, 0xFFFFFC00, 0xFFFFF800,
  0xFFFFF000, 0xFFFFE000, 0xFFFFC000, 0xFFFF8000,
  0xFFFF0000, 0xFFFE0000, 0xFFFC0000, 0xFFF80000,
  0xFFF00000, 0xFFE00000, 0xFFC00000, 0xFF800000,
};
ulong second_mask_list [] = {
  0x00000000, 0x00000001, 0x00000003, 0x00000007,
  0x0000000F, 0x0000001F, 0x0000003F, 0x0000007F,
  0x000000FF, 0x000001FF, 0x000003FF, 0x000007FF,
  0x00000FFF, 0x00001FFF, 0x00003FFF, 0x00007FFF,
  0x0000FFFF, 0x0001FFFF, 0x0003FFFF, 0x0007FFFF,
  0x000FFFFF, 0x001FFFFF, 0x003FFFFF, 0x007FFFFF
};

ulong first_mask, second_mask, encode_shift, max_key;

///////////////////////////////////////////////////////////////////////////
// process the command line options
// call the frequent itemset mining algorithm
///////////////////////////////////////////////////////////////////////////

stapl::exit_code stapl_main(int argc, char **argv)
{
  string meta_path, data_path, out_path;
  double min_supp = 0.05, min_conf = 0.9, load_bal = 1.5;
  ifstream meta_in;
  stapl::stream<ifstream> data_in;
  stapl::stream<ofstream> freq_out;
  char in_type = 'b'; // or 't'

  if ( argc > 1 ) {
    meta_path = string(argv[1]);
    meta_in.open(meta_path.c_str());
    if ( !meta_in.is_open() ) {
      stapl::do_once([&]() {
        cerr << "Unable to open input meta file: " << meta_path << endl;
      });
    }
    meta_in.close();
  } else {
    stapl::do_once([&]() {
      cerr << "Must specify input meta file path" << endl;
    });
    return EXIT_FAILURE;
  }

  if ( argc > 2 ) {
    data_path = string(argv[2]);
    data_in.open(data_path.c_str());
    if ( !data_in.is_open() ) {
      stapl::do_once([&]() {
        cerr << "Unable to open input data file: " << data_path << endl;
      });
    }
    data_in.close();
  } else {
    stapl::do_once([&]() {
      cerr << "Must specify input data file path" << endl;
    });
    return EXIT_FAILURE;
  }

  if ( argc > 3 ) {
    out_path = string(argv[3]);
    freq_out.open(out_path.c_str(), ios::out );
    if ( !freq_out.is_open() ) {
      stapl::do_once([&]() {
        cerr << "Unable to open output file: " << out_path << endl;
      });
    } {
    }
    freq_out.close();
  } else {
    stapl::do_once([&]() {
      cerr << "Must specify output file path" << endl;
    });
    return EXIT_FAILURE;
  }

  if ( argc > 4 ) {
    in_type = *argv[4];
    if ( in_type != 'b' && in_type != 't' ) {
      stapl::do_once([&]() {
        cerr << "Input type must be binary or text: " << argv[4] << endl;
      });
      return EXIT_FAILURE;
    }
  }

  bool once_only = false;
  if ( argc > 5 ) {
     char temp = *argv[5];
     if ( 'y' == temp ) {
       once_only = true;
     }
  }

  if ( argc > 6 ) {
    char *temp = 0;
    min_supp = strtod(argv[6],&temp);
    if ( min_supp <= 0.0 || min_supp >= 1.0 ) {
      stapl::do_once([&]() {
        cerr << "Invalid minimum support: " << argv[6] << ": "
             << min_supp << endl;
      });
      return EXIT_FAILURE;
    }
  }

  if ( argc > 7 ) {
    char *temp = 0;
    min_conf = strtod(argv[7],&temp);
    if ( min_conf <= 0.0 || min_conf >= 1.0 ) {
      stapl::do_once([&]() {
        cerr << "Invalid minimum confidence: " << argv[7] << ": "
             << min_conf << endl;
      });
      return EXIT_FAILURE;
    }
  }

  if ( argc > 8 ) {
    char *temp = 0;
    load_bal = strtod(argv[8],&temp);
    if ( load_bal <= 0.0 || load_bal >= 1.0 ) {
      stapl::do_once([&]() {
        cerr << "Invalid load balancing factor: " << argv[8] << ": "
             << load_bal << endl;
      });
      return EXIT_FAILURE;
    }
  }

  return experiment( meta_path, data_path, out_path,
                  min_supp, min_conf, in_type, once_only, load_bal );
}

///////////////////////////////////////////////////////////////////////////
// execute frequent itemset mining enough times to achieve
// desired confidence interval
///////////////////////////////////////////////////////////////////////////

stapl::exit_code experiment( string meta_path, string data_path,
                             string out_path,
                             double min_supp, double min_conf, char in_type,
                             bool once_only, double load_bal )
{
  counter_t timer;
  bool success = true;

  bool continue_iterating = true;
  confidence_interval_controller iter_control(32, 100, 0.05);
  while (continue_iterating && success) {
    timer.reset();
    timer.start();

    success =  freq_mine( meta_path, data_path, out_path,
                          min_supp, min_conf, in_type, load_bal );

    iter_control.push_back(timer.stop());

    stapl::array<int> continue_ct(stapl::get_num_locations());
    stapl::array_view<stapl::array<int>> continue_vw(continue_ct);
    continue_vw[stapl::get_location_id()] = iter_control.iterate() ? 1 : 0;

    int iterate_sum = stapl::accumulate(continue_vw, (int) 0);

    continue_iterating = (!once_only) && (iterate_sum != 0 ? true : false);
  }

  iter_control.report("freqmine");

  if ( success ) {
    return EXIT_SUCCESS;
  } else {
    return EXIT_FAILURE;
  }
}

///////////////////////////////////////////////////////////////////////////
// parallel frequent itemset mining
///////////////////////////////////////////////////////////////////////////

bool freq_mine( string meta_path, string data_path, string out_path,
                double min_supp, double min_conf, char in_type,
                double load_bal ) {

  ifstream meta_in;
  stapl::stream<ifstream> data_in;
  stapl::stream<ofstream> freq_out;

  meta_in.open(meta_path.c_str());
  data_in.open(data_path.c_str());
  freq_out.open(out_path.c_str(), ios::out );

  // Read the transactions database metadata

  // -- transaction count

  ulong trans_cnt = 0;
  if ( !meta_in.eof() ) {
    meta_in.read( (char *)&trans_cnt, sizeof(ulong) );
  } else {
    return false;
  }

  if ( trans_cnt == 0 ) {
    cerr << "freq_mine: transaction count == 0" << endl;
    return false;
  }

  // -- max transaction size

  ulong max_sz = 0;
  if ( !meta_in.eof() ) {
    meta_in.read( (char *)&max_sz, sizeof(ulong) );
  } else {
    return false;
  }

  if ( max_sz == 0 ) {
    cerr << "freq_mine: maximum transaction size == 0" << endl;
    return false;
  }

  // -- total unique items

  ulong tot_items = 0;
  if ( !meta_in.eof() ) {
    meta_in.read( (char *)&tot_items, sizeof(ulong) );
  } else {
    return false;
  }

  if ( tot_items == 0 ) {
    cerr << "freq_mine: total unique items == 0" << endl;
    return false;
  }

  // -- unique item counts

  pary_ul_tp item_key(tot_items), item_cnt(tot_items);

  stapl::do_once([&]() {
    if ( !meta_in.eof() ) {
      ulong buffer[tot_items*2];
      meta_in.read( (char *)&buffer, sizeof(ulong)*tot_items*2 );
      for ( ulong i=0; i<tot_items; i++ ) {
        item_key[i] = buffer[2*i];
        item_cnt[i] = buffer[2*i+1];
      }
    }
  });

  // -- item counts per transaction

  pary_ul_tp trans_len(trans_cnt);

  stapl::do_once([&]() {
    if ( !meta_in.eof() ) {
      ulong buffer[trans_cnt];
      meta_in.read( (char *)&buffer, sizeof(ulong)*trans_cnt );
      for ( ulong i=0; i<trans_cnt; i++ ) {
        trans_len[i] = buffer[i];
      }
    }
  });
  stapl::rmi_fence();

  stapl::stream<ofstream> dbg_out;
  string dbg_path("freqmine");
  time_t init_time;
  init_time = time(NULL);
  string dbg_suffix = to_string(init_time);
  dbg_path += dbg_suffix;
  dbg_path += ".dbg";
  dbg_out.open(dbg_path.c_str());
  if ( !dbg_out.is_open() ) {
    return false;
  }

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "freq_mine: trans count: " << trans_cnt << endl;
    dbg_out << "freq_mine: trans size: " << max_sz << endl;
    dbg_out << "freq_mine: total items: " << tot_items << endl;
  });
#endif

  encode_shift = 1 + (std::log(tot_items) / std::log(2));
  assert( encode_shift < 16 );
  max_key = std::exp2(2 * encode_shift);
  first_mask = first_mask_list[encode_shift];
  second_mask = second_mask_list[encode_shift];
  assert ( 0 == (first_mask & second_mask) );
  assert ( 0xFFFFFFFF == (first_mask | second_mask) );

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "freq_mine: max_key:     " << max_key << endl;
    dbg_out << "freq_mine: encode_shift: " << encode_shift << endl;
    dbg_out << "freq_mine: first_mask:   " << first_mask << endl;
    dbg_out << "freq_mine: second_mask:  " << second_mask << endl;
  });
#endif


  ulong count = 0;
#ifdef USE_NESTPAR
  bool force_nested = false;
  // must determine actual threshold empirically
  if ( force_nested || (trans_cnt / max_sz) >= 100000 ) {

#if defined(DBG_STEP)
    stapl::do_once([&]() {
      dbg_out << "Double nest: " << trans_cnt << " " << max_sz << endl;
    });
#endif

    count = eclat_nest( min_supp, min_conf, load_bal,
                        trans_cnt, max_sz, tot_items, in_type,
                        trans_len, item_cnt,
                        data_in, freq_out, dbg_out );
#if defined(DBG_STEP)
    stapl::do_once([&]() {
      dbg_out << "Double nest DONE: " << count << endl;
    });
#endif

  } else {
#endif
#if defined(DBG_STEP)
    stapl::do_once([&]() {
      dbg_out << "Single nest: " << trans_cnt << " " << max_sz << endl;
    });
#endif

    count = eclat_flat( min_supp, min_conf, load_bal,
                        trans_cnt, max_sz, tot_items, in_type,
                        trans_len, item_cnt,
                        data_in, freq_out, dbg_out );
#if defined(DBG_STEP)
    stapl::do_once([&]() {
      dbg_out << "Single nest DONE: " << count << endl;
    });
#endif

#ifdef LINK_NESTPAR
  }
#endif

  if ( count > 0 ) {
    return true;
  } else {
    stapl::do_once([&]() {
      cerr << "Not enough work for allocated processors." << endl;
      cerr << "Decrease minimum support, increase transaction count," << endl;
      cerr << "or invoke program with fewer processors" << endl;
    });
    return false;
  }

  meta_in.close();
  data_in.close();
  freq_out.close();
}

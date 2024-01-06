#!/usr/bin/env perl

#@inputnames = `ls inputs/input*`;
#@timinginputnames = `ls inputs/timing/test_input*`;

@inputnames = `ls /home/timmie/asci_work/asci/inputs/input*`;
@timinginputnames = `ls /home/timmie/asci_work/asci/inputs/timing/test_input*`;

foreach $input (@inputnames) {
  chomp $input;

  print "Testing input $input ... ";

  `java xpath  $input > zzz`;
  $len = 0;
  $len = `grep "Failed" zzz | wc -l`;
  `rm -f zzz`;
  if ($len != 0) {
    print "Failed!\n";
  } else {
    print "Passed.\n";
  }
}

foreach $input (@timinginputnames) {
  chomp $input;

  print "Testing input $input ... ";

  `java xpath  $input > zzz`;
  $len = 0;
  $len = `grep "Failed" zzz | wc -l`;
  `rm -f zzz`;
  if ($len != 0) {
    print "Failed!\n";
  } else {
    print "Passed.\n";
  }
}


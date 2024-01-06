#!/usr/bin/perl

use POSIX qw(ceil floor);

#round up to $sig_digs significant digits
sub sig_ceil($$) { 
  my $num = $_[0];
  my $sig_digs = $_[1];

  my $cmp_num = 1; 
  my $i = 0; 
  my $digits = 0;

  do { $cmp_num *=10; $i++} while ($i != $sig_digs);
  while ($num >= $cmp_num) { $num /= 10; $digits++; }
  $num = ceil $num;
  while ($digits) { $digits--; $num *=10; }
  return $num;
}

sub sig_floor($$) { 
  my $num = $_[0];
  my $sig_digs = $_[1]; 

  my $cmp_num = 1; 
  my $i = 0; 
  my $digits = 0;

  do { $cmp_num *=10; $i++} while ($i != $sig_digs);
  do { $num /= 10; $digits++; } while $num >= $cmp_num;
  $num = floor $num;
  do { $digits--; $num *=10; } while $digits;
  return $num;
}

1;



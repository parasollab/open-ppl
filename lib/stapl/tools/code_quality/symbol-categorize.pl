#!/usr/bin/perl

#
# This script receives a binary as input and categorizes the symbols, attempting
# to aggregate instantiations of a common class or function template together,
# giving an overview of how often various templates are instantiated.  Grouped
# symbols are printed in descending order by count.
#

# TODO - The parsing below improperly handles a few cases such as pointer to
# member definitions, operator<<, operator<, etc.  They get blanked out
# completely.  For now they are categorized as such.

# TODO - c++filt fails to demangle some symbols

use strict;
use warnings;

die "Need to specify filename." if (@ARGV < 1);

my @long_strings = split(/\n/, `nm --demangle $ARGV[0]`);

my %hash = ();
foreach (@long_strings)
{
  my $original_string = $_;
  chomp($_);
  my $string = substr($_, 19);

  my $done = 0;
  my $cnt  = 0;
  my $done2 = 0;

  #
  # Remove <.*> from each symbol.  Properly handles nested angle brackets.
  #
  do {
    my @groups = $string =~ m/(<(?:[^<>]++|(?1))*>)/; #/g;

    foreach my $idx (@groups)
    {
      my $val = quotemeta($idx);
      $string =~ s/$val//g;
    }

    $done2 = @groups eq 0;

  } while (not $done2);


  #
  # Remove (.*) from each symbol.  Properly handles nested parenthesis.
  #
  do {
    my @groups = $string =~ m/(\((?:[^\(\)]++|(?1))*\))/; #/g;

    foreach my $idx (@groups)
    {
      my $val = quotemeta($idx);
      $string =~ s/$val//g;
    }

    $done2 = @groups eq 0;
  } while (not $done2);


  #
  # Detect const qualification at end (and leading space) prior to removal
  # function return type.
  #
  my $b_const_qualified = 0;

  if ($string =~ /.* const$/)
  {
    $string =~ s/ const$//g;
    $b_const_qualified = 1;
  }

  $string =~ s/.* //g;

  if ($b_const_qualified)
  { $string = $string." const"; }


  #
  # Aggregate GCC exception table symbols
  #
  $string =~ s/GCC_except_table[0-9]*/GCC_except_table/g;


  #
  # Aggregate .LCPI symbols, whatever they are.
  #
  $string =~ s/\.LCPI.*/.LCPI/g;

  #
  # Detect improper parsing and mark accordingly.
  #
  if ($string =~ /^[ ]*$/)
  { $string = "BLANK / Improperly Parsed (e.g., Pointer to Members)"; }

  #
  # Increment the count for this symbol
  #
  $hash{$string}++;
}


#
# Print out in descending order by count.
#
foreach my $name (sort { $hash{$b} <=> $hash{$a} } keys %hash)
{
  printf "%-80s %s\n", $name, $hash{$name};
}



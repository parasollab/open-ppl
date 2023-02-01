#!/usr/bin/perl

#
# Replace Hex addresses given by Address Sanitizer with
#   their corresponding symbol.
# The output is written to standard out
#

if (@ARGV < 2 || @ARGV > 2)
{
  print "=========================================\n";
  print "== Address Sanitizer Output Symbolizer ==\n";
  print "=========================================\n";
  print "Covert the hex addresses of Address Sanitizer output to line numbers";
  print " and function names if possible\n\n";
  print "usage ./symbolize_ASAN_report.sh executable ASanOutputTextFile\n\n";
  print "Note: The executable must be compiled with symbols.\n";
  print "      Any linked libraries should also be compiled with symbols.\n";
  exit;
}

open(ASANREPORT, $ARGV[1]);

while (<ASANREPORT>)
{

  #if there is a hex value
  if ($_ =~ /(0x[0123456789abcdefABCDEF]*)/)
  {
    #get the hex value
    $hex = $1 if $_ =~ /(0x[0123456789abcdefABCDEF]*)/;

    #find the symbol using addr2line
    $result = `addr2line -e $ARGV[0] -p -i -f -C $hex`;

    #edit the line if addr2line was successful, otherwise do nothing
    #  addr2line was unsuccessful if it contains ?? in the response
    #TODO: invert logic for easier comprehension
    if ($result =~ /(\?)\1/) {}
    else
    {
      #pretty print formatting
      chomp $result;
      $result = "\n      " . $result . "\n     ";
      $result =~ s/ at /\n      at /;

      $_ =~ s/$hex/$result/;
    }
  }

  #print the possibly edited line
  print "$_";

}

close(ASANREPORT);

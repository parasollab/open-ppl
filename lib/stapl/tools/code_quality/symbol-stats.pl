#!/usr/bin/perl

=head1 SYNOPSIS
 
  symbol-stats dumps so info about symbols in stapl binary.

  Usage:

    symbol-stats filename [options]

  Help Options:

    --demangle        Report on demangled symbols (default).
    --mangled         Report on mangled symbols.
    --stapl           Include only stapl symbols (stapl namespace). 
    --no-stapl        Include only non stapl symbols.
    --no-histogram    Do not bother generated histogram.
    --no-comma        Stats are comma-less.  Easier to parse with other scripts.
    --help            Show this script's help information.

=cut

use List::Util qw(sum);
use List::Util qw(max);
use Getopt::Long;
use Text::Histogram qw(histogram);
use Pod::Usage;
use strict;

sub commify
{
   my $input = shift;

   $input    = reverse $input;
   $input    =~ s/(\d\d\d)(?=\d)(?!\d*\.)/$1,/g;

   return reverse $input;
}


sub cond_commify
{
  my($do_it, $str) = @_;

  return commify($str) if ($do_it);
  return $str;
}


my $stapl        = '';
my $no_stapl     = '';
my $demangle     = '';
my $mangled      = '';
my $no_comma     = '';
my $no_histogram = '';
my $help         = '';

GetOptions('stapl'        => \$stapl,
           'no-stapl'     => \$no_stapl,
           'mangled'      => \$mangled,
           'demangle'     => \$demangle,
           'no-comma'     => \$no_comma,
           'no-histogram' => \$no_histogram,
           'help'         => \$help);

my $comma=!$no_comma;

if ($help)
{
  pod2usage();
  exit;
}

die "Need to specify filename." if (@ARGV < 1);
           
die "Can't define both --just-stapl and --no_stapl)" if ($stapl && $no_stapl);
die "Can't define both --mangled and --demangle" if ($mangled && $demangle);

my $grep_str        = "| grep stapl" if ($stapl);
my $grep_str        = "| grep -v stapl" if ($no_stapl);

my $demangle_str    = "--demangle " if (!$mangled);

my $description_str = "All Symbols";

$description_str = "STAPL Symbols" if ($stapl);
$description_str = "Non-STAPL Symbols" if ($no_stapl);

my @symbol_lengths = split(/\n/, `nm $demangle_str $ARGV[0] $grep_str | awk '{print length}'`);


if (!$no_histogram)
{
  my %opts = ('bins', '30');
  print histogram(\@symbol_lengths, \%opts)."\n";
}

print "----------------------------------------------------\n";
print " Statistics for $ARGV[0] ($description_str)\n";
print "----------------------------------------------------\n";

my $num_symbols    = @symbol_lengths;
my $total_cnt_txt  = cond_commify($comma, $num_symbols);

my $sum            = sum(@symbol_lengths);
my $total_size_txt = cond_commify($comma, $sum);

my $avg_size_txt   = $sum / $num_symbols;
my $max_size_txt   = cond_commify($comma, max(@symbol_lengths));

my $pad_len = 15;

printf "   Total count:  %${pad_len}s\n", $total_cnt_txt;
printf "   Total size:   %${pad_len}s\n", $total_size_txt;
printf "   Average size: %${pad_len}.2f\n", $avg_size_txt;
printf "   Maximum size: %${pad_len}s\n", $max_size_txt;


#!/usr/bin/perl

use strict;
use warnings;
use Getopt::Long;

my $stapl_dir = "/research/amarillo/timmie/freshest";

#my $verbose   = '';

#GetOptions('verbose' => \$verbose);

die "Need to specify filename." if (@ARGV < 1);

my $cmd  = qq(grep -i "mpic\\|elapsed\\|Entering\\|Leaving" $ARGV[0]);

my @input = split(/\n/, `$cmd`);

my $level             = 0;
my $output_line       = 0;

my @time_stack        = (0);
my @count_stack       = (0);
my @prefix_stack      = ($stapl_dir);
my @output_idx_stack  = 0;

my @output            = $stapl_dir;

foreach (@input)
{
  my $spaces = " " x (2 * $level);

  if ($_ =~ /Entering directory/)
  {
    ++$level;
    ++$output_line;

    s/.*Entering directory \`$prefix_stack[0]//;
    s/^\///;
    s/\'//;

    push(@time_stack, 0);
    push(@count_stack, 0);

    unshift(@prefix_stack, $prefix_stack[0]."/".$_);
    unshift(@output_idx_stack, $output_line); 
 
    push(@output, $_);

  }  

  if ($_ =~ /elapsed /)
  {
    s/.*system //g;
    s/elapsed.*//g;

    #if (defined $verbose)
    #{
    #  print $spaces, $_, "\n";
    #}

    (my $minutes, my $seconds) = split(/:/, $_);

    $seconds += $minutes * 60;

    $count_stack[$#count_stack] += 1;
    $time_stack[$#time_stack]   += $seconds;
  }


  if ($_ =~ /Leaving directory/)
  {
    --$level;

    my $level_time  = pop(@time_stack);
    my $level_count = pop(@count_stack);

    $time_stack[$#time_stack]   += $level_time;
    $count_stack[$#count_stack] += $level_count; 

    shift @prefix_stack;

    my $idx        = shift @output_idx_stack;
    my $time_fmt   = sprintf("%4d",$level_time);

    if ($level_time eq 0)
    {
      $output[$idx] = "";
    }

    if ($output[$idx] ne "")
    {
      $output[$idx]  = $spaces.$time_fmt." || ".$output[$idx]." (".$level_count." tests)\n";
    }
  }
}

my $time_fmt = sprintf("%4d", pop(@time_stack));
$output[0] = $time_fmt." || ".$output[0]." (".pop(@count_stack)." tests)\n";

foreach (@output)
{
  print $_;
}


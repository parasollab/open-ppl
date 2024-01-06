#!/usr/bin/perl

use Getopt::Long;
use POSIX;
use strict;
use warnings;

GetOptions ("s=s" => \my $opt_s,
            "n=s" => \my $opt_n,
            "o=s" => \my $opt_o,
            "t=s" => \my $opt_t,
            "m=s" => \my $opt_m,
            "g=s" => \my $opt_g);

my %hash = ();
my %colors = ();

if (!defined $opt_s) { die "must define stapl timing file";             }
if (!defined $opt_n) { die "must define nas timing file";               }
if (!defined $opt_o) { die "must define output eps file (-o filename}"; }
if (!defined $opt_t) { die "must define title (-t title)";              }
if (!defined $opt_m) { die "must define machine name (-m machine)";     }
if (!defined $opt_g) { die "must define graph type (-g {s,t})";         }

my $value_field;

if ($opt_g eq 's')
{
  $colors{'ideal'} = 'g';
  $value_field = 'scale';
}
elsif ($opt_g eq 't')
{
  $value_field = 'mean';
}

$colors{'stapl'} = 'b';
$colors{'nas'}   = 'r';

my $max_mean = 0;

read_times($opt_s, 'stapl');
read_times($opt_n, 'nas');

my @sorted_procs = sort { $a <=> $b } keys %hash;

my $base_proc = $sorted_procs[0];
my $max_procs = $sorted_procs[$#sorted_procs]; 


foreach my $proc (@sorted_procs)
{
  # make sure the input files defined times for the same processor counts 
  die "Invalid Input" unless keys(%{$hash{$proc}}) eq 2;

  # if we're making a scalability plot,
  #   (1) compute the scalability and assoicated delta from mean times
  #   (2) add additional data for the ideal curve. 
  if ($opt_g eq 's') 
  {
    my $scale;

    foreach my $app (keys(%{$hash{$proc}}))
    {
      my $base_mean    = $hash{$base_proc}{$app}{'mean'};
      my $base_m_delta = $hash{$base_proc}{$app}{'mean_delta'};

      my $mean         = $hash{$proc}{$app}{'mean'};
      my $m_delta      = $hash{$proc}{$app}{'mean_delta'};
     
      $scale = sprintf("%.4f", $base_mean / $mean),

      $hash{$proc}{$app}{'scale'}       = $scale;
      $hash{$proc}{$app}{'scale_delta'} = 
        sprintf("%.4f",
                (abs(($base_mean - $base_m_delta)/($mean + $m_delta) - $scale) +
                 abs(($base_mean + $base_m_delta)/($mean - $m_delta) - $scale))/2);
    }

    $hash{$proc}{'ideal'}{'scale'}       = $proc / $base_proc;   
    $hash{$proc}{'ideal'}{'scale_delta'} = 0;
  }
}


#
# header - common fields of time and scalability
#
my $script  = "hold on\n\n";
$script .= "inserts = [".join("; ", @sorted_procs)."];\n\n";

# generate data sets and plots
foreach (sort keys %colors)
{
  $script .= $_."_values = [".join("; ", extract($_, $value_field))."];\n";
  $script .= $_."_delta  = [".join("; ", extract($_, $value_field.'_delta'))."];\n\n";

  $script .= "h = errorbar(inserts, ".$_."_values, ".$_."_delta, '".$colors{$_}."')\n";

  # set log scale if this is a timing graph
  # 
  # FIXME - could make log log a command line parameter
  # 
  if ($opt_g eq 't') 
  {
    $script .= "errorbarlogx(.01);\n";
    $script .= "set(get(h,'Parent'), 'XScale', 'log')\n";
    $script .= "set(get(h,'Parent'), 'YScale', 'log')\n";
  }

  $script .= "\n\n";
}
 
$script .= "h = legend('".join("','", map($_ = uc($_), sort keys %colors))."'); \n\n";

my $title;

#
# timing graph generation (log log scale)
#
if ($opt_g eq 't')
{
  $script .= "axis([0, ".$max_procs * 1.05.", 0, ".ceil($max_mean)."]);\n\n";
  
#  $script .= "set(gca, 'xtick', [".join(" ", @sorted_procs)."], ";
#  $script .= "'ytick', [".join(" ", extract('ideal', 'scale'))."]);\n\n";
  
  $script .= "xlabel('Processors')\n\n";
  $script .= "ylabel('Time(sec)')\n\n";
   
  $title = "$opt_t Running Time ".ucfirst($opt_m);
}


#
# scalabilty graph generation
#
if ($opt_g eq 's')
{
  $script .= "set(h,'Location','NorthWest')\n\n";
 
  $script .= "axis([$base_proc, ".$max_procs * 1.05.", 0, $hash{$max_procs}{'ideal'}{'scale'}]);\n\n";
  
  $script .= "set(gca, 'xtick', [".join(" ", @sorted_procs)."], ";
  $script .= "'ytick', [".join(" ", extract('ideal', 'scale'))."]);\n\n";

  $script .= "n=get(gca,'Xtick');\n";
  $script .= "set(gca,'xticklabel',sprintf('%d |',n'));\n";
  
  $script .= "xlabel('Processors')\n\n";
  $script .= "ylabel('Scalability')\n\n";
  
  $title = "$opt_t Scalability ".ucfirst($opt_m);
}

#
# footer - common fields of time and scalability
#
$script .= "title('".$title."');\n\n";
$script .= "print -r600 -depsc '$opt_o';\n\n";
$script .= "exit;\n";

print $script;


sub read_times
{
  my ($filename, $app) = @_;
  
  open(DATA, "$filename");

  while (<DATA>)
  { 
    s/(^\s+)|(\s+$)//g;
    my ($procs, $mean, $delta) = split(/\s+/,$_);
    
    $procs =~ s/,//g;
    
    if ($mean > $max_mean) { $max_mean = $mean }
    
    $hash{$procs}{$app}{'mean'}       = $mean;
    $hash{$procs}{$app}{'mean_delta'} = $delta;
  }
 
  close(DATA);
}


sub extract
{
  my ($app, $field) = @_;

  my @vals = ();

  foreach my $proc (@sorted_procs)
  {
    push(@vals, $hash{$proc}{$app}{$field});
  }

  return @vals;
}

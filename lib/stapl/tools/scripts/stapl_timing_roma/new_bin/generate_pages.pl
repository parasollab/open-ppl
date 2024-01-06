#!/usr/bin/perl

use DBI;
#use strict;

require "./db_funcs.pl";
require "./utility.pl";

init_db();

sub get_dates
{
  my ($set, $mid, $num_ts) = @_;

  my $sth = $dbh->prepare(qq{
    SELECT DISTINCT S1.value AS timestamp
    FROM runs, statistics AS S1
    WHERE application_id = '7'
    AND runs.run_id = S1.run_id
    AND machine_id = '$mid'
    AND S1.stat_id = '81'
    AND description LIKE '$set Timing'
    ORDER BY timestamp DESC
    LIMIT $num_ts
  });

  $sth->execute();
  my @run_sets = ();
  while (my ($ts) = $sth->fetchrow_array()) { push @run_sets, $ts; }
  $sth->finish();
  @run_sets = reverse(@run_sets);

  return @run_sets;
}


sub get_histogram
{
  my ($app, $set, $mid, $container, @tstamps) = @_;
  my $palgo = get_appname($app);
  my $system = get_machinename($mid);

  my $stl_alg = $app + 74;

  my $url = "http://chart.apis.google.com/chart?cht=bvg&chs=700x300&chbh=5,1,10";
  
  &chd=t:0.92,1,0.89,0.86,0.86,0.87,0.91,0.87,0.87,0.89|0.86,0.92,0.78,0.76,0.76,0.78,0.83,0.77,0.77,0.79|0.45,0.47,0.41,0.42,0.42,0.42,0,0.41,0.42,0.43|0.25,0.25,0.25,0.24,0.24,0.24,0.25,0.24,0.24,0.27|0.16,0.16,0.15,0.15,0.15,0.15,0.15,0.16,0.17,0.15|0.12,0.12,0.12,0.12,0.12,0.12,0.12,0.12,0.12,0|0.06,0.06,0.06,0.06,0.06,0,0.06,0.08,0.06,0.07|0.04,0.04,0.04,0.03,0.18,0.06,0.04,0.05,0.12,0.03&chco=&chtt=p_for_each+on+hydra+using+p_array&chbh=5,1&chco=00FF00&chds=0,1&chxt=y,y,x&chxl=0:|0|0.1|0.2|0.3|0.4|0.5|0.6|0.7|0.8|0.9|1|1:|Time+(sec)|2:|06/10|06/11|06/12|06/13|06/14|06/15|06/16|06/17|06/18|06/19|&chxp=1,50&chg=0,10,0



}

sub generate_timing_page
{
  my ($file, $machine, $set, $type, $container) = @_;
  my $mid = get_machineid($machine);
  my $num_ts = 10;

  print $file "<?php include (\"header_rwergergroup.incl\"); ?>\n";
  print $file "<title>STAPL Timing</title>\n";
  print $file "<TABLE BORDER=\"1\">\n";
  print $file "<TR>\n";
  print $file "<TD width = \"100\" rowspan = \"4\">\n";
  print $file "  <center>\n";
  print $file "  <H3>Details</H3>\n";

  my @tstamps = get_dates($set, $mid, $num_ts);

  my $old_ts = 0;
  for (my $i = 0; $i <= $#tstamps; $i++) {
    $ts = $tstamps[$i];
    $ts_formatted = $tstamps[$i];
    $ts_formatted =~ s,(\d\d\d\d)(\d\d)(\d\d)(\d\d\d\d)(\d\d),$2/$3/$1,;
    print $file "    <a href=\"details.php?set=$set&ts=$ts&old_ts=$old_ts&type=$type\">$ts_formatted</a><br>\n";
    $old_ts = $ts;
  }
  print $file "  </center>\n";
  print $file "</TD>\n";

  if ("palgorithm" eq $type) {
    if ("nightly" eq $set) {
      $url = get_histogram(7,$set,$mid,$container,@tstamps);
      print $file "<TD> historgram image goes here.</TD></TR>\n";
      print $file "<TD> historgram image goes here.</TD></TR>\n";
      print $file "<TD> historgram image goes here.</TD></TR>\n";
      print $file "</TABLE>";
    }
  }
  print $file "<?php include(\"footer.incl\"); ?>\n";
}

#####################################
# main program to generate the files
#####################################

open(TIMING,">./timing_hydra_nightly_palgorithm_p_array.php");
my $fh = *TIMING;
&generate_timing_page($fh, "hydra", "nightly", "palgorithm", "p_array");
close(TIMING);

open(TIMING,">./timing_hydra_nightly_palgorithm_p_vector.php");
my $fh = *TIMING;
&generate_timing_page($fh, "hydra", "nightly", "palgorithm", "p_vector");
close(TIMING);

open(TIMING,">./timing_hydra_nightly_palgorithm_p_matrix.php");
my $fh = *TIMING;
&generate_timing_page($fh, "hydra", "nightly", "palgorithm", "p_matrix");
close(TIMING);

close_db();

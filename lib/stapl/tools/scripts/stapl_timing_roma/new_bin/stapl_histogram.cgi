#!/usr/bin/perl

use DBI;
use CGI qw(:standard);
use GD::Graph::mixed;
use GD::Graph::colour qw( :files );

require "./db_funcs.pl";
require "./utility.pl";

$query = new CGI;
$app = $query->param('appid');
$mid = $query->param('mid');
$timing_set = $query->param('set');

#print "Content-type: text/html\n\n";

GD::Graph::colour::read_rgb( "/usr/X11R6/lib/X11/rgb.txt" ) or die( "Can't read colours" );

init_db();

$palgo = get_appname($app);
$system = get_machinename($mid);

my $sth = $dbh->prepare(qq{ 
SELECT DISTINCT S1.value AS timestamp
                FROM runs, statistics AS S1            
                WHERE application_id = '$app'
                AND runs.run_id = S1.run_id 
                AND machine_id = '$mid'
                AND S1.stat_id = '81'
                AND description LIKE '$timing_set Timing'
                ORDER BY timestamp DESC
                LIMIT 10
});


$sth->execute();
@run_sets = ();
while (my ($ts) = $sth->fetchrow_array()) { push @run_sets, $ts; }
$sth->finish();
@run_sets = reverse(@run_sets);

@glist = ([],[],[]);
$y_max = 0;
foreach $ts (@run_sets) {
   $ts =~ m/(\d\d\d\d)(\d\d)(\d\d)(\d\d)(\d\d)(\d\d)/;
   $print_ts = "          $2/$3/$1";

   # stat_id 1 is num_procs
   # stat_id 81 is timing_group
   # stat_id 5 is exec_time
   # queries return the run_ids we are interested in
   @runs = ();

   #grab the STL run_id
   $stl_app = $app + 74;
   $sth = $dbh->prepare(qq{ SELECT runs.run_id
     FROM runs, statistics AS S0,statistics AS S1
     WHERE runs.application_id = '$stl_app' 
     AND S1.run_id = runs.run_id AND S1.stat_id = '81'
     AND S0.run_id = runs.run_id AND S0.stat_id = '5'
     AND description = '$timing_set Timing' AND S1.value = '$ts' 
     GROUP BY runs.run_id});

   $sth->execute();
   while (my ($run_id) = $sth->fetchrow_array()) { push @runs, $run_id;  }
   $sth->finish();

   #now grab the parallel runs
   $sth = $dbh->prepare(qq{ SELECT runs.run_id
     FROM runs, statistics, statistics AS S0,statistics AS S1
     WHERE runs.application_id = '$app' 
     AND runs.run_id = statistics.run_id AND statistics.stat_id = '1' 
     AND S1.run_id = runs.run_id AND S1.stat_id = '81'
     AND S0.run_id = runs.run_id AND S0.stat_id = '5'
     AND description = '$timing_set Timing' AND S1.value = '$ts' 
     GROUP BY runs.run_id});

   $sth->execute();
   while (my ($run_id) = $sth->fetchrow_array()) { push @runs, $run_id;  }
   $sth->finish();

   $flag = 0;
   foreach (@runs) {
      $sth = $dbh->prepare(qq{ SELECT statistics.value
             FROM statistics
             WHERE statistics.stat_id = '5'
             AND statistics.run_id = $_});  
      $sth->execute();
      while (my @row = $sth->fetchrow_array()) { 
         if (!$flag) { $flag++; push @{@glist[0]}, $print_ts; } 
         else {push @{@glist[0]}, ""; }
         push @{@glist[1]}, $row[0];
         $y_max = 0+$row[0] unless $row[0] < $y_max;
      }
      $sth->finish();
   }
   
   push @{@glist[0]}, "  "; push @{@glist[1]}, 0;
}  

$y_max = sig_ceil($y_max, 2);

my $graph = GD::Graph::mixed->new(700, 300);
$graph->set(title           => "$palgo on $system", 
            types           => [ qw(bars)], 
            dclrs           => [qw(green)],
            y_max_value     => "$y_max",
            y_tick_number   => 10,
            long_ticks      => 1,
            y_label         => "Time (sec)");

#$graph->set_legend( "Execution Time", "Sweep");

my $format = $graph->export_format;
print header("image/$format");
binmode STDOUT;
print $graph->plot(\@glist)->$format();

close_db();

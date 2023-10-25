#!/usr/bin/perl
=head1 SYNOPSIS

  STAPL_Performance_Verifier.pl
    This script scans the database and determines if a test is performing 
    outside of desired paramters. If it is, an email is generated telling us 
    of our impending doom. 
    
  Usage:
    STAPL_Performance_Verifier.pl [options]

  Options:
    -debug
      Shows debug info

    -spike=s
      Sets what percentage results can deviate from interval average before 
      generating an email. Default is 10
      
    -historical=s
      Sets what percentage results can deviate from interval average before 
      generating an email. Historical average over interval is compared to 
      median value over 3x interval. Default is 2
      
    -interval=s
      Sets how many days long the comparison interval should be. Default is 30.
      
    -data_size=s
      Choose which data size you want to check: tiny, small, medium, large

    -noise_threshold=s
      Choose which value size we ignore the result as timer noise. Default is .0001
      
    -help
      Shows this help information.
=cut

use Pod::Usage;
use Getopt::Long;
use DBI;

use warnings;
use strict;

# Global Variables
my $opt_help;
my $opt_magic;
my $opt_debug;
my $opt_interval;
my $opt_spike;
my $opt_historical;
my $opt_data_size;
my $opt_noise_threshold;

my @failure_list = ();
my $failure_count = 0;
my $failed_tests = 0;
my $total_applications = 0;
my $total_tests = 0;
my $total_chances_to_fail = 0;
my $historical_interval = 3; # Historical median modifier.

# Get command line options.
my $options_result =
GetOptions ("magic"       => \$opt_magic,
            "interval=s" => \$opt_interval,
            "data_size=s" => \$opt_data_size,
            "spike=s" => \$opt_spike,
            "historical=s" => \$opt_historical,
            "noise_threshold=s" => \$opt_noise_threshold,
            "debug"        => \$opt_debug,
            "help"        => \$opt_help);

die "Invalid option specification" if (!$options_result);

# Display help is -help option was provided.
if ($opt_help)
{ pod2usage(); exit; }

# Set default values if needed.
if (not defined ($opt_spike))
{
  $opt_spike = 10.0;
}
$opt_spike = $opt_spike / 100.0; # Convert to percentage.
$opt_spike = $opt_spike +1 ; # Make it a multiplier.

if (not defined ($opt_interval))
{
  $opt_interval = 30;
}

if (not defined ($opt_noise_threshold))
{
  $opt_noise_threshold = .0001;
}

if (not defined ($opt_data_size))
{
  $opt_data_size = "medium";
}

if (not defined ($opt_historical))
{
  $opt_historical = 2;
}
$opt_historical = $opt_historical / 100; # Convert to percentage.
$opt_historical = $opt_historical +1 ; # Make it a multiplier.

# Database Setup
my $dsn = 'DBI:mysql:stapl_new_perf:pdbsrv.cs.tamu.edu';
my $db_user_name = 'nthomas';
my $db_password = 'howdy';
my $dbh =  DBI->connect($dsn, $db_user_name, $db_password)
  || die "connect did not work: $DBI::errstr";

#Make a list of all the tests we have run in the last 30 days.
my $query = $dbh->prepare("SELECT DISTINCT r.machine_id, r.application_id, t.name, t.version, tr.stat_id FROM runs r, tests t, test_results tr WHERE r.run_id=t.run_id AND t.test_id=tr.test_id AND r.data_size = ? AND r.timestamp > CURDATE() - INTERVAL ? DAY");
$query->bind_param( 1, $opt_data_size );
$query->bind_param( 2, $opt_interval );
$query->execute();

## Check each test for borkedness. If borked, add to failure list.
while (my @data = $query->fetchrow_array()) 
{
  if($opt_debug)
  {
    print "Test-> opt_interval:$opt_interval data[0]:$data[0] data[1]:$data[1] data[2]:$data[2] data[3]:$data[3] data[4]:$data[4]\n";
  }
  
  my $this_test_fail = 0;
  my $fail_header = "";
  my $failure_line = '';
  
  $total_tests++; #count total # of tests.
  
  # Get core counts for test
  my $core_count_stmt = $dbh->prepare("SELECT distinct r.processor_count FROM runs r, tests t, test_results tr WHERE r.run_id=t.run_id AND t.test_id=tr.test_id AND r.timestamp > CURDATE() - INTERVAL ? DAY AND r.machine_id=? AND r.application_id=? AND t.name=? AND t.version=? AND tr.stat_id=? AND r.data_size = ? ORDER BY r.processor_count ASC");
  $core_count_stmt->bind_param( 1, $opt_interval ); #Placeholders numbered from 1, because Perl sucks.
  $core_count_stmt->bind_param( 2, $data[0] );
  $core_count_stmt->bind_param( 3, $data[1] );
  $core_count_stmt->bind_param( 4, $data[2] );
  $core_count_stmt->bind_param( 5, $data[3] );
  $core_count_stmt->bind_param( 6, $data[4] );
  $core_count_stmt->bind_param( 7, $opt_data_size );
  $core_count_stmt->execute();

  while (my @core_data = $core_count_stmt->fetchrow_array()) 
  {

    if($opt_debug)
    {
      print "Test-> cores: $core_data[0]\n";
    }
    
    $total_chances_to_fail++; # count chances to fail a test
  
    # Get average
    my $past_test_stmt = $dbh->prepare("SELECT avg(tr.value) FROM runs r, tests t, test_results tr WHERE r.run_id=t.run_id AND t.test_id=tr.test_id AND r.timestamp > CURDATE() - INTERVAL ? DAY AND r.machine_id=? AND r.application_id=? AND t.name=? AND t.version=? AND tr.stat_id=? AND r.processor_count = ? AND r.data_size = ?");
    $past_test_stmt->bind_param( 1, $opt_interval ); #Placeholders numbered from 1, because Perl sucks.
    $past_test_stmt->bind_param( 2, $data[0] );
    $past_test_stmt->bind_param( 3, $data[1] );
    $past_test_stmt->bind_param( 4, $data[2] );
    $past_test_stmt->bind_param( 5, $data[3] );
    $past_test_stmt->bind_param( 6, $data[4] );
    $past_test_stmt->bind_param( 7, $core_data[0] );
    $past_test_stmt->bind_param( 8, $opt_data_size );
    $past_test_stmt->execute();
    
    my @interval_avg = $past_test_stmt->fetchrow_array();
    
    # Get most recent date for this test.
    $past_test_stmt = $dbh->prepare("SELECT max(r.timestamp) FROM runs r, tests t, test_results tr WHERE r.run_id=t.run_id AND t.test_id=tr.test_id AND r.timestamp > CURDATE() - INTERVAL ? DAY AND r.machine_id=? AND r.application_id=? AND t.name=? AND t.version=? AND tr.stat_id=? AND r.processor_count = ? AND r.data_size = ?");
    $past_test_stmt->bind_param( 1, $opt_interval ); #Placeholders numbered from 1, because Perl sucks.
    $past_test_stmt->bind_param( 2, $data[0] );
    $past_test_stmt->bind_param( 3, $data[1] );
    $past_test_stmt->bind_param( 4, $data[2] );
    $past_test_stmt->bind_param( 5, $data[3] );
    $past_test_stmt->bind_param( 6, $data[4] );
    $past_test_stmt->bind_param( 7, $core_data[0] );
    $past_test_stmt->bind_param( 8, $opt_data_size );
    $past_test_stmt->execute();

    my @most_recent_timestamp = $past_test_stmt->fetchrow_array();
    
    # Get most recent value for this test.
    $past_test_stmt = $dbh->prepare("SELECT tr.value FROM runs r, tests t, test_results tr WHERE r.run_id=t.run_id AND t.test_id=tr.test_id AND r.timestamp > CURDATE() - INTERVAL ? DAY AND r.machine_id=? AND r.application_id=? AND t.name=? AND t.version=? AND tr.stat_id=? AND r.timestamp=? AND r.processor_count = ? AND r.data_size = ?");
    $past_test_stmt->bind_param( 1, $opt_interval ); #Placeholders numbered from 1, because Perl sucks.
    $past_test_stmt->bind_param( 2, $data[0] );
    $past_test_stmt->bind_param( 3, $data[1] );
    $past_test_stmt->bind_param( 4, $data[2] );
    $past_test_stmt->bind_param( 5, $data[3] );
    $past_test_stmt->bind_param( 6, $data[4] );
    $past_test_stmt->bind_param( 7, $most_recent_timestamp[0] );
    $past_test_stmt->bind_param( 8, $core_data[0] );
    $past_test_stmt->bind_param( 9, $opt_data_size );
    $past_test_stmt->execute();

    my @most_recent_value = $past_test_stmt->fetchrow_array();
    
    # Calculate median value for ?x historical time. There is no easy SQL solution to medians.
    $past_test_stmt = $dbh->prepare("SELECT tr.value FROM runs r, tests t, test_results tr WHERE r.run_id=t.run_id AND t.test_id=tr.test_id AND r.timestamp > CURDATE() - INTERVAL ? DAY AND r.machine_id=? AND r.application_id=? AND t.name=? AND t.version=? AND tr.stat_id=? AND r.processor_count = ?  AND r.data_size = ?ORDER BY tr.value");
    $past_test_stmt->bind_param( 1, $historical_interval * $opt_interval ); #Placeholders numbered from 1, because Perl sucks.
    $past_test_stmt->bind_param( 2, $data[0] );
    $past_test_stmt->bind_param( 3, $data[1] );
    $past_test_stmt->bind_param( 4, $data[2] );
    $past_test_stmt->bind_param( 5, $data[3] );
    $past_test_stmt->bind_param( 6, $data[4] );
    $past_test_stmt->bind_param( 7, $core_data[0] );
    $past_test_stmt->bind_param( 8, $opt_data_size );
    $past_test_stmt->execute();
    
    my @median_list = ();
    while (my @median_data = $past_test_stmt->fetchrow_array()) 
    {
      push(@median_list, $median_data[0]);
      if ($opt_debug)
      {
        print "Pushing $median_data[0] \n";
      }
    }
    my $median_value = $median_list[$#median_list/2];
    if ($median_value == 0)
    {
      $median_value = .00000001;
    }
    # See if we failed.
    if (($most_recent_value[0]  > ($interval_avg[0]* $opt_spike)) || ($interval_avg[0]  > ($median_value* $opt_historical)))
    {
      if ($opt_debug)
      {
        print "Testing: ($most_recent_value[0]  > ($interval_avg[0]* $opt_spike)) || ($interval_avg[0]  > ($median_value* $opt_historical))\n";
      }
      
      # There is a noise threshold, so tests with near zero time don't bung up the results with timer noise. (these tests are generally not timed or have broken timers.
      if(($most_recent_value[0] > $opt_noise_threshold) && ( $interval_avg[0] > $opt_noise_threshold) )
      {
        # Get app name
        my $fail_app_stmt = $dbh->prepare("SELECT name FROM applications WHERE app_id=?");
        $fail_app_stmt->bind_param( 1, $data[1] ); #Placeholders numbered from 1, because Perl sucks.
        $fail_app_stmt->execute();
        my @app_name = $fail_app_stmt->fetchrow_array();
        
        # Get machine name
        my $fail_machine_stmt = $dbh->prepare("SELECT name FROM machines WHERE machine_id=?");
        $fail_machine_stmt->bind_param( 1, $data[0] ); #Placeholders numbered from 1, because Perl sucks.
        $fail_machine_stmt->execute();
        my @machine_name = $fail_machine_stmt->fetchrow_array();
       
        # Get stat name
        my $fail_stat_stmt = $dbh->prepare("SELECT name FROM statistics_info WHERE stat_id=?");
        $fail_stat_stmt->bind_param( 1, $data[4] ); #Placeholders numbered from 1, because Perl sucks.
        $fail_stat_stmt->execute();
        my @stat_name = $fail_stat_stmt->fetchrow_array();

        $fail_header = "\n$app_name[0] ($data[1]) $machine_name[0] ($data[0]) $data[2] $data[3] $stat_name[0] ($data[4]) \n";
        $fail_header .= "https://parasol.tamu.edu/groups/rwergergroup/intranet/stapl_perf/performance.php?machineid=$data[0]&appid=$data[1]&test=$data[2]&version=$data[3]&data_size=$opt_data_size\n";
        $failure_line .= "  Cores: $core_data[0], $most_recent_timestamp[0] Test: $most_recent_value[0], $opt_interval day average: $interval_avg[0], " . $historical_interval * $opt_interval . " day median: $median_value \n";
        if ($most_recent_value[0]  > ($interval_avg[0]* $opt_spike))
        {
          $failure_line .= "    Most recent value greater than $opt_spike times $opt_interval day average: $most_recent_value[0] = ";
          $failure_line .= $most_recent_value[0] / $interval_avg[0];
          $failure_line .= " * $interval_avg[0] \n";
        }
        if ($interval_avg[0]  > ($median_value* $opt_historical))
        {
          $failure_line .= "    The $opt_interval day average is more than $opt_historical times ";
          $failure_line .= $historical_interval * $opt_interval;
          $failure_line .= " day median value: $interval_avg[0] = ";
          $failure_line .= $interval_avg[0] / $median_value;
          $failure_line .= " * $median_value \n";
        }
        
        $failure_count++;
        $this_test_fail++;
      }
    }
  }
  
  if ($this_test_fail > 0)
  {
    $failed_tests++;
    $fail_header .= $failure_line;
    push(@failure_list, $fail_header);
  }
  
}

print "Total of $failure_count failures in $failed_tests failed tests (out of $total_chances_to_fail chances to fail in $total_tests total tests). Failure rates: ";
print $failure_count/$total_chances_to_fail . "%, " . $failed_tests/$total_tests . "%. \n";
foreach my $failure (@failure_list)
{
  print $failure;
}


#Close down DB connection gracefully.
$dbh->disconnect();

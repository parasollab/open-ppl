#!/usr/bin/perl

=head1 SYNOPSIS

  STAPL_Performance_Master_Script.pl
    This script drives the main STAPL performance testing. It is primarily 
    intended for running on Rain, but it supports other systems, including 
    Dell Optiplexes like cali.
    
  Usage:
    STAPL_Performance_Master_Script.pl [options]

  Options:
    -debug
      Shows debug info. Prevents SVN updating and gmaking.

    -help
      Shows this help information.
=cut

use strict;
use Cwd qw(abs_path);
use File::Basename;
use Sys::Hostname;
use Env::Modulecmd;
use POSIX;
use Getopt::Long;
use Pod::Usage;

my $opt_debug;
my $opt_help;
my $opt_data_size;

# Get command line options.
my $options_result =
GetOptions ("debug"        => \$opt_debug,
            "data_size=s"  => \$opt_data_size,
            "help"        => \$opt_help);

die "Invalid option specification" if (!$options_result);

# Display help is -help option was provided.
if ($opt_help)
{ pod2usage(); exit; }

# DEBUG option means no gmaking and no SVNing
if ($opt_debug) { print "\n\nDEBUG MODE ON! SVN and GMAKE disabled!\n\n"; }

# Defaults
my $data_size = "medium";
if ($opt_data_size)
{
  $data_size = $opt_data_size;
}

# Get the root directory for this whole process.
my $abs_path = dirname(abs_path($0));

# Find what computer we're on. If it's Rain, load some modules.
my $host = hostname;

if ($host eq "rain")
{
	Env::Modulecmd::unload (qw(PrgEnv-cray));
	Env::Modulecmd::load (qw(PrgEnv-gnu/4.2.34));
	Env::Modulecmd::unload (qw(gcc/4.4.4));
	Env::Modulecmd::unload (qw(gcc/4.7.3));
	ENV::Modulecmd::load (qw(gcc/4.8.1));
	Env::Modulecmd::load (qw(papi/5.1.2));
	$ENV{'platform'} = "CRAY_gcc";
	$ENV{'STAPL_NUM_THREADS'} = "1";
	$ENV{'stl'} = "./tools/libstdc++/4.7.2";
	$ENV{'TMPDIR'} = "/mnt/lustre/lus0/STAPL/tmp";
}

# Start recording log file. (This gets written to file at the end)
my $output .= "Running on $host from $abs_path\n";

$output .= `rm -f $abs_path/STAPL_Performance_Master_Script.log 2>&1`;
$output .= `env 2>&1`;

# Make that our current directory, in case we started from somewhere weird.
chdir $abs_path or die "Can't cd to $abs_path $!\n"; 
if( not($opt_debug) )
{
  print "svn revert -R $abs_path/stapl\n";
	$output .= `svn revert -R $abs_path/stapl`;
  print "svn update $abs_path/stapl\n";
	$output .= `svn update $abs_path/stapl`;
}

# Move to the stapl subdirectory and rebuild the STAPL core.
print "Gmake on path/stapl.\n";
chdir "$abs_path/stapl";
$ENV{'PWD'} = "$abs_path/stapl";
$output .= `date 2>&1 `;
if( not($opt_debug) )
{
	$output .= `gmake clean 2>&1`;
	$output .= `gmake platform=CRAY_gcc stl=tools/libstdc++/4.7.3 2>&1`;
}

# Dump log so far before individual folder runs.
open  (my $output_filehandle, ">>", "$abs_path/STAPL_Performance_Master_Script.log");
print $output_filehandle $output;
close ($output_filehandle);

# Folders to run tests in.
my @config_list = ();
my %cores_list = ();

# Read Config file  (Spawned off of $abs_path/stapl)
# config lines likke like this: N rain /test/rel_alpha small 00:10:00 1 2 4 8
my $config_file = "$abs_path/STAPL_Performance_Master_Script.config";
open(my $config_filehandle, "<", $config_file) or die "Failed to open config file. $!";
my $this_host_is_in_config = 0;
while (my $line = <$config_filehandle>)
{
	chomp $line; # Kill end of line char
	if (substr($line,0,1) eq "#") {} # Comment, do nothing.
  elsif ((substr($line,0,1) eq "N")) # Node list
  {
    my @line_elements = split( ' ', $line); # convert line into an array
    shift(@line_elements); # Get rid of the N that designates this line as a node list
    
    # Determine if we use this line on this host
    my $line_host = shift(@line_elements);
    if ($line_host eq $host) 
    {
      $this_host_is_in_config = 1; # We will quit if this host not configed
      push(@config_list, \@line_elements);
    }
  }
}

# Ensure we have config info for this host.
if (not($this_host_is_in_config))
{
  $output .= "CRITICAL ERROR! UNABLE TO FIND HOST $host IN CONFIG FILE $config_file \n";
 	# Dump log for this folder's run.
	open  ($output_filehandle, ">>", "$abs_path/STAPL_Performance_Master_Script.log");
	print $output_filehandle $output;
	close ($output_filehandle);
  die "CRITICAL ERROR! UNABLE TO FIND HOST $host IN CONFIG FILE $config_file \n";
}

# Process individual folders
print "Beginning tests.\n";
foreach my $config_line (@config_list)
{
  my $folder = shift(@$config_line);
  my $data_size = shift(@$config_line);
  my $wall_time = shift(@$config_line);
  my @node_list = @$config_line;

	$output = "Now processing $folder ( $abs_path/stapl$folder ) Data size: $data_size \n";
  print " $output";
	chdir "$abs_path/stapl$folder";
	$ENV{'PWD'} = "$abs_path/stapl$folder";
    
  # Build test files
	if ( not($opt_debug) )
	{
    print " gmake";
		$output .= `gmake platform=CRAY_gcc stl=tools/libstdc++/4.7.3 2>&1`;
	}
	
  # Execute test files
  if ($host eq "rain")
  {
    print " on $host \n";
    foreach my $num_cores (@node_list)
    {
      print "  $num_cores cores\n";

      #Clear out old files
      $output .= `rm -f *.job`;
      $output .= `rm -f *.zout`;
      $output .= `rm -f *.results*`;
      $output .= `rm -f PENDING_JOBLIST`;
      
      # find files to execute
      my @test_files;

      # for each entry in the directory, push it onto test_files array if it is a
      # file, is executable, and a "binary" file.
      for my $file (glob("*"))
      {
        push(@test_files, $file) if (-B $file && -f $file && -x $file);
      }

      # Get known fails
      my %known_fails;
      open(KNOWNFAILS, "knownfails") or die "Couldn't open knownfails";
      foreach my $line (<KNOWNFAILS>) 
      {
        $line =~ s/\s+//g;
        $known_fails{$line} = 1;
      }
      close KNOWNFAILS;

      #find and remove known fails so we don't try to process them
      my $temp = $#test_files;
      for (my $t=$temp; $t >= 0; $t--) # Work backwards so removing entries doesn't bork list.
      {
        if( exists $known_fails{$test_files[$t]})
        {
          splice(@test_files, $t, 1); # remove known fail file from test_files.
        }
      } 
      
      # generate one batch file for each app/size combo and qsub it.
      foreach my $app (@test_files)
      {     
        #Generate batch file
        my $mppwidth = "#PBS -l mppwidth=$num_cores";
        my $mppnppn = "";
        # We are not spreading across nodes, but rather packing all processes onto as few nodes as possible.
        #if ($num_cores < 384) { $mppnppn = "#PBS -l mppnppn=".ceil($num_cores/16); }
        my $job_name = $num_cores."_$app".".job";
        my $out_file = $app.".results";
        
        # Set up walltime based on data size
        my $header = "#PBS -q batch
            #PBS -l walltime=$wall_time
            $mppwidth
            $mppnppn
            #PBS -N $out_file
            #PBS -j oe
            export STAPL_NUM_THREADS=1
            cd \$PBS_O_WORKDIR\n\n
            aprun -n $num_cores ./$app -data $data_size\n";          
        open(my $job_file_handle, ">", $job_name);
        print $job_file_handle $header;
        close ($job_file_handle);
        
        # Run job
        $output .= `qsub $job_name 2>&1`;
        sleep 10; # pause a little to see if LL can catch up and not skip jobs
        
        # Add to pending jobs list.
        open  (my $pending_joblist_filehandle, ">>", "PENDING_JOBLIST");
        print $pending_joblist_filehandle "$out_file\n";
        close ($pending_joblist_filehandle);
      }
    
      # Sleep until all jobs complete. Then exit and process results.
      my $wait_for_processing = 1; 
      my $sleep_delay = 30; # Reset sleep delay. Increases by 60 each time it sleeps.
      while ($wait_for_processing)
      {
        $wait_for_processing = 0;
        sleep($sleep_delay);
        
        # Get all the result files in the directory. Chop off the unknown .o##### bit. Convert to hash for lookup.
        my @result_files = glob("*.results.o*");
        for (my $i=0; $i < scalar(@result_files); $i++)
        {
        	$result_files[$i] =~s{\.[^.]+$}{};
        }
        my %result_file_hash = map { $_ => 1 } @result_files; # convert to hash. Exists on arays is deprecated.
  
        my $pending_job_names;
        open (my $pending_joblist_filehandle, "<", "PENDING_JOBLIST");
        foreach my $line (<$pending_joblist_filehandle>) 
        {
        	chomp($line);
        	if(not(exists($result_file_hash{$line}))) 
        	{ 
         		$wait_for_processing += 1; 
            $pending_job_names .= $line . "  ";
        	} # glob lets us match a wildcard to check existance
        }
        if ($wait_for_processing) 
        { 
          $sleep_delay += 60;
          print "   Waiting on $wait_for_processing processes: $pending_job_names (will sleep $sleep_delay)\n";
          $output .= "   Waiting on $wait_for_processing processes: $pending_job_names (will sleep $sleep_delay)\n";
        }
        close($pending_joblist_filehandle);
      }
       
      # Standardize all filenames to get rid of the .o####### crud.
      my @rename_files = glob("*.results.o*");
			foreach my $rename_file (@rename_files) 
			{
				my $new_name = $rename_file;
				$new_name =~s{\.[^.]+$}{}; 
				rename $rename_file, $new_name;
			} 
			
      # All jobs are now done. Process those suckers.
      $output .= `$abs_path/stapl$folder/process_execution_times.pl -num_cores $num_cores -data_size $data_size -debug`;  

      # Now Tarball the completed files
      $output .= `tar -czf $abs_path/stapl$folder/$data_size.$num_cores.Results.tar.gz *.results`;
    }
    
    # Save results in the archive
    my $curr_date = strftime("%Y-%m-%d", localtime);
    $output .= `tar -czf $abs_path/stapl$folder/archive/$curr_date.results.tar.gz *.tar.gz`;

    # Delete the .gz folders.
    $output .= `rm $abs_path/stapl$folder/*.tar.gz`;
  }   
  else # non-rain default for cali and other PCs
  {
    foreach my $num_cores (@{$cores_list{$host}})
    {
      print " $num_cores ";

      $output .= "  Executing on $num_cores. \n";
      $output .= `$abs_path/stapl$folder/test_driver.pl -data $data_size -par $num_cores -generate`;
      $output .= `$abs_path/stapl$folder/process_execution_times.pl -num_cores $num_cores`;
    }
  }
  
	# Dump log for this folder's run.
  print "\n";
	open  ($output_filehandle, ">>", "$abs_path/STAPL_Performance_Master_Script.log");
	print $output_filehandle $output;
	close ($output_filehandle);
}

$output = `date 2>&1 `;
$output .= "Process complete.";
# Dump the end info to the log file.
open  ($output_filehandle, ">>", "$abs_path/STAPL_Performance_Master_Script.log");
print $output_filehandle $output;
close ($output_filehandle);

#!/usr/bin/perl

=head1 SYNOPSIS

  test_driver.pl
    Frontend script for the STAPL testing infrastructure.  Scans directory for
    tests and version information (i.e., STAPL, MPI, OpenMP, etc) and gives the
    user unified interface to call, debug variances, and gather timing
    information.

  Usage:
    test_driver.pl [options]

  Options:
    -- -option1 -option2 ...
      Arguments after a double dash will passed to all test_executables called
      by the driver.

    -brief
      Print condensed version of output showing only test name and return
      status (i.e., PASS, FAIL, KNOWN_ISSUE, SURPRISE).  If not specified, the
      default report includes information about the various versions that were
      run. Not compatible with -print-{all|times|notes} family of options.

    -data
      Specify the data size to use.  Valid values are {tiny,small,medium,huge}.

    -debug
      Compare the output of two versions of a test.  Differences are reported to
      stdout.  Requires a single -test parameter and two -version parameters. A
      single -version is allowed when combined with -reference.

    -generate
      Generate timing files as output
      
    -just-print
      Do not actually invoke any test binaries. Just print to stdout the list of
      command that would be called.

    -noalert
      executes tests as requested but does not send an alert email when one
      would otherwise be expected.

    -noref
      Disables comparison of STAPL results with any reference implementation
      (i.e., STL, MPI, etc). Cannot be combined with -reference.

    -out filename
      Write output of a single test version to the specified file.  Exactly one
      -test paramater and one -version parameter must be specified.

    -par
      Specify the degree of parallelism to use.

    -print-all
      Print both version running times and any notes in addition to standard
      information.  Equivalent to --print-notes --print-times.

    -print-notes
      Print version notes in addition to standard information.

    -print-times
      Print version running times in addition to standard information.

    -reference filename
      Define a file to use as the reference implementation.  If not specified,
      look for file named {test}.{data}.ref to use as reference.
      Can not be combined with -noref.

    -test test_name
      Specify a test to run.  Parameter is optional; if it isn't specified,
      all tests found in a directory will be executed. Parameter can occur
      multiple times to run multiple tests in same invocation. Unless one or
      more -version parameters are specified, all versions of a test that are
      found will be executed.

    -timeout value
      Specify custom time allowance (in seconds) for test version to be given
      to complete before they are deemed hung and killed.

    -tolerance value
      Specify acceptable tolerance of difference in floating point values when
      comparing two versions.  If none is specified and floating point values
      are encountered, a default of 1E-5 is used.

    -version version_name
      Specify a version that of tests to run.  Parameter is optional; if it
      isn't specified, all versions found for each test will be executed.
      Parameter can occur multiple times to run multiple versions in same
      invocation.  Cannot be coupled with -noref.

    -help
      Shows this help information.
=cut

use Getopt::Long;
use IPC::Run qw(run timeout);
use Data::Dumper;
use JSON qw(decode_json);
use Pod::Usage;
use File::Temp qw(tempfile tempdir);
use Cwd;
use Scalar::Util::Numeric qw(isnum isint isfloat);

use warnings;
use strict;

#
# TODO - Allow implicit use of default reference file in debug mode. And perhaps
# implicit use of stapl if no -version parameters are specified at all.
#
# TODO - json from files - allow timeout specification...
#
# TODO - either change json format to export version info on file level
# granularity or verify that all tests in file have same version.
#
my $b_verbose          = 0;
my $b_run_all_versions = 0;

my @tmp_files;


# interrupt handler in case control-c is called.  Clean up temporary files
# and exit.
$SIG{INT} = sub { unlink(@tmp_files); exit; };

my $opt_data;
my $opt_par;
my @opt_tests;
my @opt_versions;
my $opt_just_print;
my $opt_generate; # defined: generate output, undefined: do not.
my $opt_out;
my $opt_noref;
my $opt_debug;
my $opt_timeout;
my $opt_noalert;
my $opt_reference;
my $opt_verbose;
my $opt_brief;
my $opt_print_times;
my $opt_print_notes;
my $opt_print_all;

my $opt_tolerance;
my $use_tolerance;

my %known_tests;
my %known_files;
my %known_fails;

# tolerance to use if non is defined on command line.
my $default_tolerance = 0.00001;

my $help;
my $delimit = ' ';
my $num_debug_diff = 300;

# maximum number of errors to report in debug mode before bailing.
my $error_threshold = 100;


# condition print passed string parameter if verbose reporting requested.
sub vprint
{
  print $_[0] if (defined $opt_verbose);
}


# strip version suffix to find base name of file set.
sub find_filename_prefix
{
  my $filename = $_[0];

  $filename =~ s/_mpi$//;
  $filename =~ s/_pth$//;
  $filename =~ s/_omp$//;
  $filename =~ s/_stl$//;
  $filename =~ s/_con$//;
  $filename =~ s/_manual$//;

  return $filename;
}


# detect file version type based on suffix.
sub filename_to_version
{
  my $filename = $_[0];

  if ($filename =~ /_mpi/)     { return 'mpi'; }
  if ($filename =~ /_manual/)  { return 'manual'; }
  if ($filename =~ /_pth/)     { return 'pth'; }
  if ($filename =~ /_omp/)     { return 'omp'; }
  if ($filename =~ /_stl/)     { return 'stl'; }
  if ($filename =~ /_con/)     { return 'con'; }

  # default to stapl when no matching suffix qualifier found.
  return 'stapl';
}


# map a version to a filename, given the filename base.  Append _version_name
# unless it's stapl.
sub version_to_filename
{
  my $base_filename = $_[0];
  my $version       = $_[1];

  if ($version eq 'stapl') {
    return $base_filename;
  }

  return $base_filename."_".$version;
}


# map a version to the command to run it.  Also take P as parameter and inject
# it properly into the command string returned.
sub version_to_command
{
  my $version     = $_[0];
  my $parallelism = $_[1];

  if ($version eq 'stapl') { return ("mpirun", "-np", "$parallelism"); }
  if ($version eq 'mpi')   { return ("mpirun", "-np", "$parallelism"); }
  if ($version eq 'omp')   { die "omp command not implemented"; }
  if ($version eq 'stl')   { die "stl command not implemented"; }
}


# Run a command in the shell, limiting its exec time to a specified number of
# seconds.  Return stdout and boolean of whether it finished before the timeout.
sub run_with_timeout
{
  my @command     = @{$_[0]};
  my $timeout     = $_[1];
  my $finished    = 1;
  my $return_code = 0;
  my $h;
  my $in;
  my $stdout;
  my $stderr;

  eval {
    $h = run \@command, \$in, \$stdout, \$stderr, timeout($timeout)
      or die "problem spawning via run process said:\n$stderr";

    # don't know why this is needed, but it is...
    $return_code = $? / 256;
  };

  if ($@) {
    if ($@ =~ "problem spawning via run") {
      die "$@";
    }

    $h->kill_kill;
    $finished = 0;
  }

  return ($finished, $stdout, $stderr, $return_code);
}


# remove leading and trailing whitespace from a string.
sub trim_whitespace
{
  my $string = $_[0];
  $string =~ s/^\s+//;
  $string =~ s/\s+$//;

  return $string;
}


# parses output from execution (passed as parameter) and build test in data
# structure of this form:
#
# [
#   'name'   : testname
#   'status' : result
#   versions : [ 'name' : version_name, 'time' : running_time ]
#   notes    : [ note ]
# ]
sub parse_test_output
{
  my $test_output       = $_[0];
  my @test_results;

  foreach my $line (split("\n", $test_output))
  {
    $line = trim_whitespace($line);

    next if ($line !~ /:/);

    (my $tag, my $value) = split(":", $line, 2);

    $tag      = trim_whitespace($tag);
    $value    = trim_whitespace($value);
    my $uctag = uc($tag);

    if ($uctag eq "TEST") {
      my @versions = ();
      my $test_record = {'name' => $value, 'versions' => \@versions};
      push(@test_results, $test_record);
    }
    elsif ($uctag eq "STATUS") {
      $test_results[-1]{status} = uc($value);
    }
    elsif ($uctag eq "VERSION") {
      my @notes       = ();
      my $version_ref = {'name' => $value, 'notes'=> \@notes};
      push @{$test_results[-1]{versions}}, $version_ref;
    }
    elsif ($uctag eq "NOTE") {
      push @{$test_results[-1]->{versions}[-1]{notes}}, $value;
    }
    elsif ($uctag eq "TIME") {
     $test_results[-1]{versions}[-1]->{time} = $value;
    }
  } # foreach (split("\n", $test_output))

  return \@test_results;
}


# scan the test result data structure, incorporating information about known
# fails.  Print out report as directed by command line options.
sub process_test_output
{
  my $test_info_ref = $_[0];
  my $filename      = $_[1];

  my $return_code = 0;

  if (defined $opt_generate)
  {
    open(RESULTFILE, '>', "$filename.results") 
      or die "Could not open $filename.results for writing.";
  }
  foreach my $test (@{$test_info_ref})
  {
    my $status        = $test->{status};
    my $test_name     = $filename."::".$test->{name};
    my $b_known_issue = exists $known_fails{$test_name};

    print "$test_name : ";

    if (defined $opt_generate)
    {
      print RESULTFILE "$test->{name} $test->{status}";
      foreach my $ver (@{$test->{versions}})
      {
        print RESULTFILE " $ver->{name} $ver->{time}";
      }
      print RESULTFILE "\n";
    }
    if ($test->{status} =~ /NA/)
    { print "NA\n"; }
    elsif ($test->{status} =~ /PASS/) {
      if ($b_known_issue)
      { print "SURPRISE\n" }
      else
      { print "PASS\n"; }
    }
    elsif ($test->{status} =~ /FAIL/) {
      if ($b_known_issue)
      { print "KNOWN_ISSUE\n"; }
      else
      {
        print "FAIL\n";
        $return_code = 1;
      }
    }

    foreach my $version (@{$test->{versions}}) {

      print "version : $version->{name}\n" unless (defined $opt_brief);
      print "time : $version->{time}\n"    if (defined $opt_print_times);

      if (defined $opt_print_notes) {
        foreach (@{$version->{notes}}) {
          print "note : $_\n";
        }
      }
    }

    if (!defined $opt_brief
        || defined $opt_print_notes
        || defined $opt_print_times)
    { print "\n"; }
  }

  if (defined $opt_generate)
  {
    close(RESULTFILE);
  }
  return $return_code;
}


# given a command to run for testing, invoke it call subroutine to parse the
# output, report results, set driver return code appropriately.
sub run_test_command
{
  my @command      = @{$_[0]};
  my $timeout      = $_[1];
  my $return_value = $_[2];
  my $filename     = $_[3];

  @command = (@command, @ARGV);

  if (defined $opt_just_print) {
    print "@command\n";
    return;
  }

  vprint "Running ===> @command\n";

  (my $finished, undef, my $output, my $return_code) =
    run_with_timeout(\@command, $timeout);

  my $test_info_ref = parse_test_output($output);

  if (process_test_output($test_info_ref, $filename)) {
    print "FAIL: (@command) had an error\n";
    $return_value = 1;
  }

  if ($return_code) {
    print "FAIL: (@command) return code = $return_code\n";
    ${$return_value} = $return_code if ($return_code > ${$return_value});
  }

  if (!$finished) {
    print "FAIL (@command) did not finish in alloted time\n";
    ${$return_value} = 3;
  }

  return;
}


# Given a test file, check for an associated config file to get a list of tests
# and versions that it supplies.  If config file doesn't exists, query the
# binary.  In either case, return the json data decoded into a perl structure.
sub json_tests_descriptor
{
  my $filename = $_[0];

  if (-e "$filename.json") {
    open FILE, "$filename.json" or die "Couldn't open file: $!";
    my $json_tests_str = join("", <FILE>);
    close FILE;
    return decode_json($json_tests_str);
  }

  # else
  my $file_version = filename_to_version($filename);

  # use 1 processor.  We're just querying, not really running.
  my @file_command = version_to_command($file_version, 1);

  push(@file_command, "$filename", "-l");

  (my $finished, my $json_tests_str, undef, my $return_code) =
    run_with_timeout(\@file_command, 3);

  unless (!$return_code) { die "Failed to query file $filename (ret_code!=0)"; }
  unless ($finished) { die "Failed to query file $filename (!finished)"; }

  return decode_json($json_tests_str);
}


# Search the file specified by the filename input parameter, and populate
# known_tests with information about the test / versions that the given file
# provides.
sub extract_version_info
{
  my $known_tests           = $_[0];
  my $filename              = $_[1];

  my $filename_prefix       = find_filename_prefix($filename);
  my $file_tests_descriptor = json_tests_descriptor($filename);
  die "Invalid JSON format." unless (exists ${$file_tests_descriptor}{'tests'});

  $file_tests_descriptor = ${$file_tests_descriptor}{'tests'};

  my $num_tests                          = @{$file_tests_descriptor};
  my $num_stapl_versions                 = 0;
  my $num_self_validating_stapl_versions = 0;

  my @file_versions;

  for my $test (@{$file_tests_descriptor}) {
    die "invalid input" unless (keys %{$test} == 1);

    my $test_name        = (keys %{$test})[0];
    my $stored_test_name = $filename_prefix."::".(keys %{$test})[0];

    if (!exists ${$known_tests}{$stored_test_name}) {
      ${$known_tests}{$stored_test_name} = { };
    }

    my $num_versions = @{${$test}{$test_name}};

    for my $version (@{${$test}{$test_name}}) {
      die "no version name specified"
        unless (exists ${$version}{'name'});

      my $version_name = ${$version}{'name'};


      if (exists ${${$known_tests}{$stored_test_name}}{$version_name}) {
        die "version $version_name already exists for test $stored_test_name";
      }

      ${${$known_tests}{$stored_test_name}}{$version_name} =
        { 'filename' => $filename };

      if ($version_name eq "stapl") {
        ++$num_stapl_versions;

        if ($num_versions > 1) {
          ++$num_self_validating_stapl_versions;

          ${${$known_tests}{$stored_test_name}}{$version_name}
            {'self_validate'} = 1;
        }
        else {
          ${${$known_tests}{$stored_test_name}}{$version_name}
            {'self_validate'} = 0;
        }
      }
    }
    @file_versions = @{${$test}{$test_name}};
    @file_versions = map {${$_}{'name'}} @file_versions;
  }

  return @file_versions if ($num_stapl_versions == 0);

  if ($num_stapl_versions != $num_tests) {
    die "$filename has some stapl versions but not all";
  }

  return @file_versions if ($num_self_validating_stapl_versions);

  if ($num_stapl_versions != $num_self_validating_stapl_versions) {
    die "Some, but not all stapl versions in $filename are self_validating";
  }

  return @file_versions;
}


# verify for every requests test/version combination that I found an
# implementation for it to use.
sub validate_test_and_version_info
{
  my $req_tests    = $_[0];
  my $req_versions = $_[1];
  my $known_tests  = $_[2];

  for my $test (@{$req_tests})
  {
    die "Couldn't find a requested test: $test."
      unless (exists ${$known_tests}{$test});

    my $known_versions = ${$known_tests}{$test};

    for my $version (@{$req_versions})
    {
      die "Couldn't find a version $version for test $test."
        unless (exists ${$known_versions}{$version});
    }
  }
}


# compare the results of two files containing results from test execution.
sub compare_files
{
  my $file1            = $_[0];
  my $file2            = $_[1];
  my $delimit          = ${$_[2]};
  my $tolerance        = ${$_[3]};
  my $b_print_debug    = $_[4];
  my $report_threshold = ${$_[5]};

  my $line1;
  my $line2;

  my $error_found = 0;

  open(DATA1, "<$file1") or die "Cannot open $file1";
  open(DATA2, "<$file2") or die "Cannot open $file2";

  my $line_number = 0;
  my $finished    = 0;

  while ($finished != 1)
  {
    ++$line_number;

    my $line1 = <DATA1>;
    chomp $line1;
    my @items1 = split(/$delimit/, $line1);

    $line2 = <DATA2>;
    chomp $line2;
    my @items2 = split(/$delimit/, $line2);

    @items1=grep { !/^$/ } (@items1);
    @items2=grep { !/^$/ } (@items2);

    if (@items1 != @items2) {
      if ($b_print_debug) {
        print "Different item count on line $line_number\n";
      }

      ++$error_found;
    }


    # check a given line.
    my $print_lines = 0;

    for (my $idx=0; $idx<scalar(@items1); ++$idx)
    {
      my $item1 = $items1[$idx];
      my $item2 = $items2[$idx];

      # verify element of version 1 is actually a number.
      if (!isnum($item1))
      {
        if ($b_print_debug) {
          print "Version 1 has non number on line $line_number ($item1).\n";
          $print_lines = 1;
        }
        ++$error_found;
      }

      # verify element of version 2 is actually a number.
      if (!isnum($item2))
      {
        if ($b_print_debug) {
          print "Version 2 has non number on line $line_number ($item2).\n";
          $print_lines = 1;
        }
        ++$error_found;
      }

      # compare floating point values.  Make sure both are floating point types
      # and that difference of values falls within tolerance.
      if (isfloat($item1) || isfloat($item2)) {
        if (!isfloat($item1) || !isfloat($item2)) {
          if ($b_print_debug) {
            print "Value type mismatch on line $line_number ($item1/$item2).\n";
            $print_lines = 1;
          }
          ++$error_found;
        }
        else {
          if (abs($item1 - $item2) > $tolerance) {
            if ($b_print_debug) {
              print "Floating point values outside tolerance ";
              print " ($item1 / $item2).\n";
              $print_lines = 1;
            }
            ++$error_found;
          }
        }
      }

      # compare integrals.
      else {
        if (!isint($item1)) {
          if ($b_print_debug) {
            print "Version 1 value ($item1) is of unknown type on ";
            print "line $line_number.\n; ";
            $print_lines = 1;
          }
          ++$error_found;
        }

        if (!isint($item2)) {
          if ($b_print_debug) {
            print "Version 2 value ($item2) is of unknown type on ";
            print "line $line_number.\n; ";
            $print_lines = 1;
          }
          ++$error_found;
        }

        if ($item1 != $item2) {
          if ($b_print_debug) {
            print "Values are different on line $line_number ";
            print "($item1 / $item2)\n";
            $print_lines = 1;
          }
          ++$error_found;
        }
      }
    }

    # if we found any errors on the line, print the two lines.
    if ($print_lines) {
      print "< $line1\n";
      print "> $line2\n";
    }

    # we're done if we've reached the end of either file.  If the other handle
    # isn't at eof, then it's an error.
    if (eof(DATA1) || eof(DATA2)) {
      if (eof(DATA1) != 1 || eof(DATA2) != 1) {
        if ($b_print_debug) {
          print "Versions have different number of lines\n";
        }

        ++$error_found;
      }

      $finished=1;
    }

    # if we are not debugging and just testing correctness, any error is
    # sufficient to stop comparison and return.
    if ($error_found > 0 && !$b_print_debug) {
      $finished = 1;
    }

    # if we're reporting errors but we've reached the print threshold, stop.
    if ($error_found >= $report_threshold) {
      $finished = 1;
    }
  }

  close DATA1;
  close DATA2;

  if ($error_found >= $report_threshold && $b_print_debug) {
    print "Found more than $report_threshold errors.  Stopping.\n";
  }

  return 1 if ($error_found);
  return 0;
}


# return true if a base_file (i.e., one containing stapl implementations) does
# not self validate (i.e., have another version besides stapl) and hence
# requires the driver to manually validate it.
sub manual_validation
{
  my $base_file = $_[0];

  # if noref, no validation takes place. Otherwise, if we're running a stapl
  # version see if the binary containing it has another version as well.
  # (Assume then that it self references...)
  #
  if (!defined $opt_noref && !defined $opt_out)
  {
    # if I'm running all versions or select versions including stapl,
    # check if stapl binary is self validating.
    #
    if ($b_run_all_versions || grep /stapl/, @opt_versions)
    {
      my $versions_in_base_file_ref = ${known_files}{$base_file}{$base_file};

      if (!grep /stapl/, @{$versions_in_base_file_ref})
      { die "$base_file doesn't have stapl version"; }

      if (@{$versions_in_base_file_ref} == 1)
      {
        vprint "$base_file has only stapl version.  ".
               "Manual validation required.";

        return 1;
      }
      else
      { vprint "$base_file self validates.\n"; }

    } else
    {
      vprint "No STAPL Version to be run, no validation needed.\n";
    }
  } else
  {
    vprint "No Validation Check Needed, -noref specified\n";
  }

  return 0;
}


sub dump_debug
{
  die "Need to update debug mode...\n";
  # from options guards, I know I only have one test to run.  Run one version
  # if -reference provided (otherwise two), compare results, and return.
  ##if (defined $opt_debug) {
  ##  my $tmp = File::Temp->new(TEMPLATE => "XXXXXX");

  ##  for my $version (@versions) {
  ##    my $executable_name = ${${$known_tests{$test}}{$version}}{'filename'};
  ##    my $file_version    = filename_to_version($executable_name);
  ##    my @file_command    = version_to_command($file_version, $opt_par);
  ##    my $out_filename    = ".".$test."_".$version."_".$tmp->filename();

  ##    push(@file_command, "$executable_name");
  ##    push(@file_command, "-noref", "-test", $test, "-version", $version);
  ##    push(@file_command, "-out", $out_filename);

  ##    if (defined $opt_tolerance) {
  ##      push(@file_command, "-tolerance", "$use_tolerance")
  ##    }

  ##    push(@tmp_files, $out_filename);

  ##    if (defined $opt_just_print) {
  ##      print "@file_command\n";
  ##      exit;
  ##    }

  ##    (my $finished, my $output) = run_with_timeout(\@file_command, $timeout);
  ##    die "@file_command filed to finish" unless ($finished);
  ##  }

  ##  my $file1;
  ##  my $file2;

  ##  if (defined $opt_reference) {
  ##    die "Unexpected tmp file count" unless (@tmp_files == 1);

  ##    $file1 = $opt_reference;
  ##    $file2 = $tmp_files[0];
  ##  }
  ##  else {
  ##    $file1 = $tmp_files[0];
  ##    $file2 = $tmp_files[1];
  ##  }

  ##  print "$file1, $file2\n";

  ##  compare_files $file1, $file2, \ $delimit,
  ##                \$use_tolerance, 1, \$error_threshold;

  ##  # cleanup any temp file and remove from signal handler cleanup list.
  ##  unlink(@tmp_files);
  ##  @tmp_files = ();
}


# if knownfails file exists in the directory, extract lists of tests and
# populate hash reference input parameter.
sub initialize_known_fails
{
  my $known_fails = $_[0];

  return unless (-e "knownfails");

  vprint "knownfails file found. initializing.\n";

  open(KNOWNFAILS, "knownfails") or die "Couldn't open knownfails";

  foreach my $line (<KNOWNFAILS>) {
    $line =~ s/\s+//g;
    ${$known_fails}{$line} = 1;
  }

  close KNOWNFAILS;
}

sub data_dump
{
  print "Debug info\n";
  print "@_\n";
}
sub hash_dump
{
  print "Debug info\n";
  my $params = shift;
  my %paramhash = %$params;  
  for my $key ( sort keys %paramhash ) {
    print "$key: $paramhash{$key}  -->";
    }   
    print "\n";
}
#
# main
#

my $options_result =
GetOptions ("data=s"      => \$opt_data,
            "par=s"       => \$opt_par,
            "test=s"      => \@opt_tests,
            "verbose"     => \$opt_verbose,
            "version=s"   => \@opt_versions,
            "out=s"       => \$opt_out,
            "tolerance=s" => \$opt_tolerance,
            "debug"       => \$opt_debug,
            "generate"    => \$opt_generate,
            "noalert"     => \$opt_noalert,
            "noref"       => \$opt_noref,
            "reference=s" => \$opt_reference,
            "just-print"  => \$opt_just_print,
            "timeout=s"   => \$opt_timeout,
            "brief"       => \$opt_brief,
            "print-times" => \$opt_print_times,
            "print-notes" => \$opt_print_notes,
            "print-all"   => \$opt_print_all,
            "help"        => \$help);

die "Invalid option specification" if (!$options_result);

if ($help)
{ pod2usage(); exit; }

my @data_list = ('tiny', 'small', 'medium', 'big', 'huge');
my %data_set  = map { $_ => 1 } @data_list;


# make sure we know about this data size.
if (!defined $opt_data || !$data_set{$opt_data})
{ die "must define valid data size (-data {tiny,small,medium,big,huge})."; }

my $timeout;

if (defined $opt_timeout)
{ $timeout = $opt_timeout; }
else {
  $timeout = 5      if ($opt_data eq "tiny");
  $timeout = 10     if ($opt_data eq "small");
  $timeout = 100    if ($opt_data eq "medium");
  $timeout = 1000   if ($opt_data eq "big");
  $timeout = 10000  if ($opt_data eq "huge");
}


if (!defined $opt_par || !$opt_par =~ /D/ ||  $opt_par <= 0)
{ die "must define valid degree of parallelism (-par N)."; }

if (defined $opt_reference) {
  die "opt_reference support needs updating.";
  die "cannot combine -reference and -noref" if (defined $opt_noref);
}


if (defined $opt_noref) {
  if  (@opt_versions != 0)
  { die "parameter -version not compatible with -noref (stapl implicit)."; }

  push(@opt_versions, 'stapl');
}

if (defined $opt_out && (@opt_tests != 1 || @opt_versions != 1))
{ die "Must define exactly one test and one version with -out"; }


if (defined $opt_debug) {
  my $num_req_versions = 2;

  if (defined $opt_reference)
  { $num_req_versions = 1; }

  if (@opt_tests != 1 || (@opt_versions != $num_req_versions)) {
   die "Debug mode requires one test and either two versions or".
       " one version and reference file";
 }
}


if (defined $opt_tolerance)
{ $use_tolerance = $opt_tolerance; }
else
{ $use_tolerance = $default_tolerance; }

if (defined $opt_brief && defined $opt_print_all)
{ die "-brief and -print_all not compatible"; }

if (defined $opt_brief && defined $opt_print_times)
{ die "-brief and -print_times not compatible"; }

if (defined $opt_brief && defined $opt_print_notes)
{ die "-brief and -print_notes not compatible"; }

if (defined $opt_print_all)
{ $opt_print_times = 1; $opt_print_notes = 1; }

my @test_files;

# for each entry in the directory, push it onto test_files array if it is a
# file, is executable, and a "binary" file.
for my $file (glob("*"))
{
  push(@test_files, $file) if (-B $file && -f $file && -x $file);
}

#find and remove known fails so we don't try to process them
initialize_known_fails(\%known_fails);
my $temp = $#test_files;
for (my $t=$temp; $t >= 0; $t--) # Work backwards so removing entries doesn't bork list.
{
  if( exists $known_fails{$test_files[$t]})
  {
    splice(@test_files, $t, 1); # remove known fail file from test_files.
  }
}

# extract test & version info from each binary
foreach my $one_file (@test_files) {
  my $test_file    = $one_file;
  my $base_file    = find_filename_prefix($test_file);
  my $file_version = filename_to_version($test_file);

  if (!exists ${known_files}{$base_file})
  { ${known_files}{$base_file} = { }; }

  my @versions_in_file = extract_version_info(\%known_tests, $one_file);
  die "duplicate file found" if (exists ${known_files}{$base_file}{$test_file});
  ${known_files}{$base_file}{$test_file} = \@versions_in_file;
}

# if not tests specified on command line, run all tests that I found when
# inspecting files.
my $b_run_all_tests = 0;

if (@opt_tests == 0) {
  $b_run_all_tests = 1;

  for my $one_test (keys %known_tests)
  { push (@opt_tests, $one_test); }
}
validate_test_and_version_info(\@opt_tests, \@opt_versions, \%known_tests);

$b_run_all_versions = 1 if (@opt_versions == 0);

my $return_value = 0;

if ($b_run_all_tests)
{
  vprint "Running All Tests\n";

  for my $base_file (sort keys %known_files)
  {
    vprint "\n\nProcessing base file $base_file\n";

    if (manual_validation($base_file))
    {
      vprint "$base_file does not self validate. Searching for ".
             "another version to validate against.";

      die "alltests && manual validation needs work.";
    }

    for my $file (sort keys %{$known_files{$base_file}})
    {
      my $file_version   = filename_to_version($file);
      my @file_command   = version_to_command($file_version, $opt_par);

      push(@file_command, "$file");
      push(@file_command, "-d", "$opt_data");
      push(@file_command, "-noref") if (defined $opt_noref);

      my $need_to_execute = 0;

      # if I'm running all versions, run every file with no -v parameter.
      if ($b_run_all_versions) {
        $need_to_execute = 1;
      }
      # if I'm running select versions, scan @opt_versions to see if this file
      # provides any of the versions I need and push a version parameter.
      else {
        for my $version (@opt_versions) {
          if (grep /$version/, @{${$known_files{$base_file}}{$file}}) {
            $need_to_execute = 1;
            push(@file_command, "-version", $version);
          }
        }
      }

      next unless ($need_to_execute);

      # arbitrarily "big value" since we're running a lot of tests.
      run_test_command(\@file_command, 1000, \$return_value, $base_file);
      next if (defined $opt_just_print);
    }
  }

  exit $return_value;
}

#
# explicitly listed tests...
#
for my $test (@opt_tests)
{
  vprint "List of tests explicitly defined on command line\n";

  my @versions;

  if ($b_run_all_versions) {
    @versions = sort keys %{$known_tests{$test}};
  }
  else {
    @versions = @opt_versions;
  }

  if ($opt_debug) {
    dump_debug();
    exit;
  }

  my @output_compare_versions;

  # check if we're running stapl versions and see who handles validation
  # If we are (i.e., the binary isn't self validating), come up with a plan.
  #
  if (grep /stapl/, @versions)
  {
    my $stapl_executable_name = ${${$known_tests{$test}}{'stapl'}}{'filename'};

    if (manual_validation($stapl_executable_name)) {
      push(@output_compare_versions, 'stapl');
      die "implement find validation version and do it quietly";
    }
  }

  # run all tests
  for my $version (@versions)
  {
    print "Running $test ($version)\n";

    my $executable_name = ${${$known_tests{$test}}{$version}}{'filename'};

    my $file_version = filename_to_version($executable_name);
    my @file_command = version_to_command($file_version, $opt_par);

    my $out_filename;

    push(@file_command, "$executable_name");

    # if we're generating a reference file, use that as the -out parameter.
    # otherwise, specify a tmp file so we can capture output and compare to the
    # reference implementation.
    if (grep /$version/, @output_compare_versions)
    {
      my $tmp           = File::Temp->new(TEMPLATE => "XXXXXX");
      my $out_filename  = ".".$test."_".$version."_".$tmp->filename();
      push(@file_command, "-out", "$out_filename");
      push(@tmp_files, $out_filename);
    }

    my $passed_test_name = $test;
    $passed_test_name =~ s/.*:://;

    push(@file_command, "-out", "$opt_out")    if (defined $opt_out);
    push(@file_command, "-noref")              if (defined $opt_noref);
    push(@file_command, "-t", "$passed_test_name");
    push(@file_command, "-v", "$version")      unless (defined $opt_noref);

    if (defined $opt_tolerance) {
      push(@file_command, "-tolerance", "$use_tolerance")
    }

    push (@file_command, "-d", "$opt_data");

    run_test_command(
      \@file_command, $timeout, \$return_value, $executable_name
    );

    next if (defined $opt_just_print);

    ### # if we're just generating reference file, we're done.  Otherwise, find a
    ### # reference file to compare this to versions against and call comparison
    ### # subroutine.
    ### else {
    ###   exit if (defined $opt_out || defined $opt_noref);

    ###   my $baseline_file;
    ###   if (defined $opt_reference) {
    ###     $baseline_file = $opt_reference;
    ###   }
    ###   else {
    ###     my $default_ref_filename = "$test.$opt_data.ref";

    ###     if (!(-e $default_ref_filename)) {
    ###       die "Cannot find reference file $default_ref_filename";
    ###     }

    ###     $baseline_file = $default_ref_filename;
    ###   }

    ###   $return_value = compare_files $baseline_file, $out_filename, \ $delimit,
    ###                                 \$use_tolerance, 1, \$error_threshold;

    ###   if ($return_value == 0) {
    ###     print "PASS\n";
    ###   }
    ###   else {
    ###     print "FAIL (does not match reference, run -debug for more details.)\n";
    ###   }

    ### }
  } # for (@versions)

  # I only use temporary files I am managing validation manually.  If this is
  # the case, and I get to files, that means I've run both stapl and the chosen
  # version for reference.
  if (@tmp_files == 2)
  {
    print "Comparing ".join(" and ", @tmp_files)." versions: ";

    my $return_code = compare_files $tmp_files[0], $tmp_files[1], \$delimit,
                                    \$use_tolerance, 1, \$error_threshold;

    if ($return_code == 0) {
      print "PASS\n";
    }
    else {
      print "FAIL\n";

      $return_value = $return_code if ($return_code > $return_value);
    }

    # cleanup any temp file and remove from signal handler cleanup list.
    unlink(@tmp_files);
    @tmp_files = ();
  }
} # for (@opt_tests)

exit $return_value;

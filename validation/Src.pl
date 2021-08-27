#!/usr/bin/perl

use Getopt::Std;
use Env::Modulecmd;
getopts('c:r:d:p:');

my $startRun = time();

#
# configuration
#
$outputdir = "/tmp/pmpl_nightly_logs";
$ADMIN_MAILTO = "jdenny\@cs.tamu.edu";
$workdir  = "/scratch/zenigata/jdenny/pmpl_nightly";
$REPO = "svn+ssh://parasol-svn.cs.tamu.edu/research/parasol-svn/svnrepository/pmpl/trunk";

#
# find out which platform is used
#
if (!defined $opt_c || !($opt_c eq "LINUX_gcc")) {
  die "must define compilation platform (-c LINUX_gcc)";
}
if (!defined $opt_r ||
  !(($opt_r eq "cfg") ||
    ($opt_r eq "surface") ||
    ($opt_r eq "reach") ||
    ($opt_r eq "multi"))) {
  die "must define robot type (-r cfg | surface | reach | multi)";
}
if (!defined $opt_d || !(($opt_d eq "0") || ($opt_d eq "1"))) {
  die "must define debugging option (-d 0 | 1)";
}
if (!defined $opt_p || !(($opt_p eq "0") || ($opt_p eq "1"))) {
  die "must define parallel option (-p 0 | 1)";
}
$PLATFORM = $opt_c;
$ROBOT = "PMPCfg";
if($opt_r eq "surface") { $ROBOT = "PMPCfgSurface"; }
if($opt_r eq "reach") { $ROBOT = "PMPReachableVolume"; }
if($opt_r eq "multi") { $ROBOT = "PMPCfgMultiRobot"; }
$DEBUG = $opt_d;
$PARALLEL = $opt_p;

#
# setup shell variables
#
if ($PLATFORM eq "LINUX_gcc") {
  $ENV{'PATH'} = "/usr/lib64/ccache:".$ENV{'PATH'};
}
if ($PARALLEL eq "1") {
  Env::Modulecmd::load(qw(stapl_dev/gcc));
  $ENV{'PATH'} = "/usr/lib64/mpich/bin:".$ENV{'PATH'};
  $ENV{'LD_LIBRARY_PATH'} = "/usr/local/boost/boost-1.56/lib64:".$ENV{'LD_LIBRARY_PATH'};
  $ENV{'BOOST_ROOT'}="/usr/local/boost/boost-1.56";
  $ENV{'STAPL_NUM_THREADS'} = 1;
}

#
# figure out time and date
#
@months   = qw(Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec);
@weekdays = qw(Sun Mon Tue Wed Thu Fri Sat);
($second, $minute, $hour, $dayOfMonth, $monthOfYear, $yearOffset, $dayOfWeek, $dayOfYear, $daylightSavings) = localtime();
$day      = $weekdays[$dayOfWeek];
$month    = $months[$monthOfYear];
$year     = 1900 + $yearOffset;
$fulldate = "$day-$month-$dayOfMonth-$year";

#
# start log file
#
$OUTPUT = "$fulldate\nplatform = $PLATFORM / ROBOT_DEF = $ROBOT / debug = $DEBUG / parallel = $PARALLEL\n";

#
# go to working dir, remove old directories, and check out new copy
# (if necessary)
#
chdir $workdir or die "Can't cd to $workdir $!\n";
$pmpldir = "$fulldate";
opendir(DIR, $workdir) or die $!;
while(my $file = readdir(DIR)) {
  if(!($file eq $pmpldir) && !($file eq ".") && !($file eq "..")) {
    $OUTPUT = $OUTPUT."Removing directory $file\n".`rm -rf $file 2>&1`;
  }
}
closedir(DIR);
if(!-e "$pmpldir") {
  $OUTPUT = $OUTPUT."Checking out repository.\n";
  while(!-e "$pmpldir") {
    $OUTPUT = $OUTPUT.`svn --quiet checkout $REPO $pmpldir 2>&1`;
  }
}
else {
  $OUTPUT = $OUTPUT."Repository already checked out, continuing compilation.\n";
}

#
# print system details
#
chdir "$pmpldir/src";
$OUTPUT = $OUTPUT."Started at ".`date 2>&1`;
$OUTPUT = $OUTPUT."g++ path: ".`which g++ 2>&1`;
$OUTPUT = $OUTPUT."g++ details:\n".`g++ -c -v 2>&1`;
if($PARALLEL eq "1") {
  $OUTPUT = $OUTPUT."mpi:".`which mpic++ 2>&1`;
  $OUTPUT = $OUTPUT."env:\n".`printenv 2>&1`;
}

#
#compile
#
$OUTPUT = $OUTPUT.`make platform=$PLATFORM ROBOT_DEF=$ROBOT debug=$DEBUG parallel=$PARALLEL reallyreallyclean 2>&1`;
$OUTPUT = $OUTPUT.`make platform=$PLATFORM ROBOT_DEF=$ROBOT debug=$DEBUG parallel=$PARALLEL pmpl -j4 2>&1`;
if(-e "$workdir/$pmpldir/src/pmpl") {
  $OUTPUT = $OUTPUT."=====\nPassed: pmpl compilation\n=====\n";
}
else {
  $OUTPUT = $OUTPUT."=====\nFailed: pmpl compilation\n=====\n";
}

#
#tests
#
if(-e "$workdir/$pmpldir/src/pmpl") {
  chdir "Test";
  $TEST = "CfgTests";
  if($ROBOT eq "PMPCfgMultiRobot") {
    $TEST = "MultiRobotTests";
  };
  if($ROBOT eq "PMPCfgSurface") {
    $TEST = "SurfaceTests";
  };
  if($ROBOT eq "PMPReachableVolume") {
    $TEST = "ReachableVolumeTests";
  };
  if($PARALLEL eq "1") {
    $TEST = "ParallelCfgTests"
  }
  $OUTPUT = $OUTPUT.`make Test=$TEST parallel=$PARALLEL test 2>&1`;
}

#
# Timing stats
#
my $endRun = time();
my $runTime = $endRun - $startRun;
$OUTPUT = $OUTPUT."\n\nTest took $runTime seconds.\n";

$OUTPUT = $OUTPUT."Done at ".`date 2>&1`;

#
# output log to /tmp
#
if (!-e "$outputdir") {
  `mkdir $outputdir`;
}
if (!-e "$outputdir/$fulldate") {
  `mkdir $outputdir/$fulldate`;
}
$outfile = "$outputdir/$fulldate/pmpl.$opt_c.$opt_r.debug$opt_d.parallel$opt_p.out";
open(OUT, ">$outfile" || die "error, could not open $outfile for reading: $!");
print OUT $OUTPUT;
close(OUT);

#
# send mail
#
open(MAIL, "|/usr/lib/sendmail -t");
print MAIL "To: $ADMIN_MAILTO\n";
print MAIL "From: pmpl_nightly\@tamu.edu\n";
print MAIL "Subject: PMPL nightly - zenigata.cse.tamu.edu pmpl.$opt_c.$opt_r.debug$opt_d.parallel$opt_p\n";
print MAIL "Done, output written to $outfile\n";
close(MAIL);


#!/usr/bin/perl
 
use Getopt::Std;
getopts('c:r:d:p:');

#
# configuration
#
$outputdir = "/tmp/pmpl_nightly_logs";
$ADMIN_MAILTO = "jdenny\@cs.tamu.edu";
$workdir  = "/scratch/antibes/jdenny/pmpl_nightly";

#
# find out which platform is used 
#
if (!defined $opt_c || !(($opt_c eq "LINUX_64_gcc") || ($opt_c eq "LINUX_gcc"))) { 
  die "must define compilation platform (-c LINUX_64_gcc | LINUX_gcc)"; 
}
if (!defined $opt_r || !(($opt_r eq "rigid") || ($opt_r eq "serial") || ($opt_r eq "reach"))) { 
  die "must define robot type (-r rigid | serial | reach)"; 
}
if (!defined $opt_d || !(($opt_d eq "0") || ($opt_d eq "1"))) { 
  die "must define debugging option (-d 0 | 1)"; 
}
if (!defined $opt_p || !(($opt_p eq "0") || ($opt_p eq "1"))) { 
  die "must define parallel option (-p 0 | 1)"; 
}
$PLATFORM = $opt_c;
$ROBOT = "-DPMPRigid";
if($opt_r eq "serial") { $ROBOT = "-DPMPSerial"; }
if($opt_r eq "reach") { $ROBOT = "-DPMPReachDistCC"; }
$DEBUG = $opt_d;
$PARALLEL = $opt_p;

#
# setup shell variables
# 
if ($opt_c eq "LINUX_64_gcc" && $opt_p eq "0") {
  $GCC_PATH = "/usr/lib64/ccache";
  $MYENV    = "/usr/local/bin:$GCC_PATH:/usr/bin:/usr/X11R6/bin";
}
else { 
  die "no suitable combination found"; 
}
$ENV{'PATH'} = $MYENV.":".$ENV{'PATH'}; 

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
# check out code, compile and run tests 
#
$pmpldir = "pmpl-$opt_c-$opt_r-$opt_d-$opt_p";
chdir $workdir or die "Can't cd to $workdir $!\n";  
$OUTPUT = "platform = $PLATFORM / ROBOT_DEF = $ROBOT / debug = $DEBUG / parallel = $PARALLEL\n";
$OUTPUT = $OUTPUT.`rm -rf $pmpldir 2>&1`;
$OUTPUT = $OUTPUT.`svn --quiet checkout svn+ssh://parasol-svn.cs.tamu.edu/research/parasol-svn/svnrepository/pmpl/trunk $pmpldir 2>&1`;
chdir "$workdir/$pmpldir/src";
$ENV{'PWD'} = "$workdir/$pmpldir/src";
$OUTPUT = $OUTPUT."Started at ".`date 2>&1`;
$OUTPUT = $OUTPUT."g++ path: ".`which g++ 2>&1`;
$OUTPUT = $OUTPUT."g++ details:\n".`g++ -c -v 2>&1`;
$OUTPUT = $OUTPUT.`gmake platform=$PLATFORM ROBOT_DEF=$ROBOT debug=$DEBUG parallel=$PARALLEL reallyreallyclean 2>&1`;
$OUTPUT = $OUTPUT.`gmake platform=$PLATFORM ROBOT_DEF=$ROBOT debug=$DEBUG parallel=$PARALLEL pmpl 2>&1`;
if (-e "$workdir/$pmpldir/src/pmpl") {
  $OUTPUT = $OUTPUT."=====\nPassed: pmpl compilation\n=====\n";
} else {
  $OUTPUT = $OUTPUT."=====\nFailed: pmpl compilation\n=====\n";
}
#$OUTPUT = $OUTPUT.`gmake test STAPL_BUILD_PARALLELISM=3 platform=$PLATFORM stl=$STL 2>&1`;
$OUTPUT = $OUTPUT."Done at ".`date 2>&1`;

#
# output log to /tmp
#
$ENV{'PATH'} = '/usr/local/bin/:/usr/X11R6/bin/:'.$ENV{'PATH'};
$ENV{'DISPLAY'} = '';

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
# copy the output to the webserver
#
#`cat $outfile | grep -v '[Pp]'assed'' > /research/www/groups/rwergergroup/intranet/stapl_perf/stapl_nightly/stapl.nightly.$opt_c.out`;

#
# send mail
#
open(MAIL, "|/usr/lib/sendmail -t"); 
print MAIL "To: $ADMIN_MAILTO\n";
print MAIL "From: pmpl_nightly\@tamu.edu\n";
print MAIL "Subject: PMPL nightly - anitbes.cse.tamu.edu pmpl.$opt_c.$opt_r.debug$opt_d.parallel$opt_p\n";
print MAIL "Done, output written to $outfile\n";
close(MAIL);


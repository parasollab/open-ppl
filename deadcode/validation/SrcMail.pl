#!/usr/bin/perl

use List::Util qw(min max);

# stapl_nightly_mail.pl
#   This checks out a copy of the pmpl trunk and checks for compilation errors
#   and sends out an email
#
#   It tests the following robots: PMP_Rigid

#
# configuration
#
$outputdir = "/tmp/pmpl_nightly_logs";
$cron_machine = "zenigata.cse.tamu.edu";
$MAILTO = "obprm\@listserv.tamu.edu";

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
# error code and message
#
use constant {
  SUCCESS    => 0,
  WARNINGS   => 1,
  ERRORS     => 2,
  INCOMPLETE => 3
};
$errcode  = SUCCESS;
$message  = "\n";

#
# search each compilation output for possible failures
#
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.cfg.debug0.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=cfg, debug=0, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.cfg.debug1.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=cfg, debug=1, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.multi.debug0.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=multi, debug=0, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.multi.debug1.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=multi, debug=1, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.reach.debug0.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=reach, debug=0, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.reach.debug1.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=reach, debug=1, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.surface.debug0.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=surface, debug=0, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.surface.debug1.parallel0.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=surface, debug=1, parallel=0)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.cfg.debug0.parallel1.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=cfg, debug=0, parallel=1)"));
$errcode = max($errcode,
  &get_error_code("$outputdir/$fulldate/pmpl.LINUX_gcc.cfg.debug1.parallel1.out",
    "Configuration: gcc 4.8.2 (platform=LINUX_gcc, robot=cfg, debug=1, parallel=1)"));

#
# Send out e-mail
#
$subject = "Subject: PMPL nightly ";

if($errcode == SUCCESS) {
  $subject = $subject."passed\n";
}
elsif($errcode == WARNINGS){
  $subject = $subject."had warnings\n";
}
elsif($errcode == ERRORS){
  $subject = $subject."had errors\n";
}
elsif($errcode == INCOMPLETE){
  $subject = $subject."did not complete\n";
}

#for debugging:
#print "subject: $subject";
#print "message:\n$message";
#exit;

open(MAIL, "|/usr/lib/sendmail -t");
print MAIL "To: $MAILTO\n";
print MAIL "From: pmpl_testing\@tamu.edu\n";
print MAIL $subject;
print MAIL $message;
print MAIL "\nLog files are located at $cron_machine:$outputdir/$fulldate\n";
#print MAIL "\nVisit https://parasol-webdev.cse.tamu.edu/groups/rwergergroup/intranet/stapl_perf/index.php\n";
close(MAIL);
sleep 5;

#
# get_error_code($OUT, $config)
#    searches file $OUT for warnings and errors, line by line
#    returns highest error_code found
#    appends global variable $message with result using $config string and details of passed tests, errors, and warnings found
#
#    ERROR returned if "failed" or "error" found (case insensitive)
#      - tinyxmlerror ignored: name of a library file
#      - Makefile.in error ignored: hack until solid makefile cleand up or nightly scripts are working
#    WARNING returned if "warning" found (case insensitive)
#      - WARNING associated with svn checkout ignored
#    INCOMPLETE returned if $OUT does not exist
#    SUCCESS returned otherwise
#
sub get_error_code {
  my($OUT, $config);
  ($OUT, $config) = @_;

  my $error_code = SUCCESS;
  my $details = '';
  my $passed_tests = '';
  my $failed_tests = '';

  if (-e "$OUT") {
    open(LOG, $OUT);
    @log = <LOG>;
    close(LOG);
    foreach $i (@log) {
      if ($i=~/Error:/i) { #test errors
        if ($error_code<ERRORS) { $error_code = ERRORS; }
        $failed_tests = $failed_tests.$i;
      }
      elsif (
        ($i=~/failed/i) ||
        #($i=~/E210002/) ||         #svn errors
        (($i=~/error/i) &&          #compile errors
          !($i=~/tinyxmlerror/) &&  #excluding...
          !($i=~/-Werror/) &&
          !($i=~/Makefile\.in/))    #last Makefile.in is a hack until solid
                                    #makefile cleaned up or nightly scripts are
                                    #working
      ) {
        if ($error_code<ERRORS) { $error_code = ERRORS; }
        $details = $details.$i;
      }
      elsif (($i=~/warning/i) && !($i=~/This computer system and data herein are available only for/)) {
        if ($error_code<WARNINGS) { $error_code = WARNINGS; }
        $details = $details.$i;
      }
      elsif ($i=~/passed/i) {
        $passed_tests = $passed_tests.$i;
      }
    }
  }
  else {
    $error_code = INCOMPLETE;
  }

  if($error_code == SUCCESS) {
    $message = $message."################################################################################\n";
    $message = $message.$config." passed\n";
  }
  elsif($error_code == WARNINGS){
    $message = $message."################################################################################\n";
    $message = $message.$config." had warnings\n";
  }
  elsif($error_code == ERRORS){
    $message = $message."################################################################################\n";
    $message = $message.$config." had errors\n";
  }
  elsif($error_code == INCOMPLETE){
    $message = $message."################################################################################\n";
    $message = $message.$config." did not complete successfully tonight\n";
  }
  if (!($details eq '')) {
    $message = $message."\n===== Errors =====\n".$details."\n\n";
  }
  if (!($failed_tests eq '')) {
    $message = $message."\n===== Failed Tests =====\n".$failed_tests."\n\n";
  }
  $message = $message."################################################################################\n";
  $message = $message."\n\n";

  $error_code; #return value;
}

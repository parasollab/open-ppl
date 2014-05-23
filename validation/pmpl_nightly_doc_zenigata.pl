#!/usr/bin/perl

#
# configuration
#
$outputdir = "/tmp/pmpl_nightly_logs";
$ADMIN_MAILTO = "jdenny\@cs.tamu.edu";
$workdir  = "/scratch/zenigata/jdenny/pmpl_nightly";

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
$pmpldir = "pmpl-docs";
chdir $workdir or die "Can't cd to $workdir $!\n";
$OUTPUT = "Documentation\n";
$OUTPUT = $OUTPUT.`rm -rf $pmpldir 2>&1`;
$OUTPUT = $OUTPUT.`svn --quiet checkout svn+ssh://parasol-svn.cs.tamu.edu/research/parasol-svn/svnrepository/pmpl/trunk $pmpldir 2>&1`;
$OUTPUT = $OUTPUT."Started at ".`date 2>&1`;
chdir "$workdir/$pmpldir/docs";
#standards doc
chdir "StandardsAndProcedures";
$OUTPUT = $OUTPUT.`make 2>&1`;
$OUTPUT = $OUTPUT.`cp standards.pdf /research/www/groups/amatogroup/intranet/PMPLDocs`;
$OUTPUT = $OUTPUT.`chmod 664 /research/www/groups/amatogroup/intranet/PMPLDocs/standards.pdf`;
#doxygen
chdir "../Doxygen";
$OUTPUT = $OUTPUT.`make 2>&1`;
$OUTPUT = $OUTPUT.`cp -r Internal Release /research/www/groups/amatogroup/intranet/PMPLDocs`;
$OUTPUT = $OUTPUT.`chmod -R 775 /research/www/groups/amatogroup/intranet/PMPLDocs/Internal`;
$OUTPUT = $OUTPUT.`chmod -R 775 /research/www/groups/amatogroup/intranet/PMPLDocs/Release`;
$OUTPUT = $OUTPUT."Done at ".`date 2>&1`;

#
# output log to /tmp
#
$ENV{'DISPLAY'} = '';

if (!-e "$outputdir") {
  `mkdir $outputdir`;
}
if (!-e "$outputdir/$fulldate") {
  `mkdir $outputdir/$fulldate`;
}
$outfile = "$outputdir/$fulldate/pmpl.docs.out";
open(OUT, ">$outfile" || die "error, could not open $outfile for reading: $!");
print OUT $OUTPUT;
close(OUT);

#
# send mail
#
open(MAIL, "|/usr/lib/sendmail -t");
print MAIL "To: $ADMIN_MAILTO\n";
print MAIL "From: pmpl_nightly\@tamu.edu\n";
print MAIL "Subject: PMPL nightly - zenigata.cse.tamu.edu pmpl.docs\n";
print MAIL "Done, output written to $outfile\n";
close(MAIL);


#!/usr/bin/perl

$system = pop @ARGV;
$workdir = "/scratch/roma/stapl_nightly_timing/".$system."_dir/new";
chdir $workdir or die "Cannot cd to $workdir\n";
$ENV{'PWD'} = $workdir;
@vals = `ls -1 *.done 2> /dev/null `;
chomp @vals;
$size = @vals;
$OUTPUT = "";

if ($size > 0) {
  foreach (@vals) {
    s/\.done//;
    $OUTPUT .= "Processing $_\n";
    $OUTPUT .= `tar xfz $_.tar.gz 2>&1`;
    $OUTPUT .= `mv $_.tar.gz archive 2>&1`;
    $OUTPUT .= `../../new_bin/sanitize_timing_data.pl $_/*out 2>&1`;
    $OUTPUT .= `../../new_bin/extract_algorithm_times.pl $_ $system 2>&1`;
    $OUTPUT .= `rm -rf $_ $_.done 2>&1`;
    $OUTPUT .= "Finished $_\n\n";
  }

#  open  (MAIL, "|/usr/lib/sendmail -t");
#  print MAIL "To: timmie\@cs.tamu.edu\n";
#  print MAIL "From: stapl_timing\@tamu.edu\n";
#  print MAIL "Subject: STAPL nightly timing insertion\n";
  print "\n".$OUTPUT."\n";
#  close (MAIL);
}

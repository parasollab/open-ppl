#!/usr/bin/env perl

# Check uncommitted code for deviations from docs/coding_conventions
# If no directory is provided the current directory is checked.

# Identify the version control system as either SVN or git
sub version_control {
  system("svn info > /dev/null 2>&1");
  return "svn" if $? == 0;

  system("git status > /dev/null 2>&1");
  return "git" if $? == 0;

  return "none";
}

# Compute list of dirty files
sub dirty_files {
  return version_control() =~ /svn/ ?
    `svn -q st` :
    `git diff --name-status master`;
}

$wdir = $ARGV[0];
if ($wdir eq "") {
  $wdir = $ENV{'PWD'};
}

chdir $wdir or die "Can't cd to $wdir $!\n";
$ENV{'PWD'} = "$ENV{'PWD'}" . "/$wdir/";

print "Checking new code in $wdir for formatting issues.\n\n";
@files = dirty_files();


while ($file = pop @files) {

  # Remove the new line and svn status indicator from the file name.
  chomp $file;
  $file =~ s/^.\s+//;
  if (!((-B $file) || ($file =~ /akefile/) || ($file =~ /check_formatting.pl/)
        || ($file =~ /diff$/) || ($file =~ /patch$/)))
  {
    print "checking $wdir/$file\n";

    # Flags used to avoid reporting the same issue multiple times in a file.
    $trailing_space = 0;
    $tabs = 0;
    $leading_underscore = 0;
    $keyword_space = 0;
    $line_length = 0;
    $const_t = 0;

    open(CODE,"<$file");
    my $i = 0;
    while ($line = <CODE>) {
      $i++;
      # Rule 1
      if (($tabs == 0) && ($line =~ /\t/) && (!($file =~ /akefile/i))) {
        print "  - lines with tabs\n";
        print "  first on line $i: $line";
        $tabs = 1;
      }

      # Rule 4
      if (($keyword_space == 0) &&
          (($line =~ / if\(/) || ($line =~ / for\(/) ||
           ($line =~ / while\(/))) {
        print "  - space needed between keyword and opening paren\n";
        print "  first on line $i: $line";
        $keyword_space = 1;
      }

      # Rule 5
      if (($trailing_space == 0) && ($line =~ /  *$/)) {
        print "  - lines with trailing spaces\n";
        print "  first on line $i: $line";
        $trailing_space = 1;
      }

      # Rule 6
      if (($line_length == 0) && ($line =~ /.{81}/) && (!($line =~ /copydoc/)))
      {
        print "  - lines over 80 characters long.\n";
        print "  first on line $i: $line";
        $line_length = 1;
      }

      # Rule 9
      if (($leading_underscore == 0) &&
          (!($line =~ /def +_STAPL$/)) &&
          (!($line =~ /__[a-z_]+/)) &&
          (!($line =~ /__VA_ARGS+/)) &&
         (($line =~ / _\D/) || ($line =~ /,_\D/))) {
        print "  - variables/functions with leading underscores\n";
        print "  first on line $i: $line";
        $leading_underscore = 1;
      }

      #Variables rule 5 -- checking for const T&
      if (($const_t == 0) &&
          ($line =~ /const\s[a-z_]+\s?&/) &&
          (!($line =~ /const volatile\s?&/)))
      {
        print "  - const T& type specification\n";
        print "  first on line $i: $line";
        $const_t = 1;
      }
    }
    close(CODE);
  }
}
print "\nFinished checking $wdir.\n";

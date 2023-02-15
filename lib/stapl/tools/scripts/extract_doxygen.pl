#!/usr/bin/perl
# breadth-first traversal of directory structure
# find lines with C++ style comments and 
# if they are part of doxygen comments marked with specified keyword
# then print them
# default keyword is "todo".  "bug" and "brief" are also possibilities

$start = $ARGV[0];
@dirlist = ( $start );
$root= $ENV{'PWD'};
%vocab = ();

$keyw = $ARGV[1];
if( $keyw eq "" ) {
  $keyw = "todo";
}
$keyw = "\@" . $keyw;

# process the directories
while ($dir = shift @dirlist) {
  chomp $dir;

  if( -d $root . "//" . $dir ) {
    print "\n@@@@@ ",$root,"//",$dir," @@@@@\n";

    chdir $root . "/" . $dir;
    @list = `ls`;

    # separate directories, source files, and others
    @filelist = ();
    foreach $item (@list) {
      chomp $item;
      if( -d $item ) {
        $path = $dir . "/" . $item;
        push @dirlist, $path;
      } elsif ( $item =~ "cc\$" || $item =~ "hpp\$" || $item =~ "h\$" ) {
        push @filelist,$item;
      }
    }

    # process the source files
    foreach $file (@filelist) {

      print "\n===== ",$dir,"/",$file," =====\n";

      $show = 0;
      open(CODE,"<$file");
      while ($line = <CODE>) {
        chomp $line;

        if( $line =~ "^ *///" ) {
 
          if( $show == 0 ) {
             if( $line =~ ("^ *///  *(" . $keyw . ").*") ) {
              $show = 1;
              print $line,"\n";
            } else {
            }
          } else {
            if( $line =~ "^ */*\$" ) {
              $show = 0;
            } else {
              print $line,"\n";
            }
          }
        }
      }
      close(CODE);

    }
  } else {
    print "error: non-directory on directory list: ",$root . "//" . $dir,"\n";
  }
}

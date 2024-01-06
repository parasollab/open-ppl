#!/usr/bin/perl
# breadth-first traversal of directory structure
# extract lines with C++ style comments and display them
# extract words from the line and put them in a vocabulary list
# print the vocabulary after processing all files
# NOTE: does not work for C style comments

$start = $ARGV[0];
@dirlist = ( $start );
$root= $ENV{'PWD'};
%vocab = ();
$ord_a = ord('a');
$ord_z = ord('z');

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

      open(CODE,"<$file");
      while ($line = <CODE>) {
        chomp $line;

        if( $line =~ "^ *//" ) {
 
          # process the line
          print $line,"\n";
          @wordlist = split(' ', $line);
          foreach $word (@wordlist) {
  
            # screen out 'words' that don't have any letters
            $text = lc($word);
            $count = 0;
            @glyphlist = split('', $text);
            foreach $glyph (@glyphlist) {
              if( ord($glyph) >= $ord_a && ord($glyph) <= $ord_z ) {
                $count++;
              }
            }
            if( $count > 0 ) {

              # screen out program symbols
              if( -1 == index($text,"::") && -1 == index($text,"->") ) {

                # remove punctuation 
                $pos = index($text,".");
                if( -1 != $pos ) {
                  substr($text,$pos) = " ";
                }
                $pos = index($text,":");
                if( -1 != $pos ) {
                  substr($text,$pos) = " ";
                }
                $pos = index($text,",");
                if( -1 != $pos ) {
                  substr($text,$pos) = " ";
                }
                $pos = index($text,";");
                if( -1 != $pos ) {
                  substr($text,$pos) = " ";
                }

                $temp = $vocab{$text};      
                $vocab{$text} = $temp + 1;
              }
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

print "\nVOCABULARY\n";
@dictionary = ();
while(($key,$value) = each %vocab ) {
   push @dictionary, ($key);
}
@wordlist = sort @dictionary;
foreach $word (@wordlist) {
  print $word,"\n";
}


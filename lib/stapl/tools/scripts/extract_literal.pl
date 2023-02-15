#!/usr/bin/perl
# breadth-first traversal of directory structure
# extract lines with literals, except #include and comments
# extract words from the literals and put them in a vocabulary list
# print the vocabulary after processing all files

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
        #$path = $item;
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

        # pick lines with literals, except #include and comments
        if( $line =~ "\"" ) {
          if( $line =~ "^#include" ) {
            #print $line;
          } elsif( $line =~ "^ *//" ) {
            #print $line;
          } else {
            # count double quotes embedded in strings
            $count = 0;
            $pos = -1;
            while(( $pos = index($line, "\"", $pos)) > -1) {
              $count++;
              $pos++;
            }
 
            # safe to extract the string?
            if( 0 == ($count % 2) ) {
              $pos = -1;
              while(( $pos = index($line, "\"", $pos)) > -1) {
                $first = $pos;
                $last = index($line, "\"", $first+1);
                $len = ($last - $first) - 1;
                $str = substr($line,$first+1,$len);
                $pos = $last + 1;
 
                # process the string
                print $str,"\n";
                @wordlist = split(' ', $str);
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
                    $temp = $vocab{$text};      
                    $vocab{$text} = $temp + 1;
                  }
                }
              }
            } else {
              print $line, "\n";
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


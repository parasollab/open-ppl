#!/usr/bin/perl
# breadth-first traversal of directory structure
# extract lines with stapl_assert and 
#         lines following that complete the statement

$start = $ARGV[0];
@dirlist = ( $start );
$root= $ENV{'PWD'};

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
      $paren = 0;
      while ($line = <CODE>) {
        chomp $line;

        # ignore comments and #preprocessor directives
        if( $line =~ "^ *//" ) {
          next;
        }
        if( $line =~ "^#include" ) {
          next;
        }

        # pick lines with 'stapl_assert' or completing that statement
        if( $line =~ "stapl_assert" || $paren > 0 ) {
          print $line,"\n";

          # count double quotes embedded in strings
          $quote_count = 0;
          $pos = -1;
          while(( $pos = index($line, "\"", $pos)) > -1) {
            $quote_count++;
            $pos++;
          }
 
          # safe to extract the non-strings?
          if( 0 == ($quote_count % 2) ) {

            # examine the non-literals in the line
            $quote = index($line, "\"");
            if( -1 == $quote ) {
              # easy case - no literals

              # count the parens so we can grab following lines as needed
              $lpar = $rpar = 0;
              @glyphlist = split('', $line);
              foreach $glyph (@glyphlist) {
                if( $glyph eq "(" ) {
                  $lpar++;
                } elsif( $glyph eq ")" ) {
                  $rpar++;
                }
              }
              $paren += $lpar - $rpar;

            } else {
              # line has literals, process the other parts

              $pos = 0;
              while( $pos < length($line) ) {

                # code before literal
                $first = $pos;
                $quote = index($line, "\"", $pos);
                if( $quote == -1 ) {
                  $pos = length($line);
                  $last = $pos - 1;
                } else {
                  $pos = $quote + 1;
                  $last = $quote - 1;
                }
                $len = ($last - $first) + 1;
                $text = substr($line,$first,$len);

                # count the parens so we can grab following lines as needed
                $lpar = $rpar = 0;
                @glyphlist = split('', $text);
                foreach $glyph (@glyphlist) {
                  if( $glyph eq "(" ) {
                    $lpar++;
                  } elsif( $glyph eq ")" ) {
                    $rpar++;
                  }
                }
                $paren += $lpar - $rpar;

                # skip over literal if present
                $first = $pos - 1;
                $quote = index($line, "\"", $pos);
                if( $quote != -1 ) {
                  $pos = $quote;
                  $last = $quote;
                  $len = ($last - $first) + 1;
                  $text = substr($line,$first,$len);
                }
              }
            } 
          } else {
              print "nested quotes prevent analysis of this line\n";
              print $line,"\n";
          }
        }
      }
      close(CODE);
    }
  } else {
    print "error: non-directory on directory list: ",$root . "//" . $dir,"\n";
  }
}

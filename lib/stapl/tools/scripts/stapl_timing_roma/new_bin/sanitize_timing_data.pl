#!/usr/bin/perl

use File::Copy;

$tmp_file = "strip_tmp.txt";
while ($file = pop @ARGV)
{
  open(FILE,"<./$file");
  open(TMP,">./$tmp_file");
  while ($line = <FILE>)
  {
    # line contains a test name or timing information
    if (($line =~ /test_/) || ($line =~ /time:/))
    {
      # These tests don't print timing information
      if (!(($line =~ /p_power:/) ||
            ($line =~ /p_swap:/) ||
            ($line =~ /p_iter_swap:/)))
      {
        # save the line
        print TMP $line;
      }
    }
  }
  close(TMP);
  close(FILE);
  move($tmp_file, $file);
}

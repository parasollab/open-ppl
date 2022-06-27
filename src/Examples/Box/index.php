<?php include("header_amatogroup.incl"); ?>

   <TITLE>Folding Box |MP Benchmarks | Parasol Laboratory</TITLE>

<!---------------------------------------------------------------------->
<!----------------------------- Title ---------------------------------->
<!---------------------------------------------------------------------->
            <HR WIDTH="90%" noshade>
            <CENTER>
            <FONT class="pagetitle">
            Box Folding Puzzle
            </FONT>
            </CENTER>
            <HR WIDTH="90%" noshade>
<!---------------------------------------------------------------------->
<!---------------------------- Page Info ------------------------------->
<!---------------------------------------------------------------------->
<P>

<FONT class="largertext">
<P>
Box Folding Model
<BR>
<i>Created by Guang Song, Parasol MP Group, CS Dept, Texas A&M University</i>
</FONT>

<P>
The <i>box folding problem</i> is a motion planning problem
where the objective is to fold the articulated model into
its final box shape.
There are no external obstacles in the enviroment, but self-collision
among the links (connected by revolute joints) must be avoided.

<P>
The model consists of 11 parts, each of which is a rectangular piece.
One is considered the fixed base, and 10 are considered links.
However, taking advantage of symmetry conditions, we can reduce
the number of parts by half (1 fixed base and 5 links).
The model is available in <A HREF="../byu.php"> BYU </A> format,
with each piece (base and links) represented in a separate file.
The linkage information is contained in an environment file,
which is in our <A HREF="../env.php"> env </A> format.
The environment file gives the pieces in the start configuration
shown below.
<BR>

<TABLE BORDER=0 CELLSPACING=5 CELLPADDING=5 >
  <TR>
  <TD ALIGN=CENTER> <IMG SRC="box0.jpg" WIDTH="100">
       <BR> <FONT class="bodytext"> Start </FONT> </TD>
  <TD> <IMG SRC="box3.jpg" WIDTH="100"> </TD>
  <TD> <IMG SRC="box4.jpg" WIDTH="100"> </TD>
  <TD ALIGN=CENTER> <IMG SRC="box5.jpg" WIDTH="100">
       <BR> <FONT class="bodytext"> Goal </FONT></TD>
  <TD> <FONT class="bodytext">
       <A HREF="box.tar.gz">Tar File</A> (BYU and env files)
       <BR>
       (<A HREF="../byu.php"> BYU format </A> &nbsp;
       <A HREF="../env.php"> env format </A>)</FONT></TD>
  </TR>
</TABLE>

<P>
We solved this problem using our OBPRM planner.
More details can be found
<A HREF="../../../research/folding/">
here </A>.
Some snapshots of the solution path are shown above.
For a better look, you can view a
<A HREF="../../../research/folding/movies/boxfolding.avi"> movie (avi)</A>
</P>

<?php include("footer.incl"); ?>

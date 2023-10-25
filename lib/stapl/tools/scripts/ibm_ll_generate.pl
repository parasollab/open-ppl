#!/usr/bin/perl

use Env;
use Getopt::Long;
use File::Basename;
use POSIX;

my $exec  = $ARGV[0] or die "Executable not defined";
my @args  = ();
my $np    = 2;                      # total number of PEs
my $nth;                            # number of threads per process
my $ppn   = 1;                      # number of processes per node
my $mixed;                          # number of levels for mixed-mode
my $mem   = 1024;                   # requested memory
my $time  = "00:30:00";             # job time
my $queue = "cs_group";             # queue
my $user  = ${'USER'};              # username
my $email = $user."\@cse.tamu.edu"; # email
GetOptions( "a=s"     => \@args,
            "np=i"    => \$np,
            "nth=i"   => \$nth,
            "ppn=i"   => \$ppn,
            "mem=i"   => \$mem,
            "mixed=i" => \$mixed,
            "time=s"  => \$time,
            "queue=s" => \$queue,
            "user=s"  => \$user,
            "email=s" => \$email ) or die "Incorrect argument";

# create job name
my $name     = basename($exec);
my $job_name = $name.'_'.$np.'p';
if ($nth ne '') {
  $job_name = $job_name.'_'.$nth.'t';
}

# create job width information
my $procs = $np; # number of MPI processes
if ($nth ne '') {
  $procs = floor($np/$nth);
  if ( ($nth * $procs)>$np ) {
    die "Required resources and requested PEs do not match: $np PEs != $nth threads x $procs MPI processes";
  }
}
$nodes = ceil($procs/$ppn); # number of nodes
$mem_node = ceil($mem/$ppn).'mb';

# print job information to file
if ($nth ne '') {
  print "Generating job file:  $np PEs, $nth threads/MPI process, $procs MPI processes, $nodes nodes, $ppn MPI processes/node\n";
}
else {
  print "Generating job file:  $np PEs, $procs MPI processes, $nodes nodes, $ppn MPI processes/node\n";
}
$job_file = 'job_'.$job_name.'.ll';
open $jf, ">", $job_file or die "$job_file could not be opened for output";
print $jf "#@ shell            = /bin/ksh\n";
if ($nth ne '') {
  print $jf "#@ comment          = $procs MPI processes, $nth threads\n";
}
else {
  print $jf "#@ comment          = $procs MPI processes\n";
}
print $jf "#@ job_name         = $job_name\n";
print $jf "#@ class            = $queue\n";
print $jf "#@ job_type         = parallel\n";
if ($nth ne '') {
  print $jf "#@ environment      = COPY_ALL; MP_SINGLE_THREAD=no; MP_PROCS=$procs; OMP_NUM_THREADS=$nth\n";
  print $jf "#@ resources        = ConsumableCpus($nth) ConsumableMemory($mem_node)\n";
}
else {
  print $jf "#@ environment      = COPY_ALL; MP_SINGLE_THREAD=yes; MP_PROCS=$procs\n";
  print $jf "#@ resources        = ConsumableCpus(1) ConsumableMemory($mem_node)\n";
}
print $jf "#@ node             = $nodes\n";
print $jf "#@ tasks_per_node   = $ppn\n";
print $jf "#@ network.MPI_LAPI = sn_single, shared, US\n";
print $jf "#@ output           = \$(job_name).out\n";
print $jf "#@ error            = \$(job_name).err\n";
print $jf "#@ wall_clock_limit = $time\n";
print $jf "#@ notification     = error\n";
print $jf "#@ notify_user      = $email\n";
print $jf "#@ queue\n";
print $jf "#\n";
print $jf "export OBJECT_MODE=64\n";
if ($nth ne '') {
  print $jf "export OMP_NUM_THREADS=$nth\n";
  print $jf "export OMP_DYNAMIC=FALSE\n";
  print $jf "export MALLOCMULTIHEAP=heaps:$nth\n";
  print $jf "export AIXTHREAD_COND_DEBUG=OFF\n";
  print $jf "export AIXTHREAD_MUTEX_FAST=ON\n";
  print $jf "export AIXTHREAD_SCOPE=S\n";
  if ($mixed ne '') {
    print $jf "export STAPL_MAIN_LEVELS=$mixed\n";
  }
}
print $jf "#\n";
print $jf "poe ./$exec @args\n";
print $jf "#\n";
#print $jf "/usr/local/bin/jobinfo\n";
close ($jf);

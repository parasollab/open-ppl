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
my $time  = "00:10:00";             # job time
my $queue = "regular";              # queue
my $user  = ${'USER'};              # username
my $email = $user."\@cse.tamu.edu"; # email

GetOptions( "a=s"     => \@args,
            "np=i"    => \$np,
            "nth=i"   => \$nth,
            "ppn=i"   => \$ppn,
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
$nodes = ceil($procs/$ppn);

# print job information to file
if ($nth ne '') {
  print "Generating job file:  $np PEs, $nth threads/MPI process, $procs MPI processes, $nodes nodes, $ppn MPI processes/node\n";
}
else {
  print "Generating job file:  $np PEs, $procs MPI processes, $nodes nodes, $ppn MPI processes/node\n";
}
$job_file = 'job_'.$job_name.'.pbs';
open $jf, ">", $job_file or die "$job_file could not be opened for output";
print $jf "#PBS -q $queue\n";
print $jf "#PBS -l mppwidth=$np\n";    # number of cores
print $jf "#PBS -l mppnppn=$ppn\n";    # number of MPI_tasks_per_node
if ($nth ne '') {
  print $jf "#PBS -l mppdepth=$nth\n"; # number of cores per MPI task
}
print $jf "#PBS -m abe\n";
print $jf "#PBS -l walltime=$time\n";
print $jf "#PBS -o $job_name.out\n";
print $jf "#PBS -e $job_name.err\n";
print $jf "#PBS -N $name\n";
print $jf "#PBS -M $email\n\n";
print $jf "cd \$PBS_O_WORKDIR\n";
if ($nth ne '') {
  print $jf "export OMP_NUM_THREADS $nth\n";
  print $jf "aprun -n $procs -d $nth -N $ppn ./$exec @args\n";
}
else {
  print $jf "aprun -n $procs -N $ppn ./$exec @args \n";
}
close($jf);

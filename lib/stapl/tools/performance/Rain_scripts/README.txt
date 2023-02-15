Cron Jobs from Cali (My computer)
10 00 * * * ssh rain "/mnt/lustre/lus0/STAPL/performance_monitoring/STAPL_Performance_Master_Script.pl"
00 12 * * * ~/bin/STAPL_Performance_Verifier.pl > ~/bin/verifier.txt
30 12 * * * mail -s "STAPL Performance Monitoring Report" hordemann@tamu.edu < ~/bin/verifier.txt
#30 12 * * * mail -s "STAPL Performance Monitoring Disabled" hordemann@tamu.edu < ~/bin/sorry.txt

The setup is pretty simple:
A computer calls STAPL_Performance_Master_Script.pl as part of a nightly cron job. This currently takes a long time to run (it finishes mid afternoon). This perl script (ugh, Perl)has some rain-specific customizations, since at this time we only run it on Rain.

STAPL_Performance_Master_Script.config contains configuration information that the master script needs. The original commit has comments in it explaining the layout of the config file.

STAPL_Performance_Master_Script.log is rewritten every night and the contents detail the execution of the previous run. It is mainly intended for debugging and troubleshooting.

In the folders where test code is run, knownfails is used to list the files that should not be run because they are already known to fail. PENDING_JOBLIST is created during every run to keep track of which jobs have not reached completion. All these examples in the initial commit are from test/rel_alpha.

The master script also archives the results of the runs in a subfolder off the target directory named "archive."

test_driver.pl is called by the master script and also resides in the target test directory. It runs the actual test code on non-batch systems. On Rain, this code is not used and the master script generates and schedules batch jobs.

process_execution_times.pl takes the results of completed executions and processes their execution times. NOTE: This may need to be modified for different programs what have different output. Database connection information is in this file.

Another cronjob calls STAPL_Performance_Verifier.pl examines the data in the database to detect when it is running long (or short) on a particular test. It outputs a file called "verifier.txt." which another cronjob mails to someone who is paying attention.
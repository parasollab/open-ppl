#!/usr/bin/perl
#nthomas 12/04/2004
#db_funcs.pl - utility functions for interacting with stapl_perf db

#use lib qw(/g/g11/timmie/mcr/usr/lib/perl5/5.00503/
#           /g/g11/timmie/mcr/usr/lib/perl5/site_perl/5.005/
#           /g/g11/timmie/mcr/usr/lib/perl5/site_perl/5.8.0/);

use DBI;
#use Proc::Spawn;

sub init_db {
#   ($ssh_pid, $ssh_in_fh, $ssh_out_fh, $ssh_err_fh) =
#      spawn("ssh -L 3001:localhost:3001 roma.cs.tamu.edu /users/timmie/bin/dbproxy.pl");
#
#   sleep 7;  #yes I know this is lame

#   my $config = {
#     dsn_at_proxy => "dbi:mysql:stapl_perf:parasol-db.cs.tamu.edu",
#     proxy => "hostname=localhost;port=3001",
#   };

#   my $dsn = sprintf "dbi:Proxy:%s;dsn=%s",
#     $config->{proxy},$config->{dsn_at_proxy};
   my $dsn = 'DBI:mysql:stapl_new_perf:parasol-db.cs.tamu.edu';

   my $db_user_name = 'nthomas';
   my $db_password = 'howdy';
   $dbh =  DBI->connect($dsn, $db_user_name, $db_password) 
      || die "connect did not work: $DBI::errstr";
}


sub close_db { 
  $dbh->disconnect(); 
#  kill 15, $ssh_pid;
}


sub get_appid($) {   #arg1 = appname
   if (!defined $dbh) { die "get_appid needs dbh defined"; }
   my $sth = $dbh->prepare(qq{ SELECT app_id, name FROM applications });
   $sth->execute();
   while (my ($id, $name) = $sth->fetchrow_array()) { $h_apps{$name} = $id; }
   $sth->finish();
   if (!defined $h_apps{$_[0]}) { die "get_appid: app $_[0] not found"; }
   return $h_apps{$_[0]};
}


sub get_userid($) {   #arg1 = username
   if (!defined $dbh) { die "get_userid needs dbh defined"; }
   my $sth = $dbh->prepare(qq{ SELECT user_id, csuser FROM users });
   $sth->execute();
   while (my ($id, $name) = $sth->fetchrow_array()) { $h_users{$name} = $id; }
   $sth->finish();
   if (!defined $h_users{$_[0]}) { die "get_userid: user not found"; }
   return $h_users{$_[0]};
}


sub get_machineid($) {   #arg1 = machinename
   if (!defined $dbh) { die "get_machineid needs dbh defined"; }
   my $sth = $dbh->prepare(qq{ SELECT machine_id, name FROM machines });
   $sth->execute();
   while (my ($id, $name) = $sth->fetchrow_array()) { 
      $h_machines{$name} = $id; 
   }
   $sth->finish();
   if (!defined $h_machines{$_[0]}) { die "get_machineid: machine not found"; }
   return $h_machines{$_[0]};
}



sub get_statid($) {   #arg1 = statname, arg2 = app_name(disabled)
   #FIXME doesn't verify app/stat associations -- do we want this?
   #my $aid = get_appid($_[1]);   
   if (!defined $dbh) { die "get_statid needs dbh defined"; }
   my $sth = $dbh->prepare(qq{ SELECT statistics_info.stat_id
            FROM statistics_info
            WHERE statistics_info.name='$_[0]'});
   $sth->execute(); ($stat_id) = $sth->fetchrow_array(); $sth->finish();
   if (!defined $stat_id) { return -1; }
   return $stat_id;
}


sub get_datatype($$) {   #arg1 = statname, arg2 = app_name
   my $sid = get_statid($_[0]);
   my $sth = $dbh->prepare(qq{ SELECT statistics_info.interpretation
                        FROM statistics_info 
                        WHERE statistics_info.stat_id = '$sid'});
   $sth->execute(); ($interpret) = $sth->fetchrow_array(); $sth->finish();
   if (!defined $interpret) { die "get_datatype: unexpected error..."; }
   if ($interpret eq "STR") {  return "VARCHAR(128)"; }
   if ($interpret eq "INT") {  return "INT UNSIGNED"; }
   if ($interpret eq "REAL") {  return "DOUBLE"; }
   die "get_datatype: don't know how to interpret";   
}


sub is_tuneable($$) {   #arg1 = statname, arg2 = app_name
   my $sid = get_statid($_[0]);
   my $sth = $dbh->prepare(qq{ SELECT statistics_info.fixed
                        FROM statistics_info
                        WHERE statistics_info.stat_id = '$sid'});
   $sth->execute(); ($val) = $sth->fetchrow_array(); $sth->finish();
   return 1 if ($val eq "0");
   return 0 if ($val eq "1");
   die "is_tuneable: fixed should be either 0 or 1";
}

sub new_statistic($$$$) { #app_name, stat_name, description, interpretation
   #FIXME, add check that description and interpreation match if already exists   

   if (!defined $dbh) { die "new_statistic needs dbh defined"; }

   my $app_id = get_appid($_[0]);
 
   my $stat_id;
   #does this stat_name already exist?
   my $sth = $dbh->prepare(qq{ SELECT statistics_info.stat_id
          FROM statistics_info WHERE statistics_info.name='$_[1]' });
   $sth->execute(); ($stat_id) = $sth->fetchrow_array(); $sth->finish();


   if (!defined $stat_id) {
     #add entry in statistics table
     $dbh->do("INSERT INTO `statistics_info`
         (`stat_id`, `name`, `fixed`, `description`, `interpretation`)
         VALUES ('', '$_[1]', '', '$_[2]', '$_[3]')");
      my $sth = $dbh->prepare(qq{ SELECT LAST_INSERT_ID() });
      $sth->execute(); ($stat_id) = $sth->fetchrow_array(); $sth->finish();
   
   }
   
   #stat_id was defined, check for association with app
   if (get_statid($_[1]) == -1) {   
      $dbh->do("INSERT INTO `statistics_associations`
         (`app_id`, `stat_id`) VALUES ('$app_id', '$stat_id')");
   } else {
      print "$_[0] already has statistics named '$_[1]' defined / associated\n";
   }
   return $stat_id;
}


#arg1 = app_name, arg2 = machine_name, arg3 = username, arg4 = timestamp, arg5 = description
sub insert_run($$$$$) {  
   my $app_id = get_appid($_[0]);
   my $machine_id = get_machineid($_[1]);
   my $user_id = get_userid($_[2]);
   my $timestamp = $_[3];
   my $description = $_[4];
 
   $dbh->do("
      INSERT INTO `runs`
      (`run_id`, `machine_id`, `application_id`,`user_id`, `description`, `timestamp`)
       VALUES ('', '$machine_id', '$app_id', '$user_id', '$description', '$timestamp')
   ");

   my $sth = $dbh->prepare(qq{ SELECT LAST_INSERT_ID() });
   $sth->execute(); ($run_id) = $sth->fetchrow_array(); $sth->finish();
   return $run_id;
}


sub insert_statistic($$$$) { #run_id, stat_name, index, value 
   my $run_id = $_[0];
   my $stat_id = get_statid($_[1]);
   my $index = $_[2];
   my $value = $_[3];
   $dbh->do("INSERT INTO `statistics` (`run_id`, `stat_id`, `index`,`value`)
            VALUES ('$run_id', '$stat_id', '$index', '$value')"); 
}

sub increment_timestamp($) { #ts in yyyyhhmmss format
  my $ts = $_[0];

  my $sec = $ts % 100;
  $ts -= $sec;

  my $min = $ts % 10000;
  $ts -= $min;

  my $hour = $ts % 1000000;
  $ts -= $hour;

  my $date = $ts;

  if ($sec < 59) {
    $sec++;
  } elsif ($min < 5900) {
    $min += 100;
    $sec = 0;
  } elsif ($hour < 230000) {
    $hour += 10000;
    $min = 0;
    $sec = 0;
  } else {
    die "Cannot add another timestamp and remain in the same day $ts\n";
  }

  $ts = $date + $hour + $min + $sec;
  return $ts;
}

1;

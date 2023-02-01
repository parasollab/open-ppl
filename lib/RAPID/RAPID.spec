#
# RAPID.spec
#
# This is designed to build RAPID from the current
# Parasol SVN.  To create the source tarball, do:
#
# svn co svn+ssh://parasol-svn.cs.tamu.edu/research/parasol-svn/svnrepository/utils/geom/CD-libs/RAPID
# rm -rf `find RAPID -name .svn`
# mv RAPID RAPID-0-`date +%Y.%m.%d`
# tar cvzf RAPID-0-`date +%Y.%m.%d`.tar.gz RAPID-0-`date +%Y.%m.%d`
#
%define date 2007.10.11
Name: RAPID
Summary: RAPID - Robust and Accurate Polygon Interference Detection library
Version: 0 
Release: %{date}%{dist}
License: Copyright 1995 The University of North Carolina at Chapel Hill.  All Rights Reserved.
Group: Development/Libraries
Source: RAPID-%{version}-%{date}.tar.gz
URL: http://www.cs.unc.edu/~geom/OBB/OBBT.html
Packager: Jack Perdue <j-perdue@tamu.edu>, Parasol Laboratory, Texas A&M University -- http://parasol.tamu.edu/
BuildRoot: %{_tmppath}/%{name}-%{version}-%{date}-buildroot

%description
RAPID is a robust and accurate polygon interference 
detection  library for large environments composed 
of unstructured models.

%package devel
Summary: RAPID header files and development libraries
Group: Development/Libraries
Requires: %{name} = %{version}-%{release}
%description devel
Header files and development libraries for RAPID.

%prep
%setup -n %{name}-%{version}-%{date}
head -41 RAPID.H > LICENSE
cat >> Makefile << EEOOFF
lib%{name}.so.%{version}.%{date}: \$(OBJECTS)
	\${CC} -shared \${CFLAGS} \$^ -Wl,-soname=lib%{name}.so.%{version} -o lib%{name}.so.%{version}.%{date}
EEOOFF

%build
make
make lib%{name}.so.%{version}.%{date}
cp lib%{name}.so.%{version}.%{date} lib%{name}.so

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/usr/include/%{name}
cp *.H %{buildroot}/usr/include/%{name}
mkdir -p %{buildroot}/usr/lib
cp libRAPID.a libRAPID.so* %{buildroot}/usr/lib

%clean
rm -rf %{buildroot}

%files
/usr/lib/libRAPID.so.%{version}.%{date}
%doc LICENSE

%files devel
/usr/include/RAPID
/usr/lib/libRAPID.so
/usr/lib/libRAPID.a
%doc doc/user_manual.txt README sample_client.C

%post
/sbin/ldconfig

%postun
/sbin/ldconfig

%changelog
* Thu Oct 11 2007 Jack Perdue <j-perdue@tamu.edu> 0-2007.10.11
- fix library versioning 

* Mon Oct  8 2007 Jack Perdue <j-perdue@tamu.edu> 0-2007.10.08
- Initial version

# EOF - RAPID.spec


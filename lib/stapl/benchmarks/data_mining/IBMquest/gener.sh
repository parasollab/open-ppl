# !/bin/sh -f
#############################################################################
# Generate input data sets for Frequent Itemset Mining
# which are sufficiently large for parallelism scaling studies
#############################################################################
date
#
# 100000 transactions, 10 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100 -tlen 10 -nitems 10 > T100L1N10.txt 
# 1000000 transactions, 10 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 1000 -tlen 10 -nitems 10 > T1000L1N10.txt 
# 10000000 transactions, 10 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 10000 -tlen 10 -nitems 10 > T10000L1N10.txt 
# 100000000 transactions, 10 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100000 -tlen 10 -nitems 10 > T100000L1N10.txt 
date
#
# 100000 transactions, 20 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100 -tlen 20 -nitems 10 > T100L2N10.txt 
# 1000000 transactions, 20 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 1000 -tlen 20 -nitems 10 > T1000L2N10.txt 
# 10000000 transactions, 20 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 10000 -tlen 20 -nitems 10 > T10000L2N10.txt
# 100000000 transactions, 20 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100000 -tlen 20 -nitems 10 > T100000L2N10.txt
date
#
# 100000 transactions, 30 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100 -tlen 30 -nitems 10 > T100L3N10.txt
# 1000000 transactions, 30 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 1000 -tlen 30 -nitems 10 > T1000L3N10.txt
# 10000000 transactions, 30 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 10000 -tlen 30 -nitems 10 > T10000L3N10.txt
# 100000000 transactions, 30 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100000 -tlen 30 -nitems 10 > T100000L3N10.txt
date
#
# 100000 transactions, 40 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100 -tlen 40 -nitems 10 > T100L4N10.txt
# 1000000 transactions, 40 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 1000 -tlen 40 -nitems 10 > T1000L4N10.txt
# 10000000 transactions, 40 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 10000 -tlen 40 -nitems 10 > T10000L4N10.txt
# 100000000 transactions, 40 average items per transactions, 10000 total items
./gen.exe lit -ascii -ntrans 100000 -tlen 40 -nitems 10 > T100000L4N10.txt
date
#

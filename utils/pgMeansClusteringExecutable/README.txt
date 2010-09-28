Steps for Running PGmeans Code

1. Run the UAS code <executable> -f <xml-file> i.e. ../../src/steval -f FSMhybrid.xml
2. Enter '3' at the cluster method prompt for pgmeans
3. Next run necessary Matlab files for solving queries in a particular
environment
	Files Needed and Brief Description:
	-pgmeans_simulation.m file. This file contains the PG-means algorithm/code. Therefore,
	this file will always be needed.
	-<environment>_script.m file. This file contains code that will run the PG-means
	code, with an alpha, number of projections, and data (SampleDataPoints.txt) as its inputs and the
	number (numClusters.txt), and location and orientation of clusters (pgmeansClusterOutput.txt) as its output.  This file also
	outputs which nodes are in a particular cluster calculated by the Euclidean Distance
	function. An example of <environment>_script.m is l_tunnel_script.m
	for the l-tunnel environment. This file is the one that needs to be
	ran at the 'Waiting for Matlab' prompt.
4. After Step 3, press Enter in the terminal running the UAS code and the code
will finish running

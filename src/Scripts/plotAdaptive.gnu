#
# Stacked histograms by percent
#
set terminal postscript enhanced color
set size 1.0,0.3
set autoscale y
set ytics auto
set border -1
set style data histograms
set style histogram gap 2
set style fill solid 0.75 border -1
set style line 1 lt 1 lw 1
set boxwidth 0.75
set key font ",10"
set key outside right samplen 1 width -5
#
set output "Nodes.eps"
set ylabel "Normalized Nodes"
set yrange [0:1.2]
plot 'Nodes.dat' using 2:xtic(1) t column(2) ls 1 lc rgb "#FF0000", \
	'' using 4:xtic(1) t column(4) ls 1 lc rgb "#00FF00", \
	'' using 6:xtic(1) t column(6) ls 1 lc rgb "#0000FF", \
	'' using 8:xtic(1) t column(8) ls 1 lc rgb "#FF00FF", \
	'' using 10:xtic(1) t column(10) ls 1 lc rgb "#00FFFF"
#

set output "CD.eps"
set ylabel "Normalized CD Calls"
set yrange [0:3]
#
plot 'CD.dat' using 2:xtic(1) t column(2) ls 1 lc rgb "#FF0000", \
	'' using 4:xtic(1) t column(4) ls 1 lc rgb "#00FF00", \
	'' using 6:xtic(1) t column(6) ls 1 lc rgb "#0000FF", \
	'' using 8:xtic(1) t column(8) ls 1 lc rgb "#FF00FF", \
	'' using 10:xtic(1) t column(10) ls 1 lc rgb "#00FFFF"
#

set output "Time.eps"
set ylabel "Normalized Time"
set yrange [0:1]
#
plot 'Time.dat' using 2:xtic(1) t column(2) ls 1 lc rgb "#FF0000", \
	'' using 4:xtic(1) t column(4) ls 1 lc rgb "#00FF00", \
	'' using 6:xtic(1) t column(6) ls 1 lc rgb "#0000FF", \
	'' using 8:xtic(1) t column(8) ls 1 lc rgb "#FF00FF", \
	'' using 10:xtic(1) t column(10) ls 1 lc rgb "#00FFFF"
#

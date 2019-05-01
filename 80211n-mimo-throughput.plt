set terminal postscript eps color enh "Times-BoldItalic"
set output "80211n-mimo-throughput.eps"
set xlabel "Distance (Meters)"
set ylabel "Throughput (Mbit/s)"
set xrange [0:100]
set yrange [0:600]
set ytics 0,50,600
set style line 1 dashtype 1 linewidth 5
set style line 2 dashtype 1 linewidth 5
set style line 3 dashtype 1 linewidth 5
set style line 4 dashtype 1 linewidth 5
set style line 5 dashtype 1 linewidth 5
set style line 6 dashtype 1 linewidth 5
set style line 7 dashtype 1 linewidth 5
set style line 8 dashtype 1 linewidth 5
set style line 9 dashtype 2 linewidth 5
set style line 10 dashtype 2 linewidth 5
set style line 11 dashtype 2 linewidth 5
set style line 12 dashtype 2 linewidth 5
set style line 13 dashtype 2 linewidth 5
set style line 14 dashtype 2 linewidth 5
set style line 15 dashtype 2 linewidth 5
set style line 16 dashtype 2 linewidth 5
set style line 17 dashtype 3 linewidth 5
set style line 18 dashtype 3 linewidth 5
set style line 19 dashtype 3 linewidth 5
set style line 20 dashtype 3 linewidth 5
set style line 21 dashtype 3 linewidth 5
set style line 22 dashtype 3 linewidth 5
set style line 23 dashtype 3 linewidth 5
set style line 24 dashtype 3 linewidth 5
set style line 25 dashtype 4 linewidth 5
set style line 26 dashtype 4 linewidth 5
set style line 27 dashtype 4 linewidth 5
set style line 28 dashtype 4 linewidth 5
set style line 29 dashtype 4 linewidth 5
set style line 30 dashtype 4 linewidth 5
set style line 31 dashtype 4 linewidth 5
set style line 32 dashtype 4 linewidth 5
set style increment user
plot "-"  title "HtMcs4" with lines ls 1
0 82.6063
10 82.5898
20 82.6086
30 82.4932
40 0
50 0
60 0
70 0
80 0
90 0
100 0
e

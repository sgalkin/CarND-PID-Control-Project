#!/usr/bin/env gnuplot

set term pngcairo enhanced size 800,600 font 'Hack'

set yrange [-1.75:1.75]

set output 'P.png'
set title 'P'
plot 0 with lines title 'zero', \
	'P.dat' using 1:2 with lines title 'P' lw 2, \
	''      using 1:3 with lines title 'P +10%' lw 2, \
	''      using 1:4 with lines title 'P -10%' lw 2

set output 'I.png'
set title 'I'
plot 0 with lines title 'zero', \
	'I.dat' using 1:2 with lines title '0.004' lw 2, \
	''      using 1:3 with lines title '0.000' lw 2, \
	''      using 1:4 with lines title '0.008' lw 2

set output 'D.png'
set title 'D'
plot 0 with lines title 'zero', \
	'D.dat' using 1:2 with lines title 'D' lw 2, \
	''      using 1:3 with lines title 'D -1' lw 2, \
	''      using 1:4 with lines title 'D +1' lw 2

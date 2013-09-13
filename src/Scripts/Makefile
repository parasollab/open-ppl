all: xmlgen scripts filter dataCollect normalize avg dist parsemap

xmlgen: xmlgen.cpp
	g++ xmlgen.cpp -o xmlgen

scripts: scripts.cpp
	g++ scripts.cpp -o scripts

filter: filter.cpp
	g++ filter.cpp -o filter

dataCollect: dataCollect.cpp
	g++ dataCollect.cpp -o dataCollect

normalize: normalize.cpp
	g++ normalize.cpp -o normalize

avg: avg.cpp
	g++ avg.cpp -o avg

dist: dist.cpp
	g++ dist.cpp -o dist

parsemap: parsemap.cpp
	g++ parsemap.cpp -o parsemap

clean:
	rm xmlgen scripts filter dataCollect normalize avg dist parsemap

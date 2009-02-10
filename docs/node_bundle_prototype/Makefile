BOOST_INCL = -I /usr/local/include

default_target: node_bundle_prototype

node_bundle_prototype:
	g++ -O3 main.cpp -o $@ $(BOOST_INCL)

clean:
	rm -f node_bundle_prototype

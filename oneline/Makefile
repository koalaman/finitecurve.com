CXXFLAGS := -Wall -Werror -ansi -pedantic -std=c++17 $(CXXFLAGS)
CXX ?= g++

main: oneline

clean:
	rm -f oneline image.o points.o oneline.js oneline.wasm

test: tests
	if type valgrind; then valgrind -q ./tests; else ./tests; fi

image.o: image.cpp image.h stb_image.h
	$(CXX) $(CXXFLAGS) -Wno-all -Wno-error -c image.cpp -o image.o

points.o: points.cpp points.h
	$(CXX) $(CXXFLAGS) -c points.cpp -o points.o

options.o: options.h options.cpp Jzon.h Jzon.cpp
	$(CXX) $(CXXFLAGS) -c options.cpp -o options.o

oneline: oneline.cpp image.o points.o
	$(CXX) $(CXXFLAGS) oneline.cpp points.o image.o -o oneline

tests: tests.cpp points.o
	$(CXX) $(CXXFLAGS) tests.cpp points.o -o tests -lcppunit

oneline.js: onelinejs.cpp points.o wrapper.js image.o options.o
	$(CXX) --bind $(CXXFLAGS) --pre-js wrapper.js onelinejs.cpp image.o points.o options.o -o oneline.js -s WASM=1 -g1 -s ASSERTIONS=1 -s ALLOW_MEMORY_GROWTH=1


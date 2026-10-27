CXX=g++
CXXFLAGS=-g -Wall -fPIC
CXXFLAGS+=-std=c++11

.cc.o:
	$(CXX) -c -o $@ $(CXXFLAGS) $(INCLUDES) $<

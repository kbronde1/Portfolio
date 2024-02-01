CXX = g++
CXXFLAGS = -g -Wall -Wextra -pedantic -std=c++11

assignment3 : assignment3_context.o assignment3_plugin.o
	$(CXX) -o assignment3 assignment3_context.o assignment3_plugin.o

assignment3_context.o : assignment3_context.cpp assignment3_context.h
	$(CXX) $(CXXFLAGS) -c assignment3_context.cpp

assignment3_plugin.o: assignment3_plugin.cpp
	$(CXX) $(CXXFLAGS) -c assignment3_plugin.cpp

clean:
	rm -f *.o assignment3
	rm -f assignment3

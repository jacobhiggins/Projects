CXX= g++
CXXFLAGS=-g -I ./eigen -std=c++11 -Wall

dp_make: DP_solver.cpp
	g++ -I ./eigen DP_solver.cpp -o DP_solver

dp2_make: DP_solver2.cpp
	$(CXX) $(CXXFLAGS) DP_solver2.cpp -o DP_solver2

map_test: map_test.cpp
	$(CXX) $(CXXFLAGS) map_test.cpp -o map_test
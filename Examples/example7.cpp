#include <iostream>
#include "graph.h"

int main()
{
    wgraph<char> g;
    g.add_edge('A', 'B', 4);
    g.add_edge('B', 'D', 9);
    g.add_edge('D', 'F', 2);
    g.add_edge('F', 'E', 6);
    g.add_edge('E', 'C', 3);
    g.add_edge('C', 'A', 5);
    g.add_edge('B', 'C', 11);
    g.add_edge('E', 'D', 13);
    g.add_edge('E', 'B', 7);

    std::cout << "By Dijkshtra's Algorithm.... " << std::endl;
    std::cout << std::endl;

    std::map<char, int> V = g.dijkstra('A');
    std::cout << "Shortest Distance From A to all other nodes : " << std::endl;
    for (auto it : V)
    {
        std::cout << it.first << " : " << it.second << std::endl;
    }
    std::cout<<std::endl;
    std::vector<char> V1 = g.path('A', 'F');
    std::cout << "Shortest Path From A to F is : " ;
    for (auto it : V1)
    {
        std::cout<<it<<" ";
    }
    std::cout<<std::endl;

    return 0;
}
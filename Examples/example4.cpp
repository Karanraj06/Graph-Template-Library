#include <iostream>
#include "graph.h"

int main()
{
    digraph<int> g;
    g.add_edge(1, 3);
    g.add_edge(2, 0);
    g.add_edge(3, 2);
    g.add_edge(4, 0);
    g.add_edge(4, 1);
    g.add_edge(4, 2);

    
    std::vector<int> V = g.topological_sort();
    std::cout << "The Topological sort is : ";
    for (auto it : V)
    {
        std::cout << it << " ";
    }
    std::cout << std::endl;
    
    return 0;
}
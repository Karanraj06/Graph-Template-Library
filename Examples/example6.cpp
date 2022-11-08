#include <iostream>
#include "graph.h"

int main()
{
    wdigraph<int> g;
    g.add_edge(0, 1, -5);
    g.add_edge(0, 3, 3);
    g.add_edge(0, 2, 2);
    g.add_edge(2, 3, 1);
    g.add_edge(1, 2, 4);
    
    std::cout<<"By Bellman Ford Algorithm.... "<<std::endl;
    std::cout << std::endl;

    std::vector<std::vector<int>> V = g.bellman_ford();
    std::cout << "Shortest Distance From all nodes to all other nodes : " << std::endl;

    std::cout << std::endl;
    std::cout << "  ";
    for (auto i : g.adj)
    {
        std::cout << i.first << " ";
    }
    std::cout << std::endl;
    auto it = g.adj.begin();
    for (auto i : V)
    {
        std::cout << it->first << " ";
        for (auto j : i)
        {
            if (j == INT_MAX)
            {
                std::cout << "N ";
            }
            else
            {
                std::cout << j << " ";
            }
        }
        std::cout << std::endl;
        it++;
    }
    std::cout << std::endl;

    return 0;
}
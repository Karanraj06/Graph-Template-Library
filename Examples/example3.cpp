#include <iostream>
#include "graph.h"

int main()
{
    graph<int> g;
    g.add_edge(0, 1);
    g.add_edge(0, 3);
    g.add_edge(0, 4);
    g.add_edge(1, 2);
    g.add_edge(2, 3);
    g.add_edge(3, 4);
    g.add_edge(4, 1);

    std::cout << "The Adjacency List is : " << std::endl;

    for (auto it : g.adj)
    {
        std::cout << it.first << " : ";
        for (auto i : it.second)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    if (g.is_bipartite())
    {
        std::cout << "The Graph is Bipartite" << std::endl;
    }
    else
    {
        std::cout << "The Graph is not Bipartite" << std::endl;
    }
    g.remove_edge(4, 1);
    std::cout << std::endl;
    std::cout << "The Adjacency List after removing edge between 4 and 1 is : " << std::endl;

    for (auto it : g.adj)
    {
        std::cout << it.first << " : ";
        for (auto i : it.second)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    g.remove_node(4);
    std::cout << std::endl;
    std::cout << "The Adjacency List after removing node 4 is : " << std::endl;

    for (auto it : g.adj)
    {
        std::cout << it.first << " : ";
        for (auto i : it.second)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    if (g.is_bipartite())
    {
        std::cout << "The Graph is Bipartite" << std::endl;
    }
    else
    {
        std::cout << "The Graph is not Bipartite" << std::endl;
    }
    return 0;
}
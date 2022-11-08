#include <iostream>
#include "graph.h"

int main()
{
    graph<int> g;
    g.add_edge(0, 1);
    g.add_edge(0, 4);
    g.add_edge(0, 5);
    g.add_edge(1, 2);
    g.add_edge(1, 4);
    g.add_edge(2, 3);
    g.add_edge(2, 4);
    g.add_edge(3, 5);
    g.add_edge(4, 5);
    g.add_edge(6, 7);
    g.add_edge(6, 8);

    g.add_node(9);

    std::vector<int> V1 = g.dfs(0);
    std::vector<int> V2 = g.bfs(0);
    std::cout << "The DFS Traversal is : ";
    for (auto it : V1)
    {
        std::cout << it << " ";
    }
    std::cout << std::endl;
    std::cout << "The BFS Traversal is : ";
    for (auto it : V2)
    {
        std::cout << it << " ";
    }
    std::cout << std::endl;

    if (g.cyclic())
    {
        std::cout<<"Graph has atleast one cycle."<<std::endl;
    }
    else{
        std::cout<<"Graph doesn't have a cycle"<<std::endl;
    }
    return 0;
}
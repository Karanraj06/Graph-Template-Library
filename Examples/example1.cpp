#include <iostream>
#include "graph.h"

int main() {
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

    
    std::cout<<"The Number of Nodes is : "<<g.number_of_nodes()<<std::endl;
    std::cout<<"The Number of Edges is : "<<g.number_of_edges<<std::endl;
    std::cout<<"The Number of Connected Components is : "<<g.number_of_connected_components()<<std::endl;
    std::cout<<"The Degree of 4 is : "<<g.degree(4)<<std::endl;
    std::map<int,std::vector<int>>M= g.adj;
    std::cout<<"\n";
    std::cout<<"The Adjacency List is : "<<std::endl;
    
    for(auto it:g.adj){
        std::cout<<it.first<<" : ";
        for(auto i:it.second){
            std::cout<<i<<" ";
        }
        std::cout<<std::endl;
    }


    
    return 0;
}
# Graph-Template-Library
Defines container template classes to study graphs in C++
# Summary
- Graph template library to study graphs in C++
- Generic graph library
- Defines several container template classes
- Data structures for graphs, digraphs and weighted graphs
- Many standard graph algorithms
- Nodes can be any built-in data type
# Example
Finds number of connected components in an undirected graph
```cpp
#include <iostream>
#include "graph.h"

int main() {
    graph<char> g;
    g.add_edge('a', 'b');
    g.add_edge('c', 'd');
    g.add_edge('d', 'e');
    g.add_edge('f', 'g');

    std::cout << "Number of connected components in g are " << g.number_of_connected_components() << "\n";
    return 0;
}
```
Output
```
Number of connected components in g are 3
```
# Test
## Terminal commands to compile and run C++ program
### Change directory to current working directory
```
% cd file_path
```
For example,
```
% cd "Desktop/3rd Semester/CS201/CS201_Project_Group_1/Code_file_Group_1/Examples"
```
OR
```
% cd Desktop/3rd\ Semester/CS201/CS201_Project_Group_1/Code_file_Group_1/Examples
```
### Compile C++ program
```
% g++ -std=c++20 file_name
```
For example,
```
% g++ -std=c++20 example1.cpp
```
### Run C++ program
```
% ./a.out
```

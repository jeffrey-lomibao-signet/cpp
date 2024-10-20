# Overview
This homework3 builds on homework2.
A new Graph constructor is added where it takes a name of a file that contains the graph data.
A new Graph method is added that gets the MST value of a graph.

# Class Hierarchy
## Edge
- defined as a distance to a node
- starting node is defined by its position in a list of vertices

## Edge List
- contains a list of edges in a given node

## Graph
- defined as an adjacency list
- contains a list of vertices, where each vertex is a list of edges
- The index in this list is used as the starting node.

## Priority Queue
- maintains a list where the minimum node value is kept at the top

## Priority Queue Element
- defines data type to be used in a priority queue
- contains node and value, where value is the total distance to get to the node

# File Structure
`main.cpp` 
- this is the main file that instantiates graphs and runs tests
- supporting classes are defined in separate header files
- header files contain implementations to be able to build the project easier
- easier here means without makefiles or custom VS Code build settings

`mst_example.txt`
- Example in section 4.2 Jarnik-Prim MST

`mst_prim_example.txt`
- Example in section 4.3 Jarnik-Prim MST:Another Look

`mst_kruskal_example.txt`
- Example in section 4.4 Kruskal's Algorithm

`mst_data.txt`
- Example data given in this homework

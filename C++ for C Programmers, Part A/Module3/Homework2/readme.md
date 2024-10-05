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

## Shortest Path
- contains methods that determine shortest path in a graph
- uses Djikstra's algorithm
- uses a priority queue

## Priority Queue
- maintains a list where the minimum node value is kept at the top

## Priority Queue Element
- defines data type to be used in a priority queue
- contains node and value, where value is the total distance to get to the node

## Graph Instances
### Example Graph
- directed graph that was presented in Module 2

### Wikipedia Graph
- undirected graph that is presented in Djiktra's Algorithm Wikipedia page

### Random Graphs
- undirected graphs created with random nodes and edges
- edges are randomly generated given a density and distance range

# File Structure
- `dijkstra.cpp` is the main file that instantiates graphs and runs tests
- supporting classes are defined in separate header files
- header files contain implementations to be able to build the project easier
- easier here means without makefiles or custom VS Code build settings

# What I've learned
## Graph Theory
- terminologies like node, edge, and density
- difference between directed and undirected graph
- matrix vs adjacency list representations
## C++ Concepts
- different ways to iterate over vectors
- deconstructing a big class into smaller classes
- value of being able to print out custom data types
- how to use new random functions in STL
- how to use `friend` classes

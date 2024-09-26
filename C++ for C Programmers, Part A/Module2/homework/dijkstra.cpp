#include <iostream>
#include <vector>

using namespace std;

//=============================================================================
class Edge {
public:
  Edge(int n = 0, int d = 0)
    : node{n}, distance{d} {}

  int getNode() { return node; }
  int getDistance() { return distance; }
  void setNode(int n) { node = n; }
  void setDistance(int d) { distance = d; }
  void setEdge(int n, int d) { setNode(n); setDistance(d); }
  Edge getEdge() { return Edge(node, distance); }

  friend ostream& operator<<(ostream& out, const Edge& e) {
    out << "(" << e.node << "," << e.distance << ")";
    return out;
  }

private:
  int node;
  int distance; 
};

//=============================================================================
class Vertex {
public:
  Vertex() {};
  void add(Edge e) { v.push_back(e); }
  int getNumEdges() { return v.size(); }

// private:
  vector<Edge> v;
};

//=============================================================================
class Graph {
public:
  int getNumVertices() { return g.size(); }; // V() returns the number of vertices in the graph
  int getNumEdges(); // E() returns the number of edges in the graph
// adjacent (G, x, y): tests whether there is an edge   from node x to node y.
// neighbors (G, x): lists all nodes y such that there is an edge from x to y.
  void add(int node1, int node2, int distance); // (G, x, y): adds to G the edge from x to y, if it is not there.
  void add(int node);
// delete (G, x, y): removes the edge from x to y, if it is there.
// get_node_value (G, x): returns the value associated with the node x.
// set_node_value( G, x, a): sets the value associated with the node x to a.
// get_edge_value( G, x, y): returns the value associated to the edge (x,y).
// set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.
// One important consideration for the Graph class is how to represent the graph as a member ADT. Two basic implementations are generally considered: adjacency list and adjacency matrix depending on the relative edge density. For sparse graphs, the list approach is typically more efficient, but for dense graphs, the matrix approach can be more efficient (reference an Algorithm’s source for space and time analysis). Note in some cases such as add(G, x, y) you may also want to have the edge carry along its cost. Another approach could be to use (x, y) to index a cost stored in an associated array or map.

// private:
  vector<Vertex> g;
};

int Graph::getNumEdges() {
  int numEdges{0};
  for(Vertex v: g) {
    numEdges += v.getNumEdges();
  }
  return numEdges;
}

void Graph::add(int node1, int node2, int distance) {
  Edge e{node2, distance};
  if(node1 >= getNumVertices()) {
    Vertex v;
    v.add(e);
    g.push_back(v);
  }
  else {
    g.at(node1).add(e);
  }
}

void Graph::add(int node) {
  if(node >= getNumVertices()) {
    Vertex v;
    g.push_back(v);
  }
}

//=============================================================================
typedef enum { S, A, B, C, D, E, F, G, T } Nodes;
constexpr int NUM_NODES = int(T) + 1; 
ostream& operator<<(ostream& out, const Nodes& n) {
  switch(n) {
  case S: out << "S"; break;
  case A: out << "A"; break;
  case B: out << "B"; break;
  case C: out << "C"; break;
  case D: out << "D"; break;
  case E: out << "E"; break;
  case F: out << "F"; break;
  case G: out << "G"; break;
  case T: out << "T"; break;
  }
  return out;
}

Nodes operator++(Nodes& n) { 
  n = static_cast<Nodes>((static_cast<int>(n) + 1) % NUM_NODES);
  return n; 
}
Nodes operator++(Nodes& n, int) {
  Nodes temp = n;
  n = Nodes((int(n) + 1) % NUM_NODES);
  return temp;
}

//=============================================================================
void addExampleGraph(Graph& g) {
  g.add(Nodes::S, Nodes::A, 4);
  g.add(Nodes::S, Nodes::B, 3);
  g.add(Nodes::S, Nodes::D, 7);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::A, Nodes::C, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::B, Nodes::D, 4);
  g.add(Nodes::B, Nodes::S, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::C, Nodes::D, 3);
  g.add(Nodes::C, Nodes::E, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::D, Nodes::F, 5);
  g.add(Nodes::D, Nodes::T, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::E, Nodes::G, 2);
  g.add(Nodes::E, Nodes::T, 4);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::F);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::G, Nodes::T, 3);
  g.add(Nodes::G, Nodes::E, 2);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Nodes::T, Nodes::F, 5);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
}

void printExampleGraph(const Graph& g) {
  Nodes node{S};
  for (auto v: g.g) {
    cout << node << ":";
    for (auto e: v.v) {
      cout << " " << e; 
    }
    cout << endl;
    ++node;
  }
}

//=============================================================================
int main() {
  Graph g;
  addExampleGraph(g);
  printExampleGraph(g);
  return 0;
}

#include <iostream>
#include <vector>
#include <cassert>

using namespace std;

//=============================================================================
enum class Node { S, A, B, C, D, E, F, G, T };
constexpr int NUM_NODES = int(Node::T) + 1; 
constexpr int INVALID_NODE = -1;

ostream& operator<<(ostream& out, const Node& n) {
  switch(n) {
  case Node::S: out << "S"; break;
  case Node::A: out << "A"; break;
  case Node::B: out << "B"; break;
  case Node::C: out << "C"; break;
  case Node::D: out << "D"; break;
  case Node::E: out << "E"; break;
  case Node::F: out << "F"; break;
  case Node::G: out << "G"; break;
  case Node::T: out << "T"; break;
  default: out << "-";
  }
  return out;
}

Node operator++(Node& n) { 
  n = static_cast<Node>((static_cast<int>(n) + 1) % NUM_NODES);
  return n; 
}
Node operator++(Node& n, int) {
  Node temp = n;
  n = Node((int(n) + 1) % NUM_NODES);
  return temp;
}

ostream& operator<<(ostream& out, const vector<Node>& nodes) {
  cout << "[";
  for (auto n = nodes.begin(); n != nodes.end(); ++n) {
      cout << *n;
      if ((nodes.size() > 1) && (n != (nodes.end() - 1))) {
        cout << ",";
      } 
  }
  cout << "]";
  return out;
}

//=============================================================================
class Edge {
  friend class Vertex;
  friend ostream& operator<<(ostream& out, const Edge& e);

public:
  Edge(Node n, size_t d): node{n}, distance{d} {}
  Node getNode() { return node; }
  size_t getDistance() { return distance; }
  void setNode(Node n) { node = n; }
  void setDistance(int d) { distance = d; }
  void setEdge(Node n, int d) { setNode(n); setDistance(d); }
  Edge getEdge() { return Edge(node, distance); }

private:  
  Node node;
  size_t distance; 
};

ostream& operator<<(ostream& out, const Edge& e) {
  out << "(" << e.node << "," << e.distance << ")";
  return out;
}

//=============================================================================
class Vertex {
  friend class Graph;

public:
  vector<Edge>& getEdges() { return edges; };
  void addEdge(Edge e) { edges.push_back(e); }
  void deleteEdge(Node node);
  int getNumEdges() { return edges.size(); }
  bool isNodePresent(const Node node);
  size_t getEdgeValue(const Node node);
  void setEdgeValue(const Node node, size_t distance);
  friend ostream& operator<<(ostream& out, const Vertex& v);
  Node getNodeWithShortestDistance();
  
private:
  vector<Edge> edges;
};

ostream& operator<<(ostream& out, const Vertex& v) {
  for (auto e: v.edges) {
    cout << " " << e; 
  }
  return out;
}

bool Vertex::isNodePresent(Node node) {
  bool edgeFound{false};
  for (auto e: edges) {
    if (e.getNode() == node) {
      edgeFound = true;
      break;
    }
  }
  return edgeFound;
}

void Vertex::deleteEdge(Node node) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      edges.erase(e);
      break;
    }
  }
}

size_t Vertex::getEdgeValue(const Node node) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      return e->getDistance();
    }
  }
  return 0;
}

void Vertex::setEdgeValue(const Node node, size_t distance) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      return e->setDistance(distance);
    }
  }
}

Node Vertex::getNodeWithShortestDistance() {
  Node node{Node(INVALID_NODE)};
  size_t numEdges = edges.size();
  if (edges.size() > 0) {
    size_t minDistance{edges.at(0).getDistance()};
    node = edges.at(0).getNode();
    for (size_t i{1}; i < numEdges; ++i) {
      if (edges.at(i).getDistance() < minDistance) {
        node = edges.at(i).getNode();
      }
    }
  }
  return node;
}

//=============================================================================
class Graph {
  friend class ShortestPath;
  friend ostream& operator<<(ostream& out, const Graph& g);

public:
  int getNumVertices() { return vertices.size(); } // V() returns the number of vertices in the graph
  int getNumEdges(); // E() returns the number of edges in the graph
  bool isNodePresent(Node node); //tests whether a node is present in the graph
  bool isEdgePresent(Node nodeX, Node nodeY);// adjacent (G, x, y): tests whether there is an edge from node x to node y.
  const Vertex& getNeighborsList(Node nodeX); // neighbors (G, x): lists all nodes y such that there is an edge from x to y.
  void addNode(Node node);
  void addEdge(Node nodeX, Node nodeY, size_t distance); // (G, x, y): adds to G the edge from x to y, if it is not there.
  void deleteEdge(Node nodeX, Node nodeY); // delete (G, x, y): removes the edge from x to y, if it is there.
  int getNodeValue(Node node); // get_node_value (G, x): returns the value associated with the node x.
  // setNodeValue is not needed since we are using the vertex index as the node value
  // set_node_value( G, x, a): sets the value associated with the node x to a.
  size_t getEdgeValue(Node nodeX, Node nodeY); // get_edge_value( G, x, y): returns the value associated to the edge (x,y).
  void setEdgeValue(Node nodeX, Node nodeY, size_t distance);// set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.

private:
  // One important consideration for the Graph class is how to represent the graph as a member ADT. 
  // Two basic implementations are generally considered: adjacency list and adjacency matrix depending on the relative edge density. 
  // For sparse graphs, the list approach is typically more efficient, but for dense graphs, the matrix approach can be more efficient 
  // (reference an Algorithm’s source for space and time analysis). 
  // Note in some cases such as add(G, x, y) you may also want to have the edge carry along its cost. 
  // Another approach could be to use (x, y) to index a cost stored in an associated array or map.
  vector<Vertex> vertices; // use adjacency list to represent the graph
};

ostream& operator<<(ostream& out, const Graph& g) {
  Node node{Node(0)};
  for (auto v: g.vertices) {
    cout << node << ":";
    cout << v;
    cout << endl;
    ++node;
  }
  return out;
}

int Graph::getNumEdges() {
  int numEdges{0};
  for(Vertex v: vertices) {
    numEdges += v.getNumEdges();
  }
  return numEdges;
}

bool Graph::isNodePresent(Node node) {
  return int(node) < getNumVertices();
}

bool Graph::isEdgePresent(Node nodeX, Node nodeY) {
  bool edgeFound{false};
  if(isNodePresent(nodeX)) {
    edgeFound = vertices.at(size_t(nodeX)).isNodePresent(nodeY);
  }
  return edgeFound;
}

const Vertex& Graph::getNeighborsList(Node nodeX) {
  assert(isNodePresent(nodeX));
  return vertices.at(size_t(nodeX));
}

void Graph::addNode(Node node) {
  if(!isNodePresent(node)) {
    Vertex v;
    vertices.push_back(v);
  }
}

void Graph::addEdge(Node nodeX, Node nodeY, size_t distance) {
  Edge e{nodeY, distance};
  if (!isNodePresent(nodeX)) {
    Vertex v;
    v.addEdge(e);
    vertices.push_back(v);
  }
  else if (!isEdgePresent(nodeX, nodeY)) {
    vertices.at(size_t(nodeX)).addEdge(e);
  }
}

void Graph::deleteEdge(Node nodeX, Node nodeY) {
  if (isEdgePresent(nodeX, nodeY)) {
    vertices.at(size_t(nodeX)).deleteEdge(nodeY);
  }
}

int Graph::getNodeValue(Node node) {
  if (isNodePresent(node))
    return int(node); 
  return INVALID_NODE;
}

size_t Graph::getEdgeValue(Node nodeX, Node nodeY) {
  if (isEdgePresent(nodeX, nodeY)) {
    return vertices.at(size_t(nodeX)).getEdgeValue(nodeY);
  }
  return 0;
}

void Graph::setEdgeValue(Node nodeX, Node nodeY, size_t distance) {
  if (isEdgePresent(nodeX, nodeY)) {
    return vertices.at(size_t(nodeX)).setEdgeValue(nodeY, distance);
  }
}

//=============================================================================
class ShortestPath {
  friend ostream& operator<<(ostream& out, const vector<size_t>& v);

public:
  Graph& getGraph() { return g; }
  vector<Vertex>& getVertices(); // vertices(List): list of vertices in G(V,E).
  vector<Node>& path(Node u, Node w); // path(u, w): find shortest path between u-w and returns the sequence of vertices representing shortest path u-v1-v2-…-vn-w.
  size_t pathSize(Node u, Node w); // path_size(u, w): return the path cost associated with the shortest path.

private:
  Graph g;
  void initShortestPathSearch(Node u, Node w);
  Node start, destination;
  size_t numNodes;
  vector<Node> Q; // list of nodes that have not been visited
  void initNodesNotVisitedList();
  bool isNodeNotVisited(Node n);
  vector<size_t>dist; // list of shortest distances to nodes
  void initShortestDistanceToNodesList();
  vector<Node> prev; // list of previous nodes with shortest distance to a node
  void initPreviousNodesList();
  vector<Node> S; // list of nodes that is the shortest path
  vector<Node>& createShortestPathFromPrevNodesList();
  Node findNodeWithMinDistance();
  void markNodeAsVisited(Node u);
  void traverseNeighbors(Node u);
  size_t calcTotalDistanceToNeighbor(Node u, Node v);
  void updateMinDistanceAndPreviousNodesLists(Node u, Node v, size_t alt);
};

ostream& operator<<(ostream& out, const vector<size_t>& v) {
  cout << "[ ";
  Node n{Node(0)};
  for (auto d: v) {
    cout << "(" << n << ":" << d << ") ";
    ++n;
  }
  cout << "]";
  return out;
}

vector<Vertex>& ShortestPath::getVertices() {
  return g.vertices;
}

void ShortestPath::initNodesNotVisitedList() {
  Q.clear();
  for (size_t i{0}; i < numNodes; ++i) {
    Q.push_back(Node(i));
  }
  cout << "Q: " << Q << endl;
}

bool ShortestPath::isNodeNotVisited(Node n) {
  bool found{false};
  for (Node node: Q) {
    if (node == n) {
      found = true;
      break;
    }
  }
  return found;
}

void ShortestPath::initShortestDistanceToNodesList() {
  dist.clear();
  dist.push_back(0);
  for (size_t i{1}; i < numNodes; i++) {
    dist.push_back(SIZE_MAX);
  }
  cout << "dist: " << dist << endl;
}

void ShortestPath::initPreviousNodesList() {
  prev.clear();
  for (size_t i{0}; i < numNodes; ++i) {
    prev.push_back(Node(INVALID_NODE));
  }
  cout << "prev: " << prev << endl;
}

vector<Node>& ShortestPath::createShortestPathFromPrevNodesList() {
  S.clear(); // empty sequence
  Node u{destination}; // u = target
  if (prev[size_t(u)] != Node(INVALID_NODE) or u == start) { // if pre[u] is defined or u = source
    while (u != Node(INVALID_NODE)) { // while u is defined
      S.insert(S.begin(), u); // insert u at beginning of S
      u = prev[size_t(u)]; // u = prev[u]
    }
  }
  return S;
}

Node ShortestPath::findNodeWithMinDistance() {
  size_t min = SIZE_MAX;
  Node u{Node(INVALID_NODE)};
  for (auto i{Q.begin()}; i < Q.end(); ++i) {
    Node n = *i;
    if(dist[size_t(n)] < min) {
      min = dist[size_t(n)];
      u = n;
    }
  }
  cout << "u: " << u << endl;
  return u;
}

void ShortestPath::markNodeAsVisited(Node u) {
  // remove u from Q
  for (auto i{Q.begin()}; i < Q.end(); ++i) {
    if (*i == u) {
      Q.erase(i);
      break;
    }
  }
  cout << "Q: " << Q << endl;
}

void ShortestPath::updateMinDistanceAndPreviousNodesLists(Node u, Node v, size_t alt) {
  if (alt < dist[size_t(v)]) { // if alt < dist[v]
    dist[size_t(v)] = alt; // dist[v] = alt
    cout << "dist: " << dist << endl;
    prev[size_t(v)] = u; // prev[v] = u
    cout << "prev: " << prev << endl;
  }
}

void ShortestPath::traverseNeighbors(Node u) {
  // for each neighbor v of u still in Q
  Vertex neighbors = g.getNeighborsList(u);
  cout << "neighbors: " << neighbors << endl;
  for (auto e: neighbors.getEdges()) {
    Node v = e.getNode();
    if (isNodeNotVisited(v)) {
      size_t alt = calcTotalDistanceToNeighbor(u,v);
      updateMinDistanceAndPreviousNodesLists(u,v,alt);
    }
  }
}

size_t ShortestPath::calcTotalDistanceToNeighbor(Node u, Node v) {
  // alt = dist[u] + G.edges(u,v)
  size_t alt{dist[size_t(u)]};
  alt += g.getEdgeValue(u, v);
  cout << "alt: " << u << "," << v << "," << alt << endl;
  return alt;
}

void ShortestPath::initShortestPathSearch(Node u, Node w) {
  // make sure graph is not empty
  numNodes = g.getNumVertices();
  assert(numNodes > 0);
  start = u;
  destination = w;
  initNodesNotVisitedList();
  initShortestDistanceToNodesList();
  initPreviousNodesList();
}

vector<Node>& ShortestPath::path(Node u, Node w) {
  // Use Dijkstra's algorithm as described here:
  // https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
  initShortestPathSearch(u,w);
  while( Q.size() > 0) { // while Q is not empty
    cout << "===============" << endl;
    Node u{findNodeWithMinDistance()};
    if (u == destination)
      break;
    markNodeAsVisited(u);
    traverseNeighbors(u);
  }
  return createShortestPathFromPrevNodesList();
}

size_t ShortestPath::pathSize(Node u, Node w) {
  vector<Node>& nodes{S};
  if (u != start or w != destination) {
    nodes = path(start, destination);
  }
  size_t distance{0};
  size_t numNodes{nodes.size()}, index{0};
  while (numNodes-- > 1) {
    distance += g.getEdgeValue(nodes[index], nodes[index+1]);
    ++index;
  }
  return distance;
}

//=============================================================================
void addExampleGraph(Graph& g) {
  g.addEdge(Node::S, Node::A, 4);
  g.addEdge(Node::S, Node::B, 3);
  g.addEdge(Node::S, Node::D, 7);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::A, Node::C, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::B, Node::D, 4);
  g.addEdge(Node::B, Node::S, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::C, Node::D, 3);
  g.addEdge(Node::C, Node::E, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::D, Node::F, 5);
  g.addEdge(Node::D, Node::T, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::E, Node::G, 2);
  g.addEdge(Node::E, Node::T, 4);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addNode(Node::F);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::G, Node::T, 3);
  g.addEdge(Node::G, Node::E, 2);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::T, Node::F, 5);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::S, Node::T, 20);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
  g.deleteEdge(Node::S, Node::T);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  cout << "Graph:" << endl << g;

  cout << "S->A = " << g.getEdgeValue(Node::S, Node::A) << endl;
  cout << "G->T = " << g.getEdgeValue(Node::G, Node::T) << endl;
  g.setEdgeValue(Node::G, Node::T, 5);
  cout << "G->T = " << g.getEdgeValue(Node::G, Node::T) << endl;
}

//=============================================================================
int main() {
  ShortestPath sp;
  addExampleGraph(sp.getGraph());
  auto path = sp.path(Node::S, Node::T);
  cout << "shortest path = " << path << endl;
  size_t distance = sp.pathSize(Node::S, Node::T);
  cout << "distance = " << distance << endl;

  return 0;
}

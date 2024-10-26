// Find a Minimum Spanning Tree.

// style for names is
// - CapitalizedCamelCaps for class names
// - camelCap for all other names
// - when initials used (like MST) add _ to improve readability of the name

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;
// using namespace std::chrono;

// helper class for Graph
// used to show if edge exists, and if so, the distance.
// used instead of indicating invalid distance by some large number, -1, or some other hack.
class MatrixEdgeCandidate {
public:
   MatrixEdgeCandidate() :
      valid(false),
      distance(0) { }

   // defining a distance means the edge is valid (i.e. the MatrixEdgeCandidate is an edge)
   int operator= (const int d) {
      distance = d;
      valid = true;
      return d;       // return the distance so can chain assignments: i = j = k = l;
   }

   int getDistance() const { return distance; }

   bool isValid() const { return valid; }

private:
   bool valid;
   int distance;
};

// class graph
// In this implementation, Class Graph is as a paper map, which once created, does not change;
//    the graph doesn't keep track of anything dynamic, it only holds if the edge is valid and
//    if so, the edge's distance.
// because the graph does not change once built, it is built as a const.
class Graph {
public:
   // ctor
   // build 2d vector (adjacency matrix), showing travel distance from one vertex to another.
   //      position edgeMatrix[4][5] has distance between vertices 4 and 5, if valid.
   //      position edgeMatrix[5][4] is same as above
   Graph(const string fileName):
      edgeMatrix(createEdgeMatrix(fileName)) { }

   // ctor from previous assignment
   Graph(const int density=20,
         const int numVertices=50,
         const int low=1,
         const int high=40):
      numberOfVertices(numVertices),
      edgeMatrix(createEdgeMatrix(density, numVertices, low, high)) { }

   int getEdgeValue(const int x, const int y) const { return edgeMatrix[x][y].getDistance(); }
  
   int getVertexCount() const { return numberOfVertices; }
  
   // return true if an edge between the two vertices
   bool hasEdge(const int v1,
		const int v2) const { return(edgeMatrix[v1][v2].isValid()); }

private:
   int numberOfVertices;
   const vector<vector<MatrixEdgeCandidate>> edgeMatrix;

  
   //initialization function to create the Edge Matrix when reading from a file
   vector<vector<MatrixEdgeCandidate> > createEdgeMatrix(const string fileName) {
      ifstream file(fileName);
      if (file.is_open()) {
	 file >> numberOfVertices;
	 //create vector to be returned to initialize the const Edge adjacency matrix
	 vector<vector<MatrixEdgeCandidate> > m(numberOfVertices, vector<MatrixEdgeCandidate> (numberOfVertices));
	 int end, distance;
	 for (int start; file >> start;) {
	    file >> end;
	    file >> distance;
	    m[start][end] = m[end][start] = distance;     // this is requirement of undirected graph
	 }
	 return m;
      }
      exit(1);
   }

   //initialization function to create the Edge Matrix using random graph.
   //this initialization method is from previous assignment 
   vector<vector<MatrixEdgeCandidate> > createEdgeMatrix(const int density,
						   const int vertices,
						   const int low,
						   const int high) {
      srand(time(0));                  // seed the pseudo-random number generator
      // create the connectivity matrix map with no edges
      vector<vector<MatrixEdgeCandidate> > m(vertices, vector<MatrixEdgeCandidate> (vertices));
      for (int vertexOne = 0; vertexOne < m.size()-1; ++vertexOne) {
         for (int vertexTwo = vertexOne + 1; vertexTwo < m[vertexOne].size(); ++vertexTwo) {
            if ((rand() % 100) < density) {
               // use the distance to make the undirected connection in the map
               m[vertexOne][vertexTwo] = m[vertexTwo][vertexOne] = (rand() % (high - low + 1)) + 1;
            }
         }
     }
     return m;
   }

};
// end of class Graph


// function to print out the Edge Matrix
ostream& operator << (ostream& out, const Graph& g) {
   // first print out a two line header for each column showing the vertex number
   out << "\nEdge Matrix: \n'--' means no edge between those two vertices.\n   ";
   for (int col = 0; col < g.getVertexCount(); ++col) {
      out << setfill(' ') << setw(4) << col;
   }
   out << endl;
  
   out << "   ";
   for (int col = 0; col < g.getVertexCount(); ++col) {
      out << " ___";
   }
   out << endl;

   // now print out each row showing distances from that vertex to the other vertices
   // each row starts with the vertex number
   for (int startVertex = 0; startVertex < g.getVertexCount(); ++startVertex) {
      out << setfill(' ') << setw(2) << startVertex << ": ";
      for (int endVertex = 0; endVertex < g.getVertexCount(); ++endVertex){
         if (!g.hasEdge(startVertex, endVertex)) {
	    out << " -- ";
	 }
	 else {
	    out << setfill(' ') << setw(3) << fixed << setprecision(1) << g.getEdgeValue(startVertex, endVertex) << " ";
          }
      }
      out << " :" << startVertex << endl;
      //out << endl;
   }
   return out;
}


// helper class to hold edges in the MST and in the candidate vectors
class Edge {
public:
   Edge(int v1, int v2, int d) :
      vertex1(v1),
      vertex2(v2),
      distance(d) {}

   bool operator< (Edge e2) const {
      return distance < e2.distance;
   }

   int v1() const { return vertex1; }
   int v2() const { return vertex2; }
   int dist() const { return distance; }

private:
   int vertex1,
       vertex2,
       distance;
};


// class to manage the vertex objects
// and give each vertex's shortest edge
class Vertex {
public:
   // ctor()
   Vertex(const int me,
	 vector<Vertex> &usedVertices) :
      myIndex(me) {
      buildCandidateVector(usedVertices);
   }

   bool hasCandidates() const { return (candidateEdges.size() > 0); }
  
   // shortest is first in candidates list
   Edge getShortestCandidate() const { return *candidateEdges.begin(); }

   int getMyIndex() const { return myIndex; }

   // remove the given candidate from the candidateEdges vector
   void removeCandidate(const int candidate) {
      for (vector<Edge>::iterator position = candidateEdges.begin(); position < candidateEdges.end(); ++position) {
         if (position->v1() == candidate) {
            // remove candidate from the candidates vector
            candidateEdges.erase(position);
            break;
         }
      }
   }

  const static Graph *EM;

private:
   const int myIndex;              // index of self in list of vertices
   vector<Edge> candidateEdges;    // vertices connected by edges to this vertex, which are
                                   // candidates for the tree from this vertex

   // return true if vertex is in vertices
   bool isVertexInList(const int vertex, vector<Vertex> &vertices) const {
      for (vector<Vertex>::iterator v = vertices.begin(); v < vertices.end(); ++v) {
          if (vertex == v->myIndex)
             return true;
      }
      return false;
   }

   // find all of the edges that this vector is on, and add them to this vertex's candidates vector
   void buildCandidateVector(vector<Vertex> &usedVertices) {
      // build candidates vector from the edgeMatrix
      for (int candidateVertex = 0; candidateVertex < EM->getVertexCount(); ++candidateVertex) {         // check each column
         if (EM->hasEdge(myIndex, candidateVertex) &&
	     !isVertexInList(candidateVertex, usedVertices)) {
            candidateEdges.push_back(Edge(candidateVertex, myIndex, EM->getEdgeValue(myIndex, candidateVertex)));
         }
      }
      // sort into ascending order, so when need shortest edge, pull from begin()
      sort(candidateEdges.begin(), candidateEdges.end());
   }

};
// end of class Vertex


const Graph *Vertex::EM;   // EM: edge matrix


// class to hold and manipulate the Minimum Spanning Tree
class MST {
public:
   // ctor
   MST(const Graph &g,
       const int start) :
      edgeMatrix(g),
      first(start),
      MST_distance(0),
      countOfMissingVertices(0) {
      Vertex::EM = &g;
      initMST_Vertices();
   }

   friend ostream& operator << (ostream& out, const MST& m);

   void buildMST() {
      while (addEdge()) {}            // keep adding edges until can't anymore
   }

   int getDistance() const {return MST_distance;}

   int verifyMST() {
      verifyAllVerticesIncluded();
      return countOfMissingVertices;
   }

private:
   int MST_distance;
   const int first;
   const Graph edgeMatrix;
   vector<Vertex> MST_Vertices;
   vector<Edge> edges;           // each entry is one edge in the MST
   int countOfMissingVertices;   // need this for the ostream friend 

   void initMST_Vertices() {
      MST_Vertices.push_back(Vertex(first, MST_Vertices));
   }

   // Look for an available edge, if find one, add that edge to the MST.
   bool addEdge() {
      bool foundOne = false;
      Edge shortestEdge(0,0,0);
      // walk through the MST to find the shortest next edge
      for (Vertex vertex : MST_Vertices) {
         if (!vertex.hasCandidates()) {
            continue;
         }
         Edge candidateEdge = vertex.getShortestCandidate();
	 
         // first, if haven't found one yet, take this first one,
         // OR if have found one prior, 
         // compare new to shortest distance so far
         if (!foundOne ||
	     candidateEdge < shortestEdge) {
            foundOne = true;
	    shortestEdge = candidateEdge;
         }
      }
      // now add new edge to the tree
      if (foundOne) {
         addToTree(shortestEdge);
      }
      return foundOne;
   }

   // add this edge to the MST
   void addToTree(const Edge edge) {
      MST_Vertices.push_back(Vertex(edge.v1(), MST_Vertices));   // add new vertex to MST
      prune(edge.v1());                                          // remove new vertext from candiates vectors
      edges.push_back(edge);                                     // add edge to the MST edge vector
      MST_distance += edge.dist();
   }

   // remove vertex as a candidate from all vertices (not just the current vertex)
   // different vertices have different edges that contain this newlyAdded Vertex, have to remove from all
   // candidates list so no duplicates or loops.
   void prune(const int newlyAddedVertex) {
      for (Vertex &vertex : MST_Vertices) {
         vertex.removeCandidate(newlyAddedVertex);
      }
   }

   // this checks to see that all vertices are included in the MST
   void verifyAllVerticesIncluded() {
      vector<bool> included(edgeMatrix.getVertexCount(), false);

     // first, set the ones in the edges to true
     included[first]=true;
     for (vector<Edge>::iterator e = edges.begin(); e < edges.end(); ++e) {
       if (included[e->v1()]) {
	 // been here before which means vertex is an ending vertex twice
	 ++countOfMissingVertices;    // this is a hack
       }
       included[e->v1()] = true;
     }

     // second make sure all are true
     for (vector<bool>::iterator b = included.begin(); b < included.end(); ++b) {
        if (*b == false)
           ++countOfMissingVertices;
     }
     return;
  }

};
// end class MST

// function to print out the MST
ostream& operator << (ostream& out, const MST& m) {
  out << endl;
   if (m.countOfMissingVertices == 0) {
      out << "All vertices included\n";
   }
   else {
     out << "Missing " << m.countOfMissingVertices << "  vertices.\n";
   }

   out << "MST distance is " << m.MST_distance << endl;
   out << "start vertex is " << m.first << endl;

   for (Edge edge : m.edges) {
      out << "Between vertex "      << setfill(' ') << setw(2) << edge.v2()
          << " and vertex "         << setfill(' ') << setw(2) << edge.v1()
          << "   edge distance is " << setfill(' ') << setw(2) << edge.dist() << endl;
   }
   return out;
}

// class to build the Minimum Spanning Tree
class MST_Controller {
public:
   MST_Controller(const Graph &g,
		  const int initVertex = 0) :
      tree(MST(g, initVertex)) {}

   friend ostream& operator << (ostream& out, const MST_Controller& mc);

   bool buildMST() {
      // keep adding nodes until done or can't add more

      tree.buildMST();

      return(tree.verifyMST() == 0);
   }

  int getDistance() const { return tree.getDistance(); }

private:
  MST tree;

};
// end class MST_Controller

ostream& operator << (ostream& out, const MST_Controller& mc) {
   out << mc.tree;
   return out;
}


int main() {
   //Graph* g1 = new Graph(10);                // create random graph using last assignment
   Graph* g2 = new Graph("sampleData.txt");

   for (Graph* g : { /* g1, */ g2} ) {
     cout << *g;
     int distance;
     bool good = true;

     // run through using each vertex as a starting point (assignment didn't require this)
     // if get different values, probably something wrong
     for (int startVertex = 0; startVertex < g->getVertexCount(); ++startVertex) {
       MST_Controller *c = new MST_Controller(*g, startVertex);

       // now track history some, and see if all distances are same, and all MST are good.
       if (startVertex == 0) {
	 good = c->buildMST();
	 cout << *c;
	 cout << "(check with remaining " << g->getVertexCount()-1 << " vertices as start vertex) \n";
	 distance = c->getDistance();
       }
       else if (!c->buildMST() ||
		c->getDistance() != distance) {
	 good = false;
	 cout << "\n*** mismatch *** ";
       }
       delete c;
     }
     if (!good) {
       cout << "Distances don't match or vertices missing (or duplicated), something maybe not correct.\n";
       cout << "(if density is low, nodes can have no edges, then this happens and is expected.)\n";
     }
     else {
       cout << "Distance is same with every vertex as starting vertex and all vertices included.\n";
       cout << "(edges used or order could be different with differing start vertices)\n";
     }
     delete g;
   }

   return 0;
}


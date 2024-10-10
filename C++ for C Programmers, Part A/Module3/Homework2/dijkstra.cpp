#include "shortest_path.h"
#include "graph_example.h"
#include "graph_wikipedia.h"
#include "graph_random.h"

using namespace std;

void testShortestPath(Graph& g, Node start, Node dest);
void testRandomGraph(Graph& g);
double calcAveragePathLength(Graph &g);

int main()
{
  Graph gExample{createExampleGraph()};
  testShortestPath(gExample, NodeEnum::S, NodeEnum::T);

  Graph gWiki{createWikipediaGraph()};
  testShortestPath(gWiki, NodeEnum::A, NodeEnum::E);

  Graph gRandom1{createRandomGraph(50, 0.2, 1.0, 10.0)};
  testRandomGraph(gRandom1);

  Graph gRandom2{createRandomGraph(50, 0.4, 1.0, 10.0)};
  testRandomGraph(gRandom2);

  return 0;
}

void testShortestPath(Graph& g, Node start, Node dest) {
  ShortestPath sp(g);
  auto path = sp.path(start, dest);
  cout << "shortest path = " << path << endl;
  Distance distance = sp.pathSize(start, dest);
  cout << "distance = " << distance << endl;
}

void testRandomGraph(Graph& g)
{
  cout << "Average path length = " << setprecision(3) << calcAveragePathLength(g) << endl;
}

double calcAveragePathLength(Graph &g)
{
  ShortestPath sp(g);
  Distance sum{0};
  size_t count{0};
  for (size_t i{1}; i < g.getNumVertices(); ++i)
  {
    Distance d = sp.pathSize(Node(0), Node(i));
    if (d != 0)
    {
      sum += d;
      ++count;
    }
  }
  return double(sum) / double(count);
}

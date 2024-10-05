#ifndef PRIORITY_QUEUE_ELEMENT_H
#define PRIORITY_QUEUE_ELEMENT_H

#include "node.h"
#include "distance.h"

class PriorityQueueElement
{
  friend class PriorityQueue;

public:
  PriorityQueueElement(Node n, Distance d) : node{n}, d{d} {}
  Node getNode() { return node; }
  Distance getDistance() { return d; }

  bool operator<(const PriorityQueueElement &other) const { return d < other.d; }

private:
  Node node;
  Distance d;
};

#endif

#ifndef PRIORITY_QUEUE_ELEMENT_H
#define PRIORITY_QUEUE_ELEMENT_H

#include "node.h"
#include "distance.h"

class PriorityQueueElement
{
  friend class PriorityQueue;

public:
  PriorityQueueElement(Node n, Distance d) : node{n}, value{d} {}
  Node getNode() { return node; }
  Distance getValue() { return value; }

  bool operator<(const PriorityQueueElement &other) const { return value < other.value; }

private:
  Node node;
  Distance value;
};

#endif

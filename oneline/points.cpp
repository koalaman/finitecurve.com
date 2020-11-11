#include "points.h"
#include <climits>
#include <random>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <functional>
#include <stdint.h>


using namespace oneline;

namespace {
  int64_t joinInts(int a, int b) {
    assert(a < 0x7FFFFFFF && b < 0x7FFFFFFF);
    return ((int64_t)a)<<32 | b;
  }
  std::pair<int, int> splitInt(int64_t val) {
    int b = val & 0xFFFFFFFF;
    int a = (val >> 32) & 0xFFFFFFFF;
    return std::pair<int, int>(a,b);
  }
  template <typename T> void clearZerosFromMap(std::unordered_map<T, int>& map) {
    auto it = map.begin();
    while (it != map.end()) {
      if(it->second == 0) {
	it = map.erase(it);
      } else {
	it++;
      }
    }
  }

  template <typename K, typename V> K getMaxValueKey(const std::unordered_map<K, V>& map, const K def) {
    bool found = false;
    V max;
    K key = def;
    for(auto& kv : map) {
      if (!found || kv.second > max) {
	found = true;
	key = kv.first;
	max = kv.second;
      }
    }
    return key;
  }

#ifndef NDEBUG
  void countColors(const std::vector<Point>& points, std::unordered_map<Color, int>& map) {
    map.clear();
    for(const Point& p : points) {
      if(p.usuallyExcluded()) continue;
      auto it = map.find(p.color);
      if (it == map.end()) {
	map[p.color] = 1;
      } else {
	map[p.color]+=1;
      }
    }
  }

  bool compareMap(const std::unordered_map<Color, int>& a, const  std::unordered_map<Color, int>& b, bool x = true) {
    if(x) std::cerr<<"Map comparison starting\n";
    bool any = false;
    for(auto pair : a) {
      auto it = b.find(pair.first);
      if(it == b.end()) {
	std::cerr<< "Missing " << pair.first << "\n";
	any=true;
      } else if(it->second != pair.second) {
	std::cerr<< "Wrong value for " << pair.first << ": " << pair.second << " vs " << it->second << "\n";
	any=true;
      }
    }
    if(x) return compareMap(b, a, false);
    else std::cerr<<"Map comparison done\n";
    return any;
  }

  void verifyCounts(const std::vector<Point>& points, const std::unordered_map<Color, int>& map) {
    std::unordered_map<Color, int> realMap;
    countColors(points, realMap);
    assert(!compareMap(realMap, map));
  }
#else
  void verifyCounts(const std::vector<Point>& points, const std::unordered_map<Color, int>& map) { /* Nada */ }
#endif

}

namespace {
int orientation(const Point& p, const Point& q, const Point& r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

bool onSegment(const Point& p, const Point& q, const Point& r)
{
    return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
}

}

bool Point::wouldCross(const Point& p1, const Point& q1, const Point& p2, const Point& q2) {
  if (p1.isOn(p2) || p1.isOn(q2) || q1.isOn(p2) || q1.isOn(q2)) return false;

  // Find the four orientations needed for general and special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Points are colinear
  if (o1 == 0 && onSegment(p1, p2, q1)) return true;
  if (o2 == 0 && onSegment(p1, q2, q1)) return true;
  if (o3 == 0 && onSegment(p2, p1, q2)) return true;
  if (o4 == 0 && onSegment(p2, q1, q2)) return true;

  return false;
}


#ifndef NDEBUG
void Points::sanityCheckImpl() {
  assert(empty.empty());

  if (startId != NO_ID && hasPath) {
    assert(points[startId].isPartOfPath);
  }
  if(endId != NO_ID && hasPath) {
    assert(points[endId].isPartOfPath);
  }

  std::unordered_set<PointID> path;
  PointID current = startId;
  while (current != NO_ID && hasPath) {
    assert(points[current].isPartOfPath);
    auto res = path.insert(current);
    assert(res.second);
    int nextId = points[current].next;
    if(nextId != NO_ID) {
      points[current].hasNeighbor(nextId);
      points[nextId].hasNeighbor(current);
    }
    current = nextId;
  }

  for(Point& point : points) {
    assert(!point.isPartOfPath || !point.isIgnored);
  }

  for(int i = 0, e = points.size(); i<e; i++) {
    const struct Point& point = points[i];
    if (path.find(i) != path.end()) continue;
    assert(point.next == NO_ID);
    assert(!point.isPartOfPath);

    const Bucket& bucket = getBucketFor(point.x, point.y);
    bool found = false;
    for(PointID id : bucket) {
      if(id == i) found = true;
    }
    assert(found);
  }

  for (const Point& p : points) {
    for (PointID neighbor : p.neighbors) {
      assert(points[neighbor].hasNeighbor(getId(p)) && "Graph must be undirected");
    }
  }
}
#endif

void Points::setEndPoints() {
  const Point* leftmost = &points[0];
  const Point* rightmost = &points[1];

  for (const auto& point : this->points) {
    if (point.x < leftmost->x) leftmost = &point;
    if (point.x > rightmost->x) rightmost = &point;
  }
  setEndPoints(getId(leftmost), getId(rightmost));
}

void Points::outputGrid(std::basic_ostream<char>& out) {
  for (const auto& point : this->points) {
    for (const auto& n : point.neighbors) {
      const auto& neighbor = points[n];
      out << point.x << " " << point.y << "  # " << getId(point) << "\n";
      out << neighbor.x << " " << neighbor.y << "  # " << getId(neighbor) << "\n";
      out << "\n";
    }
  }
}

void Points::outputGrid(const std::string& out) {
  std::ofstream file;
  file.open(out);
  outputGrid(file);
  file.close();
}

void Points::outputPath(std::basic_ostream<char>& out) {
  PointID current = startId;
  while(current != NO_ID) {
    const Point& point = points[current];
    out << point.x << " " << point.y << " # pointId=" << getId(point) << "\n";
    current = point.next;
  }
}

void Points::outputPath(const std::string& out) {
  std::ofstream file;
  file.open(out);
  outputPath(file);
  file.close();
}

void Points::outputPoints(std::basic_ostream<char>& out) {
  for(auto& point : points) {
    out << point.x << " " << point.y << "\n";
  }
}

void Points::outputPoints(const std::string& out) {
  std::ofstream file;
  file.open(out);
  outputPoints(file);
  file.close();
}

void Points::outputLabels(std::basic_ostream<char>& out) {
  for(size_t i=0; i<points.size(); i++) {
    const Point& p = points[i];
    out << "set label \"" << i << " " << p.color << ((i == (size_t)startId || i == (size_t)endId) ? "*" : "") <<  "\""
	<< " at " << p.x << "," << p.y << "\n";
  }
}
void Points::outputLabels(const std::string& out) {
  std::ofstream file;
  file.open(out);
  outputLabels(file);
  file.close();
}

void Points::fillRandom(const Image& image) {
  std::srand(config.seed);

  if(config.verbose) {
    std::cerr << "Filling points...\n";
  }
  for(int y=0; y<config.height; y++) {
    for(int x=0; x<config.width; x++) {
      int shade = image.getShade(x,y);
      if(shade > config.whiteCutoff) continue;

      int d = shade *
	(config.pointDensityWhite-config.pointDensity)/255
	+ config.pointDensity;

      if (!hasPointsWithin(x, y, d)) {
	Point& p = addPoint(x, y);
	p.reach = d * config.neighborhood / config.pointDensity;
	assert(p.reach < config.maxReach);
	x += d-1;
      }
    }
  }
  if(config.verbose) {
    std::cerr << "Done\n";
  }
}

void Points::fillRandom() {
  std::srand(config.seed);

  int density = config.pointDensity;
  int randomPointPercent = config.randomPointPercent;
  int randomPoints = randomPointPercent * config.width*config.height/(density*density)/100;

  for (int p=0; p<randomPoints; p++) {
    int i=0;
    for(i=100; i>0; i--) {
      int x = std::rand()%config.width;
      int y = std::rand()%config.height;
      if (!hasPointsWithin(x, y, density)) {
	Point& p = addPoint(x, y);
	p.reach = config.neighborhood;
	break;
      }
    }
    if (i == 0) {
      std::cerr << "Failed adding random point\n";
      break;
    }
  }

  for(int y=0; y<config.height; y++) {
    for(int x=rand()%density; x<config.width; x++) {
      if (!hasPointsWithin(x, y, density)) {
	Point& p = addPoint(x, y);
	p.reach = config.neighborhood;
	x+=density-1;
      }
    }
  }
  sanityCheck();
}

void Points::connectClosePoints() {
  std::vector<std::pair<int, PointID>> candidates;
  for (Point& p : points) {
    candidates.clear();

    for (Point& neighbor : findPointsWithin(p.x, p.y, p.reach)) {
      if (p.isOn(neighbor) || p.hasNeighbor(getId(neighbor))) continue;
      candidates.emplace_back(p.monotonicDistanceTo(neighbor), getId(neighbor));
    }

    std::sort(candidates.begin(), candidates.end());
    for(const auto& pair : candidates) {
      Point& neighbor = points[pair.second];
      if ((int) p.neighbors.size() >= config.maxNeighbors) break;
      if (!wouldCrossAny(p, neighbor)) {
        connectNeighbors(p, neighbor, 0);
      }
    }
  }
}

void Points::makeGrid() {
  if(config.verbose) {
    std::cerr << "Making grid...\n";
  }
  connectClosePoints();
  pruneSimilarAngles();
  trimShortHairs(config.trimHairsShorterThan);
  shuffleNeighbors();
  if(config.verbose) {
    std::cerr << "Done\n";
  }
}

void Points::trimShortHairs(int maxLengthToTrim) {
  std::vector<PointID> hair;
  bool mayTrim;
  hair.reserve(maxLengthToTrim+1);

  for (Point& point : points) {
    if (point.neighbors.size() != 1) continue;

    PointID previous = NO_ID;
    PointID current = getId(point);
    mayTrim = true;
    hair.clear();

    do {
      hair.push_back(current);
      if (current == startId || current == endId || hair.size() > (unsigned int) maxLengthToTrim) {
	mayTrim = false;
	break;
      }

      PointID next = NO_ID;
      for (PointID c : points[current].neighbors) {
	if (c == previous) continue;
	next = c;
	break;
      }
      previous = current;
      current = next;
    } while (current != NO_ID && points[current].neighbors.size() <= 2);

    if (mayTrim) {
      for (PointID piece : hair) {
	for (PointID neighbor : points[piece].neighbors) {
	  points[neighbor].removeNeighbor(piece);
	}
	points[piece].neighbors.clear();
      }
    }
  }
}

void Points::pruneSimilarAngles() {
  std::vector<std::pair<float, std::pair<int, PointID>>> angles;
  for (Point& point : points) {
    if (point.neighbors.size() <= 1) continue;

    angles.clear();
    for (PointID n : point.neighbors) {
      Point& neighbor = points[n];
      float f = point.radiansTo(neighbor);
      int distance = point.monotonicDistanceTo(neighbor);
      angles.emplace_back(f, std::make_pair(distance, n));
    }

    std::sort(angles.begin(), angles.end());
    // Wrap lowest one around
    auto lowest = angles[0];
    lowest.first += 2*3.1415;
    angles.push_back(lowest);

    for (int i=1, e=angles.size(); i<e; i++) {
      float diff = angles[i].first - angles[i-1].first;
      assert(diff >= 0);
      if (diff < config.minRadianDifference) {
	auto distPointA = angles[i].second;
	auto distPointB = angles[i-1].second;
	if (distPointA.first > distPointB.first) {
	  disconnectNeighbors(point, points[distPointA.second]);
	} else {
	  disconnectNeighbors(point, points[distPointB.second]);
	}
      }
    }
  }
}

bool Points::wouldCrossAnyLong(const Point& from, const Point& to) {
  int d2 = from.monotonicDistanceTo(to);
  if(d2 < Point::toMonotonicDistance(config.maxReach)) {
    return wouldCrossAny(from, to);
  }

  if(wouldCrossLonghauls(from, to)) return true;

  int dx = to.x - from.x;
  int dy = to.y - from.y;
  int n = (int)ceil(sqrt(d2)/(config.maxReach-2)); // -2 to allow for 1.4 pixels off


//    std::cerr << "Distance " << from.monotonicDistanceTo(to) << " aka " << sqrt(from.monotonicDistanceTo(to)) << "\n";
//    std::cerr << "To be split into " << n << "\n";

  // Eeerk
  Point fakeFrom {from.x, from.y};
  Point fakeTo { -1, -1 };

  for(int i=1; i<=n; i++) {
    fakeTo.x = from.x + dx * i / n;
    fakeTo.y = from.y + dy * i / n;
//   std::cerr << "Measuring " << i << " of " << n << "\n";
//   std::cerr << fakeFrom.x << "," << fakeFrom.y  << "  "
//             << fakeTo.x << "," << fakeTo.y << "\n";
//   std::cerr << "Distance " << fakeFrom.monotonicDistanceTo(fakeTo) << " aka " << sqrt(fakeFrom.monotonicDistanceTo(fakeTo)) << "\n";
	
    if (wouldCrossAnyShort(fakeFrom, fakeTo)) return true;
    fakeFrom.x = fakeTo.x;
    fakeFrom.y = fakeTo.y;
  }
  return false;
}

bool Points::wouldCrossAny(const Point& from, const Point& to) {
  if(wouldCrossAnyShort(from, to)) return true;
  return wouldCrossLonghauls(from, to);
}

bool Points::wouldCrossAnyShort(const Point& from, const Point& to) {
  assert(from.monotonicDistanceTo(to) < Point::toMonotonicDistance(config.maxReach) && "Doesn't work for longhauls");
  for(Point& current : findPointsWithin(from.x, from.y, config.maxReach)) {
    for (size_t j=0, f=current.neighbors.size(); j<f; j++) {
      if(getId(current) < current.neighbors[j]) {
	// Check points that are part of the path
	if(points[current.neighbors[j]].isIgnored) continue;

	if (Point::wouldCross(from, to, current, points[current.neighbors[j]])) {
	  return true;
	}
      }
    }
  }
  return false;
}

bool Points::wouldCrossLonghauls(const Point& from, const Point& to) {
  for (PointID longhaulId : longhauls) {
    Point& longhaul = points[longhaulId];
    for(PointID neighborId : longhaul.neighbors) {
      Point& neighbor = points[neighborId];
      if (neighbor.isIgnored) continue;
      if (Point::wouldCross(from, to, longhaul, neighbor)) {
	return true;
      }
    }
  }
  return false;
}

void Points::shuffleNeighbors() {
  if(config.verbose) {
    std::cerr << "  Shuffling neighbors...\n";
  }
  auto rng = std::default_random_engine {};
  for(Point& p : points) {
    std::shuffle(p.neighbors.begin(), p.neighbors.end(), rng);
  }
}

bool Points::connect(std::vector<StackEntry>& stack, PointID start, PointID end) {
  assert(stack.size() == 0);
  stack.emplace_back(start, -2);
  while (stack.size() > 0) {
    StackEntry& current = stack.back();
    Point& point = points[current.point];

    if (current.index == -2) {
      // We should inspect the node itself
      if (current.point == end) {
	// Found the goal
	break;
      }
      if (point.visited == serial || point.usuallyExcluded()) {
	// This node was already visited or untouchable
	stack.pop_back();
	continue;
      }
      // This is a new node
      point.visited = serial;
      current.index = point.neighbors.size()-1;
    } else if(current.index == -1) {
      // We have exhausted the node without finding anything.
      stack.pop_back();
    } else {
      assert(current.index >= 0 && current.index < (int)point.neighbors.size());
      // Stack modification invalidates 'current'
      stack.emplace_back(point.neighbors[current.index--], -2);
    }
  }

  // Make sure we don't confuse nodes on next run
  serial++;

  if(stack.size() == 0) {
    // No path was found
    return false;
  }

  points[end].isPartOfPath = true;
  for(int i=1, e=stack.size(); i<e; i++) {
    Point& point = points[stack[i-1].point];
    point.isPartOfPath = true;
    point.next = stack[i].point;
  }
  stack.clear();
  return true;
}

bool Points::expandPath(std::vector<StackEntry>& stack) {
  assert(startId != NO_ID);

  bool modified = false;
  bool modifiedThisIteration;
  do {
    PointID current = startId;
    modifiedThisIteration = false;
    while (points[current].next != NO_ID) {
      PointID next = points[current].next;
      for (PointID detour : points[current].neighbors) {
	Point& detourP = points[detour];
	if (detourP.isPartOfPath) continue;
	if (connect(stack, detour, next)) {
	  points[current].next = detour;
	  modified = modifiedThisIteration = true;
	  next = detour;
	  break;
	}
      }
      current = next;
    }
  } while(modifiedThisIteration);
  return modified;
}

bool Points::invadeIslands(const std::unordered_map<Color, int> colorCount, std::vector<StackEntry>& stack) {
  PointID secondPrevious = NO_ID;
  PointID previous = NO_ID;
  PointID current = startId;

  while(current != NO_ID) {
    Point& currentPoint = points[current];
    PointID next = currentPoint.next;

    for(PointID nid : currentPoint.neighbors) {
      Point& neighbor = points[nid];
      if (neighbor.usuallyExcluded()) continue;

      // We found a bridge
      int color = neighbor.color;
      // But it was probably just a hole in the grid
      if (colorCount.find(color)->second < config.minimumGroupSize) continue;

      int prevDist = INT_MAX;
      if(previous != NO_ID && !points[previous].hasNeighbor(nid) && !wouldCrossAnyLong(points[previous], neighbor)) {
	prevDist = points[previous].monotonicDistanceTo(neighbor);
      }
      int nextDist = INT_MAX;
      if(next != NO_ID && !points[next].hasNeighbor(nid) && !wouldCrossAnyLong(points[next], neighbor)) {
	nextDist = points[next].monotonicDistanceTo(neighbor);
      }

      if (prevDist == INT_MAX && nextDist == INT_MAX) {
	// Neither sibling can be connected. Try to sacrifice the next/previous and
	// connect via a grandparent/child. We should only ever have to one extra step,
	// because otherwise we'd realistically have a path to expand to bring us back
	// to 1 or 2 steps.
	PointID secondNext = next == NO_ID ? NO_ID : points[next].next;
	PointID from, excluding, to, needsConnect;

	if (secondPrevious != NO_ID && (points[secondPrevious].hasNeighbor(nid) || !wouldCrossAnyLong(points[secondPrevious], neighbor))) {
	  needsConnect = secondPrevious;
	  from = secondPrevious;
	  excluding = previous;
	  to = current;
	} else if(secondNext != NO_ID && (points[secondNext].hasNeighbor(nid) || !wouldCrossAnyLong(points[secondNext], neighbor))) {
	  needsConnect = secondNext;
	  from = current;
	  excluding = next;
	  to = secondNext;
	} else {
	  if (config.verbose) {
	    std::cerr << "Couldn't even link grandparents to island size " << colorCount.find(color)->second << "\n";
	  }
	  continue;
	}

	if (!points[needsConnect].hasNeighbor(nid)) {
	  connectNeighbors(points[needsConnect], neighbor, points[needsConnect].monotonicDistanceTo(neighbor));
	}
	points[from].next = nid;
	neighbor.next = to;
	neighbor.isPartOfPath = true;

	Point& ex = points[excluding];
	ex.isPartOfPath = false;
	ex.next = NO_ID;

	sanityCheck();
	return true;
      }

      if (prevDist < nextDist) {
	// Go previous -> there -> this
	if (config.verbose) {
	  std::cerr << "We are on " << current << "\n";
	  std::cerr << "Prevving " << previous << " to " << nid << "\n";
	}
	connectNeighbors(points[previous], neighbor, prevDist);
	points[previous].next = nid;
	neighbor.next = current;
	neighbor.isPartOfPath = true;
	// We don't know about counts anymore, so return and re-expand
	sanityCheck();
      } else {
	// Go this -> there -> next
	if (config.verbose) {
	  std::cerr << "We are on " << current << "\n";
	  std::cerr << "Nexting " << nid << " to " << next << "\n";
	}
	connectNeighbors(neighbor, points[next], nextDist);
	currentPoint.next = nid;
	neighbor.next = next;
	neighbor.isPartOfPath = true;
      }
      sanityCheck();
      return true;
    }

    secondPrevious = previous;
    previous = current;
    current = next;
  }
  return false;
}

bool Points::fillHoles(std::vector<StackEntry>& stack) {
  std::unordered_map<PointID, std::pair<PointID, int>> lastSeen;
  std::unordered_set<PointID> touched;
  int iteration = 1;
  const int maxReach = 4;
  PointID currentId = startId;
  bool changed = false;

  while (currentId != NO_ID) {
    Point& current = points[currentId];

    for (PointID n : current.neighbors) {
      if (points[n].isPartOfPath) continue;

      auto it = lastSeen.find(n);
      if (it == lastSeen.end()) {
	lastSeen[n] = std::make_pair(currentId, iteration);
      } else {
	int distance = iteration - it->second.second;
	if (distance <= maxReach && touched.find(it->second.first) == touched.end()) {
	  // Erase previous
	  {
	    PointID last = it->second.first;
	    while(last != currentId) {
	      touched.insert(last);
	      Point& lastP = points[last];
	      assert(lastP.isPartOfPath);
	      last = lastP.next;
	      lastP.next = NO_ID;
	      lastP.isPartOfPath = false;
	    }
	  }

	  // Add new
	  {
	    PointID last = it->second.first;
	    Point& lastP = points[last];
	    lastP.next = n;
	    lastP.isPartOfPath = true;
	    points[n].next = currentId;
	    points[n].isPartOfPath = true;
	  }

	  sanityCheck();

	  changed = true;
	}
	lastSeen.erase(n);
      }
    }
    currentId = current.next;
    iteration++;
  }

  sanityCheck();

  if (changed) {
    changed = expandPath(stack);
    return changed;
  } else {
    return false;
  }
}

int Points::getPathLength() {
  int size = 0;
  PointID current = startId;
  while(current != NO_ID) {
    current = points[current].next;
    size++;
  }
  return size;
}

float Points::getPathDistance() {
  float length = 0;
  PointID current = startId;
  PointID next = current == NO_ID ? NO_ID : points[current].next;
  while(next != NO_ID) {
    const Point& a = points[current];
    const Point& b = points[next];
    int xd = b.x - a.x;
    int yd = b.y - a.y;
    length += sqrt(xd*xd + yd*yd);
    current = next;
    next = b.next;
  }
  return length;
}

void Points::dumpPoints() {
  for(size_t i=0; i<points.size(); i++) {
    std::cerr
      << (points[i].usuallyExcluded() ? "// " : "   ")
      << (points[i].isPartOfPath ? "*" : " ") << " "
      << i
      << "(color " << points[i].color << ")"
      << ": ";
    for(PointID neighbor : points[i].neighbors) {
      std::cerr << neighbor << " ";
    }
    std::cerr << "=> " << points[i].next;
    std::cerr << "\n";
  }
  std::cerr <<"\n\n";
}

void Points::outputSVG(const std::string& out) {
  std::ofstream file;
  file.open(out);
  outputSVG(file);
  file.close();
}

void Points::resetColor() {
  for (Point& point : points) {
    point.color = 0;
  }
}

int Points::colorGroup(std::vector<StackEntry>& stack, PointID from, Color color) {
  assert(color != 0);
  assert(stack.size() == 0);
  stack.emplace_back(from, 0);
  int count = 0;
  while(stack.size()) {
    StackEntry entry = stack.back();
    stack.pop_back();
    Point& point = points[entry.point];
    if (point.color != 0 || point.usuallyExcluded()) continue;
    point.color = color;
    count++;
    for (PointID neighbor : point.neighbors) {
      stack.emplace_back(neighbor, 0);
    }
  }
  return count;
}

void Points::ignoreColorsBelow(std::unordered_map<Color, int>& counts, int groupSize) {
  // Protect start and end groups
  int startColor = startId == -1 ? -1 : points[startId].color;
  int endColor = endId == -1 ? -1 : points[endId].color;

  for(Point& p : points) {
    if (p.color == startColor || p.color == endColor) continue;
    auto res = counts.find(p.color);
    if (res == counts.end()) continue;
    if (res->second >= groupSize) continue;
    p.isIgnored = true;
    res->second--;
  }
  clearZerosFromMap(counts);
  sanityCheck();
}

void Points::findCrossColorEdges(int fromColor, EdgeMaxQueue& output) {
  unsigned maxEdges = (unsigned) config.edgeQueueSize;
  assert(maxEdges > 0);
  assert(output.size() == 0);

  int pointSize = points.size();
  for (int i=0; i<pointSize; i++) {
    Point& ip = points[i];
    if (ip.color != fromColor || ip.usuallyExcluded()) continue;

    for (int j=0; j<pointSize; j++) {
      Point& jp = points[j];
      if(jp.color == fromColor || jp.usuallyExcluded()) continue;

      int distance = ip.monotonicDistanceTo(jp);
      if(output.size() < maxEdges) {
	output.emplace(distance, i, j);
      } else if(output.top().distance > distance) {
	output.pop();
	output.emplace(distance, i, j);
      }
    }
  }
}

void Points::findMinimalCrossColorEdges(std::unordered_map<int64_t, Edge>& map, int searchDistance, int largestColor) {
  int pointSize = points.size();
  for (int i=0; i<pointSize; i++) {
    Point& p1 = points[i];
    if(p1.usuallyExcluded()) continue;
    if(p1.color == largestColor) continue;

    for(Point& p2 : findPointsWithin(p1.x, p1.y, searchDistance)) {
      //Point& p2 = points[j];
      int j = getId(p2);
      if(p2.usuallyExcluded()) continue;
      if(p2.color != largestColor && p1.color >= p2.color) continue;

      int64_t key = joinInts(p1.color, p2.color);

      auto it = map.find(key);
      int distance = p1.monotonicDistanceTo(p2);
      if(it == map.end() || it->second.distance > distance) {
	if (!wouldCrossAnyLong(p1, p2)) {
	  if(it == map.end()) {
	    map.insert_or_assign(key, Edge(distance, i, j));
	  } else {
	    it->second = Edge(distance, i, j);
	  }
	}
      }
    }
  }
}

void Points::connectColors(std::unordered_map<Color, int>& colorCounts) {
  int searchRadius = config.maxReach*2;
  std::unordered_map<Color, int> realCounts;

  // Do a coarse pass
  connectColors(colorCounts, searchRadius);
  verifyCounts(points, colorCounts);

  // Connect tiny groups that tend to exceed the search radius
  // TODO: Measure impact
  heroicallyConnect(colorCounts, config.minimumGroupSize);
  verifyCounts(points, colorCounts);

  while(colorCounts.size() > 1) {
    assert(searchRadius < (config.width+config.height)*2);
    searchRadius*=2;
    connectColors(colorCounts, searchRadius);
  }
  verifyCounts(points, colorCounts);
}

void Points::heroicallyConnect(std::unordered_map<Color, int>& colorCounts, int maxGroupSize) {
  std::unordered_map<Color, Edge> minEdges;
  std::unordered_map<Color, std::unique_ptr<std::vector<PointID>>> instances;

  for(Point& p : points) {
    if (p.usuallyExcluded()) continue;
    if (colorCounts.find(p.color)->second > maxGroupSize) continue;

    {
      auto it = instances.find(p.color);
      if (it == instances.end()) {
	auto ptr = std::make_unique<std::vector<PointID>>();
	ptr->push_back(getId(p));
	instances[p.color] = std::move(ptr);
      } else {
	it->second->push_back(getId(p));
      }
    }

    for(Point& p2 : points) {
      if(p2.color == p.color || p2.usuallyExcluded()) continue;
      int dist = p.monotonicDistanceTo(p2);

      auto it = minEdges.find(p.color);
      if (it != minEdges.end() && it->second.distance <= dist) continue;
      if(wouldCrossAnyLong(p, p2)) continue;

      if (it == minEdges.end()) {
	minEdges.emplace(p.color, Edge(dist, getId(p), getId(p2)));
      } else {
	it->second.distance = dist;
	it->second.from = getId(p);
	it->second.to = getId(p2);
      }
    }
  }

  std::vector<Edge> sortedEdges;
  sortedEdges.reserve(minEdges.size());
  for (auto pair : minEdges) {
    sortedEdges.push_back(pair.second);
  }
  std::sort(sortedEdges.begin(), sortedEdges.end());

  for(auto edge : sortedEdges) {
    Point& from = points[edge.from];
    Point& to = points[edge.to];
    if (from.color == to.color) continue;
    auto fromIt = instances.find(from.color);
    // We already connected this
    if (fromIt == instances.end()) continue;
    if (wouldCrossAnyLong(from, to)) continue;
    connectNeighbors(from, to, edge.distance);

    int fromColor = from.color;
    if (config.verbose)
      std::cerr << "Eating " << fromColor << "\n";
    int toColor = to.color;
    auto toIt = instances.find(toColor);
    auto* toVector = toIt == instances.end() ? nullptr : toIt->second.get();

    for(PointID affected : *fromIt->second) {
      if (config.verbose)
	std::cerr << "Recoloring " << affected << " from " << points[affected].color << "\n";
      points[affected].color = toColor;
      if (toVector) toVector->push_back(affected);
    }

    auto colorIt = colorCounts.find(fromColor);
    assert(colorIt != colorCounts.end() && colorIt->second == (int) fromIt->second->size());
    if (config.verbose)
      std::cerr << "Adding " << fromIt->second->size() << "\n";
    colorCounts[toColor] += fromIt->second->size();
    colorCounts.erase(colorIt);
    instances.erase(fromIt);
  }
}

int Points::connectColors(std::unordered_map<Color, int>& colorCounts, int reach) {
  std::unordered_map<int64_t, Edge> minimalEdges;
  if (config.verbose)
    std::cerr << "Finding minimal edges within " << reach << "\n";
  findMinimalCrossColorEdges(minimalEdges, reach, getMaxValueKey(colorCounts, -1));
  int newEdges = 0;

  std::vector<std::pair<int64_t, Edge>> edgeList;
  for(auto& pair : minimalEdges) {
    edgeList.push_back(pair);
  }

  if (config.verbose) {
    std::cerr << "Sorting minimal edges\n";
  }
  std::sort(edgeList.begin(), edgeList.end(),
      [](const std::pair<int64_t, Edge>& a, const std::pair<int64_t, Edge>& b) {
	return a.second.distance < b.second.distance;
	}
      );

  std::unordered_map<Color, std::unordered_set<Color>*> colorToSet;
  auto getSetFor = [&](Color c) {
    auto it = colorToSet.find(c);
    if (it == colorToSet.end()) {
      auto* set = new std::unordered_set<Color>();
      set->insert(c);
      colorToSet.insert({c, set});
      return set;
    } else {
      return it->second;
    }
  };
  if (config.verbose) {
    std::cerr << "Adding minimal edges\n";
  }
  for (auto& pair : edgeList) {
    auto colors = splitInt(pair.first);
    auto* setA = getSetFor(colors.first);
    auto* setB = getSetFor(colors.second);
    if (setA == setB) continue;

    Point& a = points[pair.second.from];
    Point& b = points[pair.second.to];
    if (wouldCrossAnyLong(a, b)) continue;

    connectNeighbors(a, b, a.monotonicDistanceTo(b));
    newEdges++;

    for(Color inB : *setB) {
      setA->insert(inB);
      colorToSet.find(inB)->second = setA;
    }
    delete setB;
  }

  if (config.verbose) std::cerr << "Recoloring minimal edges\n";
  // Recolor accordingly
  for(Point& p : points) {
    if (p.usuallyExcluded()) continue;
    int from = p.color;
    int to = *getSetFor(p.color)->begin();
    if (from == to) continue;
    colorCounts.find(from)->second--;
    colorCounts.find(to)->second++;
    assert(colorCounts.find(from)->second >= 0);
    p.color = to;
  }

  clearZerosFromMap(colorCounts);

  if (config.verbose) std::cerr << "Freeing minimal edges\n";
  while (colorToSet.size() > 0) {
    auto* set = colorToSet.begin()->second;
    for (Color c : *set) {
      auto it = colorToSet.find(c);
      assert(it != colorToSet.end());
      colorToSet.erase(it);
    }
    delete set;
  }
  return newEdges;
}

void Points::color(std::vector<StackEntry>& stack, std::unordered_map<Color, int>* counts) {
  assert(!counts || counts->size() == 0);
  resetColor();
  int color = 1;
  for (Point& point : points) {
    if (point.color != 0) continue;

    int count = colorGroup(stack, getId(point), color);
    if(counts) {
      (*counts)[color] = count;
    }
    color++;
  }
}

void Points::iterateExpansion(std::unordered_map<Color, int>& colorCount, std::vector<StackEntry>& stack) {
  int iter=1000;
  while(true) {
    if(iter-- == 0) break;
    bool change = expandPath(stack);
    colorCount.clear();
    color(stack, &colorCount);
    sanityCheck();
    change |= invadeIslands(colorCount, stack);
    sanityCheck();
    if(!change) break;
  }

  iter=4;
  while(fillHoles(stack) && iter--);
}

void Points::createLine(std::vector<StackEntry>& stack) {
  assert(stack.size() == 0);
  assert(startId != NO_ID && endId != NO_ID);
  std::unordered_map<Color, int> colorCounts;
  color(stack, &colorCounts);
  ignoreColorsBelow(colorCounts, config.minimumGroupSize);
  connectColors(colorCounts);
  verifyCounts(points, colorCounts);

  assert(points[startId].color != 0);
  assert(points[endId].color != 0);
  assert(points[startId].color == points[endId].color);
  outputLabels("labels.dat");
  outputGrid("grid.dat");
  if(!connect(stack, startId, endId)) {
    assert(false && "Same color, no path.");
  }
  hasPath = true;
  sanityCheck();
  iterateExpansion(colorCounts, stack);
}

float Points::getCoverage() {
  int path = 0;
  for (const Point& p : points) {
    if (p.isPartOfPath) path++;
  }
  return (float)path/points.size();
}

void Points::outputSVG(std::basic_ostream<char>& out) {
  assert(startId != NO_ID && endId != NO_ID);
//  out << "<svg width=" << config.width << " height=" << config.height << ">\n";
  out << "<svg viewbox='0 0 " << config.width << " " << config.height << "' "
      << "width='" << config.width << "' height='" << config.height << "' xmlns='http://www.w3.org/2000/svg'>\n";
  out << "<path stroke='black' fill='none' stroke-width='" << config.strokeWidth << "' d='\n";

  if (config.smoothPath) {
    outputSVGCubic(out);
  } else {
    outputSVGLinear(out);
  }

  out << "' />\n";
  out << "</svg>\n";
}

void Points::outputSVGLinear(std::basic_ostream<char>& out) {
  PointID current = startId;
  out << "M " << points[current].x << " " << points[current].y << " \n";
  while (points[current].next != NO_ID) {
    PointID next = points[current].next;
    out << "  L " << points[current].x << " " << points[current].y << "\n";
    current = next;
  }
}


namespace {
  static const float alpha = 0.5 / 2.0;
  float t(const Point& p1, const Point& p2) {
    return powf((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y), alpha);
  }
  // Centripetal Catmull-Rom spline interpolation to cubic bezier
  void spline(std::basic_ostream<char>& out, const Point& p0, const Point& p1, const Point& p2, const Point& p3) {
    float t0 = t(p0, p1);
    float t1 = t(p1, p2);
    float t2 = t(p2, p3);

    float eps = 0.001;
    if (t1 < eps) t1 = 1.0;
    if (t0 < eps) t0 = t1;
    if (t2 < eps) t2 = t1;

    float m1x = ((p1.x - p0.x) / t0 - (p2.x - p0.x) / (t0 + t1) + (p2.x - p1.x) / t1) * t1;
    float m1y = ((p1.y - p0.y) / t0 - (p2.y - p0.y) / (t0 + t1) + (p2.y - p1.y) / t1) * t1;
    float m2x = ((p2.x - p1.x) / t1 - (p3.x - p1.x) / (t1 + t2) + (p3.x - p2.x) / t2) * t1;
    float m2y = ((p2.y - p1.y) / t1 - (p3.y - p1.y) / (t1 + t2) + (p3.y - p2.y) / t2) * t1;

    out << p1.x + m1x/3 << "," << p1.y + m1y/3 << " ";
    out << p2.x - m2x/3 << "," << p2.y - m2y/3 << " ";
    out << p2.x << "," << p2.y << "\n";
  }
}

void Points::outputSVGCubic(std::basic_ostream<char>& out) {
  PointID previous = startId;
  PointID current = startId;
  PointID next = current == NO_ID ? NO_ID : points[current].next;
  PointID future = next == NO_ID ? NO_ID : points[next].next;

  if (future == NO_ID) {
    outputSVGLinear(out);
    return;
  }

  out << "M " << points[current].x << " " << points[current].y << " C \n";
  do {
    spline(out, points[previous], points[current], points[next], points[future]);
    previous = current;
    current = next;
    next = future;
    future = points[future].next;
  } while(future != NO_ID);
  spline(out, points[previous], points[current], points[next], points[next]);
}

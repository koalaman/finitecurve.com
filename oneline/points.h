#ifndef INCLUDE_POINTS_H
#define INCLUDE_POINTS_H

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <forward_list>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <cmath>
#include "image.h"

namespace oneline {

typedef int Serial;
typedef int Color;
typedef int PointID;
static const PointID NO_ID = -1;

struct Config {
  int seed = 0;
  int width = 640, height=480;
  int pointDensity = 10;
  int pointDensityWhite = 50;
  int whiteCutoff = 240;
  int neighborhood = 15;
  int randomPointPercent = 5;
  int maxReach = 100;
  int minimumGroupSize = 8;
  int edgeQueueSize = 1000;
  int maxNeighbors = 6;
  int trimHairsShorterThan = 3;
  float minRadianDifference = 20*3.1415*2/360;
  double strokeWidth = 2.0;
  bool verbose = false;
  bool smoothPath = true;

  static Config forSize(int w, int h) {
    Config c;
    c.width = w;
    c.height = h;
    return c;
  }
};

struct Point {
  int x;
  int y;
  Serial visited;
  Color color;
  PointID next = NO_ID;
  int reach = -1;
  bool isPartOfPath;
  bool isIgnored = false;
  std::vector<PointID> neighbors;

  Point(int x, int y) {
    this->x = x;
    this->y = y;

    isPartOfPath = 0;
    visited = 0;
  }

  static int fromMonotonicDistance(int dist) {
    return (int) std::sqrt(dist);
  }
  static int toMonotonicDistance(int dist) {
    return dist*dist;
  }

  int monotonicDistanceTo(int x, int y) const {
    int xd = x - this->x;
    int yd = y - this->y;
    return xd*xd+yd*yd;
  }

  int monotonicDistanceTo(const Point& point) const {
    return monotonicDistanceTo(point.x, point.y);
  }

  float radiansTo(const Point& point) const {
    return atan2f(point.x-x, point.y-y);
  }

  bool isOn(int x, int y) const {
    return this->x == x && this->y == y;
  }
  bool isOn(const Point& p) const {
    return isOn(p.x, p.y);
  }

  bool hasNeighbor(PointID newId) {
    for(PointID id : neighbors) {
      if (id == newId) return true;
    }
    return false;
  }
  void addNeighbor(PointID newId) {
    assert(!hasNeighbor(newId));
    neighbors.push_back(newId);
  }

  void removeNeighbor(PointID n) {
    neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), n), neighbors.end());
  }

  bool usuallyExcluded() const {
    return isIgnored || isPartOfPath;
  }

  static bool wouldCross(const Point& p1, const Point& p2, const Point& p3, const Point& p4);
};


struct StackEntry {
    PointID point;
    int index;

    StackEntry(PointID point, int index) {
      this->point = point;
      this->index = index;
    }
};

struct Edge {
  Edge(int distance, int from, int to) {
    this->distance = distance;
    this->from = from;
    this->to = to;
  }

  int distance;
  PointID from;
  PointID to;
  bool operator < (const Edge e) const {
    return distance < e.distance;
  }
  bool operator > (const Edge e) const {
    return distance > e.distance;
  }
};
typedef std::priority_queue<Edge> EdgeMaxQueue;

class Points {
#ifdef TESTFRIEND
friend class TESTFRIEND;
#endif

  typedef std::forward_list<PointID> Bucket;
private:
  Config config;
  PointID startId = NO_ID, endId = NO_ID;
  bool hasPath = false;
  Serial serial = 1;
  int resolution = 25;
  int bucketWidth, bucketHeight;
  std::vector<Bucket> buckets;
  std::vector<Point> points;
  std::vector<PointID> longhauls;
  Bucket empty;

  int bucketsForSize(int size) {
    return (size + resolution - 1) / resolution;
  }

  int bucketForCoord(int coord) {
    return coord / resolution;
  }

  void sanityCheckImpl();
  void addToBucket(Bucket& bucket, PointID id) {
    bucket.push_front(id);
  }

  void registerLonghaul(PointID from, PointID to) {
    assert(isLonghaul(points[from].monotonicDistanceTo(points[to])));
    assert (from != to);
    longhauls.push_back(from < to ? from : to);
  }
  bool isLonghaul(int distance) {
    return distance >= Point::toMonotonicDistance(config.maxReach);
  }

  void connectNeighbors(Point& a, Point& b, int dist) {
    PointID ap = getId(a);
    PointID bp = getId(b);
    a.addNeighbor(bp);
    b.addNeighbor(ap);
    if (isLonghaul(dist)) {
      registerLonghaul(ap, bp);
    }
  }

  void disconnectNeighbors(Point& a, Point& b) {
    assert(!isLonghaul(a.monotonicDistanceTo(b)));
    a.removeNeighbor(getId(b));
    b.removeNeighbor(getId(a));
  }

  void outputSVGLinear(std::basic_ostream<char>& out);
  void outputSVGCubic(std::basic_ostream<char>& out);

public:
  Points(const Config config) :
      config(config),
      bucketWidth(bucketsForSize(config.width)),
      bucketHeight(bucketsForSize(config.height)),
      buckets(bucketWidth * bucketHeight)
  {
    if (config.verbose) {
      std::cerr << "Verbose mode enabled\n";
    }
    sanityCheck();
  }

  const Config& getConfig() const {
    return config;
  }

  void setEndPoints(PointID start, PointID end) {
    startId = start;
    endId = end;
  }

  void setEndPoints();

  void shouldHavePath(bool b) {
    hasPath = b;
  }

  PointID getId(const struct Point& ref) const {
    return getId(&ref);
  }

  PointID getId(const struct Point* const ptr) const {
    uintptr_t id = (((uintptr_t) ptr) - ((uintptr_t) points.data())) / sizeof(struct Point);
    assert(id >= 0 && id < points.size());
    return id;
  }

  const std::vector<Point>& getPoints() const {
    return points;
  }

  void reservePoints(int n) {
    points.reserve(n);
  }

  Point& getPoint(PointID id) {
    return points[id];
  }

  Bucket& getBucketFor(int x, int y) {
    int xb = bucketForCoord(x);
    int yb = bucketForCoord(y);
    return getBucketByIndex(xb, yb);
  }

  Bucket& getBucketByIndex(int xb, int yb) {
    if (xb < 0 || yb < 0 || xb >= bucketWidth || yb >= bucketHeight) {
      return empty;
    }
    return buckets[yb*bucketWidth + xb];
  }

  const Bucket& getBucketByIndex(int xb, int yb) const {
    return buckets[yb*bucketWidth + xb];
  }

  void sanityCheck() {
#ifndef NDEBUG
    sanityCheckImpl();
#endif
  }

  bool connect(std::vector<StackEntry>& stack, PointID start, PointID end);
  bool expandPath(std::vector<StackEntry>& stack);
  bool invadeIslands(const std::unordered_map<Color, int> colorCount, std::vector<StackEntry>& stack);
  bool fillHoles(std::vector<StackEntry>& stack);
  void iterateExpansion(std::unordered_map<Color, int>& colorCount, std::vector<StackEntry>& stack);

  void fillRandom();
  void fillRandom(const Image& img);
  void makeGrid();
  void connectClosePoints();
  void pruneSimilarAngles();
  void trimShortHairs(int maxLength);
  bool wouldCrossAny(const Point& from, const Point& to);
  bool wouldCrossAnyLong(const Point& from, const Point& to);
  bool wouldCrossAnyShort(const Point& from, const Point& to);
  bool wouldCrossLonghauls(const Point& from, const Point& to);
  void shuffleNeighbors();
  void resetColor();
  void color(std::vector<StackEntry>& stack, std::unordered_map<Color,int>* counts);
  int colorGroup(std::vector<StackEntry>& stack, PointID from, Color color);
  void ignoreColorsBelow(std::unordered_map<Color, int>& counts, int groupSize);

  void createLine(std::vector<StackEntry>& stack);

  void findCrossColorEdges(int fromColor, EdgeMaxQueue& output);
  void findMinimalCrossColorEdges(std::unordered_map<int64_t, Edge>& map, int searchDistance, int maxGroup);
  void consumeColor(int fromColor, int toColor, std::vector<Edge>& previousEdges, EdgeMaxQueue& queue);
  void connectColors(std::unordered_map<Color, int>& colorCounts);
  int connectColors(std::unordered_map<Color, int>& colorCounts, int reach);
  void heroicallyConnect(std::unordered_map<Color, int>& colorCounts, int color);

  int getPathLength();
  float getCoverage();
  float getPathDistance();
  void outputGrid(const std::string& path);
  void outputGrid(std::basic_ostream<char>& out);
  void outputPath(const std::string& path);
  void outputPath(std::basic_ostream<char>& out);
  void outputPoints(const std::string& path);
  void outputPoints(std::basic_ostream<char>& out);
  void outputLabels(const std::string& path);
  void outputLabels(std::basic_ostream<char>& out);
  void outputSVG(const std::string& path);
  void outputSVG(std::basic_ostream<char>& out);
  void dumpPoints();


  template<class...Args> Point& addPoint(Args&&... args) {
    points.emplace_back(std::forward<Args>(args)...);
    Point& point = points.back();
    point.reach = config.neighborhood;
    PointID id = points.size()-1;
    assert(point.x >= 0 && point.x < config.width);
    assert(point.y >= 0 && point.y < config.height);
    Bucket& bucket = getBucketFor(point.x, point.y);
    addToBucket(bucket, id);
    return point;
  }

  template<class...Args> PointID addPointAsId(Args&&... args) {
    addPoint(args...);
    return points.size()-1;
  }

  struct search;

  // I don't actually know C++ :(
  struct iter {
    private:
    Points& points;
    int monotonicDistance;
    int range;
    int x, y;
    int xb, yb;
    int xbd, ybd;
    Bucket::iterator it, itend;
    bool done;

    iter(Points& p, int x, int y, int dist) :
      points(p),
      monotonicDistance(Point::toMonotonicDistance(dist)),
      range(p.bucketsForSize(dist)),
      x(x), y(y),
      xb(points.bucketForCoord(x)), yb(points.bucketForCoord(y)),
      xbd(-range-1), ybd(-range),
      done(false) {
	it = points.empty.begin();
	itend = points.empty.end();
	++(*this);
      }

    iter(Points& p) : points(p), done(true) {
    }

    bool matches() const {
      if (it == itend) return false;
      PointID current = *it;
      const Point& point = points.points[current];
      if (point.isIgnored) return false;
      return point.monotonicDistanceTo(x,y) < monotonicDistance;
    }
    bool next() {
      if(done) {
	return false;
      }

      if (it != itend) {
	it++;
	return true;
      }

      xbd++;
      if (xbd == range+1) {
	xbd = -range;
	ybd++;
      }
      if(ybd == range+1) {
	done = true;
	return false;
      }
      Bucket& bucket = points.getBucketByIndex(xb+xbd, yb+ybd);
      it = bucket.begin();
      itend = bucket.end();
      return true;
    }

    public:
    bool operator==(const iter& it) const {
      return it.done == done || it.it == this->it;
    }
    bool operator!=(const iter& it) const {
      return !(it == *this);
    }

    iter& operator++() {
      while (next() && !matches());
      return *this;
    }

    Point& operator*() const {
      return points.points[*it];
    }

    friend struct search;
  };

  struct search {
    private:
    Points& points;
    int x, y, dist;

    search(Points& p, int x, int y, int dist) :
      points(p), x(x), y(y), dist(dist) {}

    public:
    iter begin() {
      return iter(points, x, y, dist);
    }
    iter end() {
      return iter(points);
    }

    friend class Points;
  };

  search findPointsWithin(int x, int y, int distance) {
    return search(*this, x, y, distance);
  }

  bool hasPointsWithin(int x, int y, int distance) {
    auto s = findPointsWithin(x, y, distance);
    return s.begin() != s.end();
  }
};

}
#endif

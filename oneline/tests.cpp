class PointsTest;
#define TESTFRIEND ::PointsTest

#include "points.h"
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>

using namespace oneline;

class PointsTest : public CppUnit::TestFixture
{
  CPPUNIT_TEST_SUITE(PointsTest);
  CPPUNIT_TEST( testDistance );
  CPPUNIT_TEST( testIdentifySpace );
  CPPUNIT_TEST( testIterator );
  CPPUNIT_TEST( testConnect );
  CPPUNIT_TEST( testExpand );
  CPPUNIT_TEST( testWouldCrossAnyLong );
  CPPUNIT_TEST( testWouldCrossColinear );
  CPPUNIT_TEST_SUITE_END();

  public:
  void testDistance() {
    int threshold = 20;
    int dist = Point::toMonotonicDistance(threshold);
    Point point(10, 10);
    for(int x=0; x<50; x++) {
      int measured = point.monotonicDistanceTo(x, 10);
      bool isWithin = measured < dist;
      CPPUNIT_ASSERT_EQUAL(x < 30, isWithin);
    }
  }

  Points create() {
    Config c = Config::forSize(100, 100);
    c.verbose = false;
    return Points(c);
  }

  void connect(Points& p, PointID from, PointID to) {
    Point& fromPoint = p.getPoint(from);
    Point& toPoint = p.getPoint(to);
    connect(p, fromPoint, toPoint);
  }
  void connect(Points& p, Point& from, Point& to) {
    int dist = from.monotonicDistanceTo(to);
    p.connectNeighbors(from, to, dist);
  }

  void testIdentifySpace() {
    Points points = create();
    points.addPoint(50, 50);
    points.addPoint(40, 50);
    CPPUNIT_ASSERT(points.hasPointsWithin(50, 50, 10));
    CPPUNIT_ASSERT(points.hasPointsWithin(59, 50, 10));
    CPPUNIT_ASSERT(!points.hasPointsWithin(60, 50, 10));

    CPPUNIT_ASSERT(points.hasPointsWithin(40, 50, 10));
    CPPUNIT_ASSERT(points.hasPointsWithin(31, 50, 10));
    CPPUNIT_ASSERT(!points.hasPointsWithin(31, 49, 9));

    CPPUNIT_ASSERT(points.hasPointsWithin(57, 57, 10));
    CPPUNIT_ASSERT(!points.hasPointsWithin(57, 58, 10));

    points.sanityCheck();
  }

  void testIterator() {
    Points points = create();
    points.addPoint(45, 45);
    points.addPoint(55, 55);

    int total = 0;
    int last = 0;
    for(auto& point : points.findPointsWithin(50, 50, 10)) {
      CPPUNIT_ASSERT(point.x == 45 || point.x == 55);
      CPPUNIT_ASSERT(last != point.x);
      last = point.x;
      total++;
    }
    CPPUNIT_ASSERT_EQUAL(2, total);

    for(auto& point : points.findPointsWithin(50, 50, 7)) {
      CPPUNIT_FAIL("Shouldn't match");
      (void) point;
    }

    total = 0;
    for(auto& point : points.findPointsWithin(52, 52, 5)) {
      CPPUNIT_ASSERT_EQUAL(55, point.x);
      CPPUNIT_ASSERT_EQUAL(0, total);
      total++;
    }
  }

  void testConnect() {
    Points points = create();
    std::vector<PointID> path;
    for(int i=0; i<10; i++) {
      PointID p = points.addPointAsId(i, i);
      path.push_back(p);

      PointID rando = points.addPointAsId(0,i);
      points.getPoint(p).addNeighbor(rando);
      points.getPoint(rando).addNeighbor(p);
    }
    for(size_t i=1; i<path.size(); i++) {
      Point& p1 = points.getPoint(path[i-1]);
      Point& p2 = points.getPoint(path[i]);
      p1.addNeighbor(path[i]);
      p2.addNeighbor(path[i-1]);
    }

    PointID lonely = points.addPointAsId(50, 50);

    std::vector<StackEntry> stack;
    CPPUNIT_ASSERT(!points.connect(stack, path[0], lonely));
    points.sanityCheck();
    points.setEndPoints(path[0], path.back());
    CPPUNIT_ASSERT(points.connect(stack, path[0], path.back()));
    points.shouldHavePath(true);
    points.sanityCheck();
  }

  void testExpand() {
    Points points = create();
    std::vector<StackEntry> stack;

    PointID p0 = points.addPointAsId(0, 0);
    PointID p1 = points.addPointAsId(1, 0);
    PointID p2 = points.addPointAsId(2, 0);
//    PointID detour2 = points.addPointAsId(2,1);
    points.getPoint(p0).addNeighbor(p1);
    points.getPoint(p1).addNeighbor(p2);

    CPPUNIT_ASSERT(points.connect(stack, p0, p2));
    points.setEndPoints(p0, p2);
    CPPUNIT_ASSERT_EQUAL(3, points.getPathLength());

    PointID detour1 = points.addPointAsId(1,1);
    points.getPoint(p0).addNeighbor(detour1);
    points.getPoint(detour1).addNeighbor(p1);

    PointID detour2 = points.addPointAsId(2,1);
    points.getPoint(p1).addNeighbor(detour2);
    points.getPoint(detour2).addNeighbor(p2);

    points.expandPath(stack);
    CPPUNIT_ASSERT_EQUAL(5, points.getPathLength());
  }

  void testWouldCrossAnyLong() {
    Config c = Config::forSize(1000, 1000);
    c.maxReach = 100;
    Points points(c);
    points.reservePoints(100);

    Point& blockA1 = points.addPoint(200, 0);
    Point& blockA2 = points.addPoint(200, c.maxReach);
    connect(points, blockA1, blockA2);

    Point& blockB1 = points.addPoint(502, 0);
    Point& blockB2 = points.addPoint(502, c.maxReach);
    connect(points, blockB1, blockB2);

    Point& at0 = points.addPoint(0, 1);
    Point& at100 = points.addPoint(100, 1);
    Point& at199 = points.addPoint(199, 1);
    Point& at201 = points.addPoint(201, 1);
    Point& at501 = points.addPoint(501, 1);
    Point& at503 = points.addPoint(503, 1);
    Point& at999 = points.addPoint(999, 1);

    CPPUNIT_ASSERT(!points.wouldCrossAnyLong(at0, at0));
    CPPUNIT_ASSERT(!points.wouldCrossAnyLong(at0, at100));
    CPPUNIT_ASSERT(!points.wouldCrossAnyLong(at0, at199));

    CPPUNIT_ASSERT(points.wouldCrossAnyLong(at0, at201));
    CPPUNIT_ASSERT(points.wouldCrossAnyLong(at0, at501));
    CPPUNIT_ASSERT(points.wouldCrossAnyLong(at0, at999));
    CPPUNIT_ASSERT(points.wouldCrossAnyLong(at199, at201));

    CPPUNIT_ASSERT(!points.wouldCrossAnyLong(at201, at501));
    CPPUNIT_ASSERT(points.wouldCrossAnyLong(at201, at503));
  }

  void testWouldCrossColinear() {
    Points points = create();
    points.reservePoints(100);

    Point& a = points.addPoint(0, 0);
    Point& b = points.addPoint(10, 0);
    Point& c = points.addPoint(20, 0);
    Point& d = points.addPoint(30, 0);

    CPPUNIT_ASSERT(!Point::wouldCross(a, b, c, d));
  }
};


CPPUNIT_TEST_SUITE_REGISTRATION(PointsTest);

int main(int argc, char* argv[])
{
  // Get the top level suite from the registry
  CppUnit::Test *suite = CppUnit::TestFactoryRegistry::getRegistry().makeTest();

  // Adds the test to the list of test to run
  CppUnit::TextUi::TestRunner runner;
  runner.addTest( suite );

  // Change the default outputter to a compiler error format outputter
  runner.setOutputter( new CppUnit::CompilerOutputter( &runner.result(),
                                                       std::cerr ) );
  // Run the tests.
  bool wasSucessful = runner.run();

  // Return error code 1 if the one of test failed.
  return wasSucessful ? 0 : 1;
}

#include "image.h"
#include "points.h"
#include <iostream>

using namespace oneline;

int main(int argc, char** argv) {
  int w=1200, h=800;

  auto img = Image::load(argc > 1 ? argv[1] : "image.jpg", w, h, 0, false);
  if(!img) {
    std::cerr << "Failed to load\n";
    return 1;
  }


  Points points(Config::forSize(w, h));

//  PointID start = points.addPointAsId(0, h/2);
//  PointID end = points.addPointAsId(w-1, h/2);
  points.fillRandom(*img);
  points.outputLabels("labels.dat");
  points.setEndPoints();
  points.makeGrid();
  
  std::vector<StackEntry> stack;
  stack.reserve(w);
  points.createLine(stack);

  std::cerr << "Emitting labels...\n";
  points.outputLabels("labels.dat");
  std::cerr << "Emitting grid...\n";
  points.outputGrid("grid.dat");
  std::cerr << "Emitting path...\n";
  points.outputPath("path.dat");
  std::cerr << "Emitting svg...\n";
  points.outputSVG("/home/vidar/public_html/lol.svg");

  if (points.getCoverage() < 0.95) return 1;
  return 0;
}


#include "options.h"
#include "Jzon.h"
#include "Jzon.cpp" // lol

namespace oneline {

struct options optionsFromString(const std::string& str) {
  struct options opts;
  Jzon::Parser parser;
  Jzon::Node node = parser.parseString(str);
  opts.resolution = node.get("resolution").toDouble();
  opts.lineWidth = node.get("lineWidth").toDouble();
  opts.contrast = node.get("contrast").toDouble();
  opts.whiteCutoff = node.get("whiteCutoff").toDouble();
  opts.invert = node.get("invert").toBool();
  return opts;
}

}

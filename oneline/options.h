#include <string>

namespace oneline {

struct options {
  double resolution;
  double lineWidth;
  double contrast;
  double whiteCutoff;
  bool invert;
};

struct options optionsFromString(const std::string& str);

}

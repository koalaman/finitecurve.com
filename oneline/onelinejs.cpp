#include "options.h"
#include "image.h"
#include "points.h"

#include <sstream>

#include <emscripten/bind.h>

using namespace oneline;
using namespace emscripten;

class OneLine {
  public:
    std::string result;
    std::string error;
    std::string image;
    std::vector<StackEntry> stack;
    int width, height;
    float lineDistance;
    struct options options;

    bool build() {
      result.clear();
      error.clear();
      width=-1;
      height=-1;

      int wh = (int) (options.resolution * 150);
      if (wh <= 0) { wh=50; };
      auto img = Image::loadBuffer(image, wh, wh, 1, options.invert);
      if (!img) {
	error = "Failed to decode image. Is it a valid jpg or png file?";
	return false;
      }

      img->adjustContrast(options.contrast, options.whiteCutoff);

      struct Config config = Config::forSize(img->getWidth(), img->getHeight());
      config.whiteCutoff = options.whiteCutoff;
      config.strokeWidth = options.lineWidth;

      Points points(config);
      points.fillRandom(*img);
      points.makeGrid();

      if(0) {
	std::stringstream out;
	points.outputGrid(out);
	result = out.str();
	return true;
      }
      points.setEndPoints();
      points.createLine(stack);

      std::stringstream output;
      points.outputSVG(output);
      result = output.str();
      width = points.getConfig().width;
      height = points.getConfig().height;
      lineDistance = points.getPathDistance();
      return true;
    }

    void setImage(const std::string& image) {
      this->image = image;
    }

    void setOptions(const std::string& options) {
      this->options = optionsFromString(options);
    }

    const std::string& getResult() {
      return result;
    }

    const std::string& getError() {
      return error;
    }

    int getWidth() const {
      return width;
    }

    int getHeight() const {
      return height;
    }

    float getLineDistance() const {
      return lineDistance;
    }
};

EMSCRIPTEN_BINDINGS(OneLine) {
  class_<OneLine>("OneLine")
    .constructor<>()
    .function("build", &OneLine::build)
    .function("setImage", &OneLine::setImage)
    .function("setOptions", &OneLine::setOptions)
    .function("getResult", &OneLine::getResult)
    .function("getError", &OneLine::getError)
    .function("getWidth", &OneLine::getWidth)
    .function("getHeight", &OneLine::getHeight)
    .function("getLineDistance", &OneLine::getLineDistance)
    ;
}


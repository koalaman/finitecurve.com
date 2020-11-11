#ifndef INCLUDE_IMAGE_H
#define INCLUDE_IMAGE_H

#include <memory>
#include <vector>

namespace oneline {

class Image {
  int width;
  int height;
  std::vector<unsigned char> pixels;

  Image(int w, int h) : width(w), height(h) {
    pixels.resize(w*h);
  }
  static std::unique_ptr<Image> loadFrom(const unsigned char* data, int imageW, int imageH, int w, int h, int smooth, bool invert);

  public:
  int getShade(int x, int y) const {
    return pixels[y*width+x] & 0xff;
  }

  int getWidth() const { return width; }
  int getHeight() const { return height; }

  void adjustContrast(int percent, int whiteCutoff);
  void invert();

  void dumpPGM(const std::string& file);
  static std::unique_ptr<Image> load(const std::string& file, int w, int h, int smooth, bool invert);
  static std::unique_ptr<Image> loadBuffer(const std::string& buf, int w, int h, int smooth, bool invert);
};

}
#endif

#include <iostream>
#include <fstream>

#include "image.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

using namespace oneline;

std::unique_ptr<Image> Image::loadFrom(const unsigned char* data, int imageWidth, int imageHeight, int width, int height, int smooth, bool invert) {
  Image* rawPtr = new Image(width, height);
  std::unique_ptr<Image> img(rawPtr);

  double xRatio = (double)imageWidth/width;
  double yRatio = (double)imageHeight/height;

  int boxW = (int) 2.0*smooth*xRatio;
  if (boxW <= 0) boxW = 1;
  int boxH = (int) 2.0*smooth*yRatio;
  if (boxH <= 0) boxH = 1;

  for(int y=0; y<height; y++) {
    for(int x=0; x<width; x++) {
      int n = 0, s = 0;
      bool transparent = false;

      for (int yd=-boxH; yd<=boxH; yd++) {
	int py = (int) (yRatio*(yd+y));
	if (py < 0) continue;
	if (py >= imageHeight) break;
	for (int xd=-boxH; xd<=boxH; xd++) {
	  int px = (int) (xRatio*(xd+x));
	  if (px < 0) continue;
	  if (px >= imageWidth) break;
	  int offset = (py*imageWidth+px)*2;
	  transparent |= data[offset+1] == 0;
	  s += data[offset];
	  n += 1;
	}
      }
      assert(n > 0);
      int val = n == 0 ? 0 : s/n;
      img->pixels[y*width+x] = transparent ? 255 : (invert ? 255-val : val);
    }
  }
  return std::move(img);
}

std::unique_ptr<Image> Image::loadBuffer(const std::string& imageData, int maxWidth, int maxHeight, int smooth, bool invert) {
  int imageWidth, imageHeight , _channels;
  unsigned char* data = stbi_load_from_memory((const unsigned char*) imageData.c_str(), imageData.size(), &imageWidth, &imageHeight, &_channels, 2);
  if(!data) return nullptr;

  double xRatio = (double) maxWidth / imageWidth;
  double yRatio = (double) maxHeight / imageHeight;
  int width = maxWidth, height = maxHeight;
  if (xRatio < yRatio) {
    height = (int) (xRatio * imageHeight);
  } else {
    width = (int) (yRatio * imageWidth);
  }

  auto img = loadFrom(data, imageWidth, imageHeight, width, height, smooth, invert);
  stbi_image_free(data);
  return img;
}

namespace {
  unsigned char truncate(float f) {
    if (f < 0) return 0;
    if (f > 255) return 255;
    return (unsigned char) f;
  }
}

std::unique_ptr<Image> Image::load(const std::string& file, int width, int height, int smooth, bool invert) {
  int imageWidth, imageHeight , _channels;
  unsigned char *data = stbi_load(file.c_str(), &imageWidth, &imageHeight, &_channels, 2);
  if(!data) return nullptr;

  auto img = loadFrom(data, imageWidth, imageHeight, width, height, smooth, invert);
  stbi_image_free(data);
  return img;
}

void Image::adjustContrast(int percent, int whiteCutoff) {
  if (percent == 50) return;

  float contrast = (percent/100.0f*512-256);
  float factor = 259*(contrast+255) / ((255 * (259-contrast)));
  for(int i=0, e=pixels.size(); i<e; i++) {
    if (pixels[i] >= whiteCutoff) continue;
    pixels[i] = truncate( factor * (pixels[i] - 128.0f) + 128.0f);
  }
}

void Image::invert() {
  for(int i=0, e=pixels.size(); i<e; i++) {
    pixels[i] = 255 - pixels[i];
  }
}

void Image::dumpPGM(const std::string& out) {
  std::ofstream file;
  file.open(out);
  file << "P2\n";
  file << width << " " << height << "\n";
  file << "255\n";
  for(int y=0; y<height; y++) {
    for(int x=0; x<width; x++) {
      file << getShade(x,y) << " ";
    }
    file << "\n";
  }
  file.close();
}

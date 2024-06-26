
#ifndef Jpeg_h
#define Jpeg_h

// based on
// http://soratobi96.hatenablog.com/entry/20180923/1537690266

#include <stdint.h>
#include <jpeglib.h>

struct JpegData {
    JpegData() : data(nullptr), width(0), height(0), ch(0) {}

    uint8_t *data;
    uint32_t width;
    uint32_t height;
    uint32_t ch;
};

bool readJpegData(JpegData &out_JpegData, const char *in_FileName);

bool
writeJpegData(const float *in_FilmBuffer, const char *out_FileName, const int width, const int height, const int ch);

void finalizeJpegData(JpegData &io_JpegData);

#endif /* Jpeg_h */


typedef struct _WARPDATA {
    unsigned char*the_image;
    unsigned char*out_image;
    int x_control;
    int y_control;
    int bilinear;
    int rows;
    int cols;
} WARPDATA;

void warp2(WARPDATA*wd);

void warp_loop(unsigned char*the_image, unsigned char*out_image,
          int x1, int x2, int x3, int x4,
          int y1, int y2, int y3, int y4,
          int extra_x, int extra_y,
          int rows, int cols);

void bi_warp_loop(unsigned char*the_image, unsigned char*out_image,
             int x1, int x2, int x3, int x4,
             int y1, int y2, int y3, int y4,
             int extra_x, int extra_y,
             int rows, int cols);

void object_warp(unsigned char*the_image, unsigned char*out_image,
            int x1, int y1, int x2, int y2,
            int x3, int y3, int x4, int y4,
            int bilinear, int rows, int cols);

void full_warp_loop(unsigned char*the_image, unsigned char*out_image,
               int x1, int x2, int x3, int x4,
               int y1, int y2, int y3, int y4,
               int extra_x, int extra_y,
               int rows, int cols);

void bi_full_warp_loop(unsigned char*the_image, unsigned char*out_image,
                  int x1, int x2, int x3, int x4,
                  int y1, int y2, int y3, int y4,
                  int extra_x, int extra_y,
                  int rows, int cols);


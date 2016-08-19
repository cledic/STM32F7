
typedef struct _GEOMDATA {
    unsigned char*the_image;
    unsigned char*out_image;
    float x_angle;
    float x_stretch;
    float y_stretch;
    int x_displace;
    int y_displace;
    float x_cross;
    float y_cross;
    int bilinear;
    int rows;
    int cols;
} GEOMDATA;

typedef struct _ROTDATA {
    unsigned char*the_image;
    unsigned char*out_image;
    float angle;
    int m;
    int n;
    int bilinear;
    int rows;
    int cols;
} ROTDATA;

void geometry2(GEOMDATA *gd);
void arotate2(ROTDATA *rd);

//int bilinear_interpolate(unsigned char *the_image, float x, float y, int rows, int cols);
int bilinear_interpolate(unsigned char *the_image, float x, float y);


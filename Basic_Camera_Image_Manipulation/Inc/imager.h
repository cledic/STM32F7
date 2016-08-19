
void fix_edges( char*im, int w);
void erosion( unsigned char*the_image, unsigned char*out_image, int value, int threshold);
void dilation(unsigned char*the_image, unsigned char*out_image, int value, int threshold);
void copy_result( unsigned char*a, unsigned char*b);
void enhance_edges(unsigned char*the_image, unsigned char*out_image, int high);
void gaussian_edge(unsigned char*the_image, unsigned char*out_image, int size, int threshold, int high);
void mask_dilation(unsigned char*the_image, unsigned char*out_image, int value, int mask_type);
void mask_erosion(unsigned char*the_image, unsigned char*out_image, int value, int mask_type);
void copy_3_x_3(unsigned char*a, unsigned char*b);


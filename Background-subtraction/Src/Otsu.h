#include <stdint.h>

/* Public functions */
int8_t getOtsuThreshold(uint8_t*p);
void OtsuThreshold(uint8_t*p, int thresh);
void OtsuThresholdGray(uint8_t*p, int thresh);
void OtsuThreshold2(uint8_t*p, int thresh, uint8_t*b);
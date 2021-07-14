#ifndef UTILITIES_H
#define UTILITIES_H

float getFloat(unsigned char *p, int index);
double getDouble(unsigned char *p, int index);
void getBytesFromFloat(unsigned char *p, int index, float f);
void getBytesFromDouble(unsigned char *p, int index, double d);

#endif // UTILITIES_H

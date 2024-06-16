#include <Arduino.h>
#ifndef INVERSEKINEMATIC
#define INVERSEKINEMATIC

void inverseMatrix(float mat[6][6], float mat_inv[6][6]);
void inverseMatrix_3(float mat[3][3], float mat_inv[3][3]);
#endif
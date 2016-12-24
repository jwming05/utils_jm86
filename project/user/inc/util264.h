#ifndef __UTIL_264_H
#define __UTIL_264_H

void ZigZagScan(int b8, int b4, int cofmatrix[4][4], int **cofAC);
void ReZigZagScan(int **cofAC, int cofmatrix[4][4]);

#endif
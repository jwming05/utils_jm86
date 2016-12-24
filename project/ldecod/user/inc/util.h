#ifndef __UTIL_H
#define __UTIL_H

int isOdd(int number);
int isEven(int number);
int LNZIndex(int *coeff);						//结尾非零系数的位置
void Lr2Zz(int *pLevel, int *pRun, int *coeff);	//Level Run转换为zigzag的数组

#endif
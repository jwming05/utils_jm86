#ifndef __UTIL_H
#define __UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "defines.h"

int mymod(int val, int m);				//支持负数求余

void clearArr(int *arr, int n);			//清空长度为n的1维数组
double average(int *number, int size);	//对数据求平均
double sum(int *number, int size);		//对数据求和
int isOdd(int number);					//判断number是否为奇数
int isEven(int number);					//判断number是否为偶数
int ArrayContains(int number, int *arr, int length);		//判断指定长度的内存中是否含有number

//数据转换函数
void Matrix2Lr(int matrix[4][4], int *pLevel, int *pRun);
void Lr2Matrix(int *pLevel, int *pRun, int matrix[4][4]);	//将level run转换为zigzag的数组
void Zz2Matrix(int *coeff, int matrix[4][4]);	//将zigzag的一维数组，转换为二维数组
void Lr2Zz(int *pLevel, int *pRun, int *coeff);	//Level Run转换为zigzag的数组
void Zz2Lr(int *pLevel, int *pRun, int *coeff);	//Zigzag转换为level run
int B8b42M(int b8, int b4);					//b8 b4所在行
int B8b42N(int b8, int b4);					//b8 b4所在列


int LNZIndex(int *coeff);						//结尾非零系数的位置
int LevelLnzIndex(int *level);					//level中的最后1为非零系数的index
int LNZ(int *coeff);							//结尾非零系数

void StatisticsLNZIndex(int* coeff);


#endif
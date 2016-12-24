#ifndef __UTIL_H
#define __UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "defines.h"

int mymod(int val, int m);				//֧�ָ�������

void clearArr(int *arr, int n);			//��ճ���Ϊn��1ά����
double average(int *number, int size);	//��������ƽ��
double sum(int *number, int size);		//���������
int isOdd(int number);					//�ж�number�Ƿ�Ϊ����
int isEven(int number);					//�ж�number�Ƿ�Ϊż��


void Zz2Matrix(int *coeff, int matrix[4][4]);	//��zigzag��һά���飬ת��Ϊ��ά����
void Lr2Zz(int *pLevel, int *pRun, int *coeff);	//Level Runת��Ϊzigzag������
void Zz2Lr(int *pLevel, int *pRun, int *coeff);	//Zigzagת��Ϊlevel run
int LNZIndex(int *coeff);						//��β����ϵ����λ��
int LevelLnzIndex(int *level);					//level�е����1Ϊ����ϵ����index
int LNZ(int *coeff);							//��β����ϵ��

void StatisticsLNZIndex(int* coeff);


#endif
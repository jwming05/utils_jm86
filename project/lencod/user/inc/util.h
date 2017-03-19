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
int ArrayContains(int number, int *arr, int length);		//�ж�ָ�����ȵ��ڴ����Ƿ���number

//����ת������
void Matrix2Lr(int matrix[4][4], int *pLevel, int *pRun);
void Lr2Matrix(int *pLevel, int *pRun, int matrix[4][4]);	//��level runת��Ϊzigzag������
void Zz2Matrix(int *coeff, int matrix[4][4]);	//��zigzag��һά���飬ת��Ϊ��ά����
void Lr2Zz(int *pLevel, int *pRun, int *coeff);	//Level Runת��Ϊzigzag������
void Zz2Lr(int *pLevel, int *pRun, int *coeff);	//Zigzagת��Ϊlevel run
int B8b42M(int b8, int b4);					//b8 b4������
int B8b42N(int b8, int b4);					//b8 b4������


int LNZIndex(int *coeff);						//��β����ϵ����λ��
int LevelLnzIndex(int *level);					//level�е����1Ϊ����ϵ����index
int LNZ(int *coeff);							//��β����ϵ��

void StatisticsLNZIndex(int* coeff);


#endif
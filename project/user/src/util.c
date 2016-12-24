#include "util.h"

//�ж�number�Ƿ�Ϊ����
int isOdd(int number)
{
	return abs(number) & 0x01;
}

//�ж�number�Ƿ�Ϊż��
int isEven(int number)
{
	return !(abs(number) & 0x01);
}

//֧�ָ�������
int mymod(int val, int m)
{
	while (val < 0)
	{
		val += m;
	}
	return val % m;
}

//��ճ���Ϊn��һά����
void clearArr(int *arr, int n)
{
	for (int i = 0; i < n; i++)
	{
		arr[i] = 0;
	}
}

//��������ƽ��
double average(int *number, int size)
{
	double sum = 0;
	for (int i = 0; i < size; i++)
	{
		sum += number[i];
	}
	return sum / size;
}

//���������
double sum(int *number, int size)
{
	double sum = 0;
	for (int i = 0; i < size; i++)
	{
		sum += number[i];
	}
	return sum;
}

//��zigzag��һά���飬ת��Ϊ��ά����
extern const byte SNGL_SCAN[16][2];
void Zz2Matrix(int *coeff, int matrix[4][4])
{
	for (int index = 0; index < 16; index++)
	{
		int i = SNGL_SCAN[index][0];
		int j = SNGL_SCAN[index][1];
		matrix[i][j] = coeff[index];
	}
}

//Level Runת��Ϊzigzag������
void Lr2Zz(int *pLevel, int *pRun, int *coeff)
{
	int cnt = 0;
	int index = 0;
	static int time = 0;
	time++;

	while (cnt < 16)
	{
		int preZeroNumber = pRun[index];
		int level = pLevel[index];

		if (level == 0)
		{	//levelΪ0 �� ȫ��Ϊ0
			while (cnt < 16)
			{
				coeff[cnt] = 0;
				cnt++;
			}
			break;
		}

		for (int i = 0; i < preZeroNumber; i++)
		{
			coeff[cnt] = 0;
			cnt += 1;
		}

		coeff[cnt] = level;
		cnt += 1;
		index += 1;
	}
}

//Zigzagת��Ϊlevel run
void Zz2Lr(int *pLevel, int *pRun, int *coeff)
{
	int coeffIndex = 0;
	int zeroLength = 0;
	int index = 0;
	
	//clear
	for (int i = 0; i < 16; i++)
	{
		pLevel[i] = 0;
		pRun[i] = 0;
	}

	while (coeffIndex < 16)
	{
		int co = coeff[coeffIndex];
		if (co == 0)
		{
			zeroLength++;
		}
		else
		{
			pLevel[index] = co;
			pRun[index] = zeroLength;
			zeroLength = 0;
			index++;
		}
		coeffIndex++;
	}
}

//level�е����1Ϊ����ϵ����index
int LevelLnzIndex(int *level)
{
	for (int i = 0; i < 16; i++)
	{
		if (level[i] == 0)
		{
			return i == 0 ? -1 : (i - 1);		//-1Ϊϵ��ȫ��
		}
	}
	return -2;	//unknown
}


//��β����ϵ����λ��
int LNZIndex(int *coeff)
{
	for (int i = 15; i >= 0; i--)
	{
		if (coeff[i] != 0)
		{
			return i;	//LNZ
		}
	}
	return -1;		//ȫ��
}

//��β����ϵ��
int LNZ(int *coeff)
{
	for (int i = 15; i >= 0; i--)
	{
		if (coeff[i] != 0)
		{
			return coeff[i];	//LNZ
		}
	}
	return 0;		//ȫ��
}

//ͳ�ƽ�β����ϵ��λ�ô���indexʱ��ֵ
void StatisticsLNZ(int *coeff, int index_thr)
{
	static int statistics[16];	//statistics[i] ��ʾ4x4���ȿ���lnz�ľ���ֵΪi�Ĵ���

	if (coeff == NULL)
	{
		printf("lnz times : ");
		for (int i = 0; i < 15; i++)
		{
			printf("%d ", statistics[i]);
		}
		printf("\n");
	}
	else
	{
		int lnz = abs(LNZ(coeff));
		int index = LNZIndex(coeff);
//		if (index >= index_thr)
		{
			statistics[lnz]++;
		}
	}
}

void StatisticsLNZIndex(int* coeff)
{
	static int statistics[16];
	static int times = 0;
	static int allZeroTimes = 0;

	if (coeff == NULL)
	{
		printf("times = %d, zero times = %d : ", times, allZeroTimes);
		for (int i = 0; i < 16; i++)
		{
			printf("%d ", statistics[i]);
		}
		printf("\n");
		printf("non all zero rate = %.2f : ", (double)(times - allZeroTimes) / times);
		for (int i = 0; i < 16; i++)
		{
			printf("%.2f ", (double)statistics[i] / (times - allZeroTimes));
		}
		printf("\n");
	}
	else
	{
		times++;
		int index = LNZIndex(coeff);
		if (index != -1)
		{
			statistics[index]++;
		}
		else
		{
			allZeroTimes++;
		}
	}	
}
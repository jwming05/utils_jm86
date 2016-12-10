#include "util.h"

//对数据求平均
double average(int *number, int size)
{
	double sum = 0;
	for (int i = 0; i < size; i++)
	{
		sum += number[i];
	}
	return sum / size;
}

//对数据求和
double sum(int *number, int size)
{
	double sum = 0;
	for (int i = 0; i < size; i++)
	{
		sum += number[i];
	}
	return sum;
}

//Level Run转换为zigzag的数组
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
		{	//level为0 ， 全置为0
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


//结尾非零系数的位置
int LNZIndex(int *coeff)
{
	for (int i = 15; i >= 0; i--)
	{
		if (coeff[i] != 0)
		{
			return i;	//LNZ
		}
	}
	return -1;		//全零
}

//结尾非零系数
int LNZ(int *coeff)
{
	for (int i = 15; i >= 0; i--)
	{
		if (coeff[i] != 0)
		{
			return coeff[i];	//LNZ
		}
	}
	return 0;		//全零
}

//统计结尾非零系数位置大于index时的值
void StatisticsLNZ(int *coeff, int index_thr)
{
	static int statistics[16];	//statistics[i] 表示4x4亮度块中lnz的绝对值为i的次数

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
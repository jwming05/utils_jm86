#include "util.h"

//判断number是否为奇数
int isOdd(int number)
{
	return abs(number) & 0x01;
}

//判断number是否为偶数
int isEven(int number)
{
	return !(abs(number) & 0x01);
}

//支持负数求余
int mymod(int val, int m)
{
	while (val < 0)
	{
		val += m;
	}
	return val % m;
}

//清空长度为n的一维数组
void clearArr(int *arr, int n)
{
	for (int i = 0; i < n; i++)
	{
		arr[i] = 0;
	}
}

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

//将zigzag的一维数组，转换为二维数组
static const int SNGL_SCAN[16][2] =
{
	{ 0, 0 }, { 1, 0 }, { 0, 1 }, { 0, 2 },
	{ 1, 1 }, { 2, 0 }, { 3, 0 }, { 2, 1 },
	{ 1, 2 }, { 0, 3 }, { 1, 3 }, { 2, 2 },
	{ 3, 1 }, { 3, 2 }, { 2, 3 }, { 3, 3 }
};
void Zz2Matrix(int *coeff, int matrix[4][4])
{
	for (int index = 0; index < 16; index++)
	{
		int i = SNGL_SCAN[index][0];
		int j = SNGL_SCAN[index][1];
		matrix[i][j] = coeff[index];
	}
}

void Lr2Matrix(int *pLevel, int *pRun, int matrix[4][4])
{
	int coeff[16];
	Lr2Zz(pLevel, pRun, coeff);
	Zz2Matrix(coeff, matrix);						//将zigzag转换为matrix
}

void Matrix2Lr(int matrix[4][4], int *pLevel, int *pRun)
{
	int cof_count, i, j;
	int level, run = -1, scan_pos = 0;
	clearArr(pLevel, 16);
	clearArr(pRun, 16);

	for (cof_count = 0; cof_count < 16; cof_count++)
	{
		i = SNGL_SCAN[cof_count][0];
		j = SNGL_SCAN[cof_count][1];
		run++;
		level = matrix[i][j];
		if (level != 0)
		{
			pLevel[scan_pos] = level;
			pRun[scan_pos] = run;
			++scan_pos;
			run = -1;
		}
	}
	pLevel[scan_pos] = 0;
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

//Zigzag转换为level run
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

//b8 b4所在行
int B8b42M(int b8, int b4)
{
	return ((8 * (b8 / 2) + 4 * (b4 / 2))) / 4;
}

//b8 b4所在列
int B8b42N(int b8, int b4)
{
	return ((8 * (b8 % 2) + 4 * (b4 % 2))) / 4;
}

//level中的最后1为非零系数的index
int LevelLnzIndex(int *level)
{
	for (int i = 0; i < 16; i++)
	{
		if (level[i] == 0)
		{
			return i == 0 ? -1 : (i - 1);		//-1为系数全零
		}
	}
	return -2;	//unknown
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

int ArrayContains(int number, int *arr, int length)
{
	for (int i = 0; i < length; i++)
	{
		if (arr[i] == number)
		{
			return 1;
		}
	}
	return 0;
}
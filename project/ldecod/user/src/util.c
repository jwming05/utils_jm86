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
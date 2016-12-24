#include "hide.h"
#include "event.h"
#include "util.h"
#include "util264.h"

static void LNZMove(int *coeff, int ref)
{
	int index = LNZIndex(coeff);
	coeff[(index + ref) & 0x0F] = coeff[index];
//	coeff[index] = 0;
}

static int get_feature_value(int *g, int n)
{
	int feature_value = 0;
	int max_value = pow(3, n);
	for (int i = 0; i < n; i++)
	{
		feature_value += pow(3, i) * g[i];
	}
	feature_value = mymod(feature_value, max_value);
}

static int find(int n, int *delta, int max_n, int target)
{
	if (n == 0)
	{
		int val = get_feature_value(delta, max_n);
		if (target == val)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		for (int i = -1; i <= 1; i++)
		{
			delta[n - 1] = i;
			if (find(n - 1, delta, max_n, target) == 1)
			{
				return 1;
			}
		}
		return 0;
	}
}

void hide1()
{
	static double sum = 0;
	Macroblock    *currMB = &img->mb_data[img->current_mb_nr];

	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		int canModifyNumber = 0;
		int eles[16];
		int eles_position[16];
		int delta[16];
		int modifyLevel = 0;
		clearArr(eles, 16);
		clearArr(eles_position, 16);
		clearArr(delta, 16);

		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int *pLevel = img->cofAC[b8][b4][0];
				int *pRun = img->cofAC[b8][b4][1];
				int coeff[16];

				Lr2Zz(pLevel, pRun, coeff);
				int index = LNZIndex(coeff);
				if (index >= 13)
				{
					eles[canModifyNumber] = coeff[index];				//
					eles_position[canModifyNumber] = b8 * 4 + b4;		//记录使用到的宏块值
					canModifyNumber++;
				}
			}
		}

		//矩形编码
		int max_value = pow(3, canModifyNumber);
		int embedded_value = rand() % max_value;
		int feature_value = get_feature_value(eles, canModifyNumber);

		int d_value = embedded_value - feature_value;
		d_value = mymod(d_value, max_value);
		if (find(canModifyNumber, delta, canModifyNumber, d_value) == 0)
		{
			exit(1);
		}
		for (int i = 0; i < canModifyNumber; i++)
		{
			eles[i] += delta[i];
			modifyLevel += abs(delta[i]);
		}
		feature_value = get_feature_value(eles, canModifyNumber);
		//将新的像素值放入宏块中
		for (int i = 0; i < canModifyNumber; i++)
		{
			int cofmatrix[4][4];
			int b8 = eles_position[i] / 4;
			int b4 = eles_position[i] % 4;
			int *pLevel = img->cofAC[b8][b4][0];
			int *pRun = img->cofAC[b8][b4][1];
			int coeff[16];
			Lr2Zz(pLevel, pRun, coeff);
			int index = LNZIndex(coeff);
			coeff[index] = eles[i];
			Zz2Lr(pLevel, pRun, coeff);

			Zz2Matrix(coeff, cofmatrix);		//将zigzag转换为matrix
			ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
		}

		double bits = log(max_value) / log(2);
		sum += bits;
		printf("canNumber = %d, modifyLevel = %d, %.2f, %.2f\n", canModifyNumber, modifyLevel, bits, sum);
	}
}

void hide2()
{
	static int embedded_value = 0;
	static int bits = 0;

	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		Macroblock *currentMb = &img->mb_data[img->current_mb_nr];
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int cofmatrix[4][4];
				int coeff[16];
				int *pLevel = img->cofAC[b8][b4][0];
				int *pRun = img->cofAC[b8][b4][1];

				Lr2Zz(pLevel, pRun, coeff);
				int index = LNZIndex(coeff);
				
				if (index >= 9)
				{
					int bit = embedded_value;
					embedded_value = !embedded_value;

					if (bit == 0)
					{
						if (isOdd(coeff[index]))
						{
							coeff[index] += (coeff[index] > 0 ? 1 : -1);
						}
					}
					else
					{
						if (isEven(coeff[index]) && coeff[index]<16)
						{
							coeff[index] += (coeff[index] > 0 ? 1 : -1);
						}
						else if (coeff[index] == 16)
						{
							coeff[index] -= 1;
						}
						else if (coeff[index] == -16)
						{
							coeff[index] += 1;
						}
					}
					bits++;
					//*********************************************************
					FILE *fp = fopen("t1.txt", "a");
					fprintf(fp, "%d %d %d %d : ", img->current_mb_nr, b8, b4, embedded_value);
					for (int i = 0; i < 16; i++)
					{
						fprintf(fp, "\t%d", coeff[i]);
					}
					fprintf(fp, "\n");
					fclose(fp);
					//**********************************************************
					Zz2Matrix(coeff, cofmatrix);		//将zigzag转换为matrix
					ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
				}
			}
		}
	}
	printf("bits = %d\n", bits);
}

void hide3()
{
	static int embedded_value = 0;
	static int bits = 0;

	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		Macroblock *currentMb = &img->mb_data[img->current_mb_nr];
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int cofmatrix[4][4];
				int coeff[16];
				int *pLevel = img->cofAC[b8][b4][0];
				int *pRun = img->cofAC[b8][b4][1];

				Lr2Zz(pLevel, pRun, coeff);
				int index = LNZIndex(coeff);

				if (index >= 13)
				{
					int bit = embedded_value;
					embedded_value = !embedded_value;

					if (bit == 0)
					{
						if (isOdd(index))
						{
							LNZMove(coeff, 1);
						}
					}
					else
					{
						if (isEven(index) && index<15)
						{
							LNZMove(coeff, 1);
						}
						else if (index == 15)
						{
							LNZMove(coeff, -1);
						}
					}
					bits++;
					//*********************************************************
					FILE *fp = fopen("t1.txt", "a");
					fprintf(fp, "%d %d %d %d : ", img->current_mb_nr, b8, b4, embedded_value);
					for (int i = 0; i < 16; i++)
					{
						fprintf(fp, "\t%d", coeff[i]);
					}
					fprintf(fp, "\n");
					fclose(fp);
					//**********************************************************
					Zz2Matrix(coeff, cofmatrix);		//将zigzag转换为matrix
					ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
				}
			}
		}
	}
	printf("bits = %d\n", bits);
}

void hide4()
{
	static double sum = 0;
	Macroblock    *currMB = &img->mb_data[img->current_mb_nr];

	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		int canModifyNumber = 0;
		int eles[16];
		int eles_position[16];
		int delta[16];
		int modifyLevel = 0;
		clearArr(eles, 16);
		clearArr(eles_position, 16);
		clearArr(delta, 16);

		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int *pLevel = img->cofAC[b8][b4][0];
				int *pRun = img->cofAC[b8][b4][1];
				int coeff[16];

				Lr2Zz(pLevel, pRun, coeff);
				int index = LNZIndex(coeff);
				if (index >= 7 && index < 15)
				{
					if (coeff[index - 1] == 0 && coeff[index + 1] == 0)
					{
						eles[canModifyNumber] = index;				//
						eles_position[canModifyNumber] = b8 * 4 + b4;		//记录使用到的宏块值
						canModifyNumber++;
					}
				}
			}
		}

		//矩形编码
		int max_value = pow(3, canModifyNumber);
		int embedded_value = rand() % max_value;
		int feature_value = get_feature_value(eles, canModifyNumber);

		int d_value = embedded_value - feature_value;
		d_value = mymod(d_value, max_value);
		if (find(canModifyNumber, delta, canModifyNumber, d_value) == 0)
		{
			exit(1);
		}

		feature_value = get_feature_value(eles, canModifyNumber);
		//将新的像素值放入宏块中
		for (int i = 0; i < canModifyNumber; i++)
		{
			int cofmatrix[4][4];
			int b8 = eles_position[i] / 4;
			int b4 = eles_position[i] % 4;
			int *pLevel = img->cofAC[b8][b4][0];
			int *pRun = img->cofAC[b8][b4][1];
			int coeff[16];
			Lr2Zz(pLevel, pRun, coeff);

			LNZMove(coeff, delta[i]);

			Zz2Matrix(coeff, cofmatrix);		//将zigzag转换为matrix
			ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
		}

		double bits = log(max_value) / log(2);
		sum += bits;
		printf("canNumber = %d, modifyLevel = %d, %.2f, %.2f\n", canModifyNumber, modifyLevel, bits, sum);
	}
}
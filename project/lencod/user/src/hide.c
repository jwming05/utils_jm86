#include "hide.h"
#include "event.h"
#include "util.h"
#include "util264.h"
#include "nsc.h"

//防帧内失真漂移+LSB
void hide1()
{
	static int embedded_value = 0;
	static int bits = 0;
	static int right_block_mode[] = { 0, 3, 7 };
	static int under_block_mode[] = { 1, 8 };
	static int under_left_block_mode[] = { 0, 1, 2, 4, 5, 6, 8 };
	static int under_right_block_mode[] = { 0, 1, 2, 3, 7, 8 };
	static int lastRightMode[4];
	static int embedded = 0;

	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		Macroblock *currentMb = &img->mb_data[img->current_mb_nr];
		int **ipredmode4 = img->ipredmode;
		int mode[4][4];
		int haveLeft = 0;

		// 加载当前宏块中所有4x4块的帧内预测模式
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int block4_x = (8 * (b8 % 2) + 4 * (b4 % 2));//++ 当前4*4块左上角像素在本宏块内的横坐标（以像素为单位）
				int block4_y = (8 * (b8 / 2) + 4 * (b4 / 2));//++ 当前4*4块左上角像素在本宏块内的纵坐标（以像素为单位）
				int pic_pix_x = img->pix_x + block4_x;
				int pic_pix_y = img->pix_y + block4_y;
				int pic_block4_x = pic_pix_x / 4;
				int pic_block4_y = pic_pix_y / 4;

				mode[block4_y / 4][block4_x / 4] = ipredmode4[pic_block4_x][pic_block4_y];
			}
		}

		if (img->current_mb_nr % 11 != 0)
		{
			haveLeft = 1;
		}

		//嵌入数据
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int cofmatrix[4][4];
				int *pLevel = img->cofAC[b8][b4][0];
				int *pRun = img->cofAC[b8][b4][1];
				int m = ((8 * (b8 / 2) + 4 * (b4 / 2))) / 4;		//当前待嵌入数据块所位于的【行】
				int n = ((8 * (b8 % 2) + 4 * (b4 % 2))) / 4;		//当前待嵌入数据块所位于的【列】
				int doublePoint[2][2] = { -1 };

				Lr2Matrix(pLevel, pRun, cofmatrix);
				//if (cofmatrix[0][0] <= 0){continue;}				//跳过

				if (m < 3 && (n == 1 || n == 2))
				{
					if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
					{	//满足条件1
						bits += 1;
						doublePoint[0][0] = 3;
						doublePoint[0][1] = 0;
						doublePoint[1][0] = 3;
						doublePoint[1][1] = 2;
						//						cofmatrix[3][0] += 1;
						//						cofmatrix[3][2] -= 1;
					}
					else if ((ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
						&& (ArrayContains(mode[m + 1][n - 1], under_left_block_mode, 7) == 1))
					{	//满足条件2
						bits += 1;
						doublePoint[0][0] = 0;
						doublePoint[0][1] = 3;
						doublePoint[1][0] = 2;
						doublePoint[1][1] = 3;
						//						cofmatrix[0][3] += 1;
						//						cofmatrix[2][3] -= 1;
					}
				}
				else if (n == 0 & m != 3)
				{
					if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
					{	//满足条件1
						bits += 1;
						doublePoint[0][0] = 3;
						doublePoint[0][1] = 0;
						doublePoint[1][0] = 3;
						doublePoint[1][1] = 2;
						//						cofmatrix[3][0] += 1;
						//						cofmatrix[3][2] -= 1;
					}
					else if (ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
					{	//满足条件2
						bits += 1;
						doublePoint[0][0] = 0;
						doublePoint[0][1] = 3;
						doublePoint[1][0] = 2;
						doublePoint[1][1] = 3;
						//						cofmatrix[0][3] += 1;
						//						cofmatrix[2][3] -= 1;
					}
				}
				else if (n == 3 && m != 3)
				{
					if ((ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
						&& (ArrayContains(mode[m + 1][n - 1], under_left_block_mode, 7) == 1))
					{	//满足条件2
						bits += 1;
						doublePoint[0][0] = 0;
						doublePoint[0][1] = 3;
						doublePoint[1][0] = 2;
						doublePoint[1][1] = 3;
						//						cofmatrix[0][3] += 1;
						//						cofmatrix[2][3] -= 1;
					}
				}
				else if (m != 3)
				{
					if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
					{	//满足条件1
						bits += 1;
						doublePoint[0][0] = 3;
						doublePoint[0][1] = 0;
						doublePoint[1][0] = 3;
						doublePoint[1][1] = 2;
						//						cofmatrix[3][0] += 1;
						//						cofmatrix[3][2] -= 1;
					}
				}


				/*
				cofmatrix[3][0] += 2;
				cofmatrix[3][2] -= 2;
				cofmatrix[1][2] += 1;
				cofmatrix[1][0] -= 1;
				*/

				if (doublePoint[0][0] != -1)
				{
					if (embedded == 0)
					{
						if (cofmatrix[doublePoint[0][0]][doublePoint[0][1]] % 2 != 0)
						{
							cofmatrix[doublePoint[0][0]][doublePoint[0][1]] += 1;
							cofmatrix[doublePoint[1][0]][doublePoint[1][1]] -= 1;
						}
					}
					else
					{
						if (cofmatrix[doublePoint[0][0]][doublePoint[0][1]] % 2 == 0)
						{
							cofmatrix[doublePoint[0][0]][doublePoint[0][1]] += 1;
							cofmatrix[doublePoint[1][0]][doublePoint[1][1]] -= 1;
						}
					}

					embedded = !embedded;
				}
				//**********************************************************
				ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
			}
		}

		for (int ind = 0; ind < 4; ind++)
		{
			lastRightMode[ind] = mode[ind][3];
		}
	}
	printf("bits:%d\n", bits);
}


//防帧内失真漂移+n维超立方编码
void hide2()
{
	static int right_block_mode[] = { 0, 3, 7 };
	static int under_block_mode[] = { 1, 8 };
	static int under_left_block_mode[] = { 0, 1, 2, 4, 5, 6, 8 };
	static int under_right_block_mode[] = { 0, 1, 2, 3, 7, 8 };
	static int embedded = 0;
	static double sum = 0;

	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		Macroblock *currentMb = &img->mb_data[img->current_mb_nr];
		int **ipredmode4 = img->ipredmode;
		int mode[4][4];

		// 加载当前宏块中所有4x4块的帧内预测模式
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int block4_x = (8 * (b8 % 2) + 4 * (b4 % 2));//++ 当前4*4块左上角像素在本宏块内的横坐标（以像素为单位）
				int block4_y = (8 * (b8 / 2) + 4 * (b4 / 2));//++ 当前4*4块左上角像素在本宏块内的纵坐标（以像素为单位）
				int pic_pix_x = img->pix_x + block4_x;
				int pic_pix_y = img->pix_y + block4_y;
				int pic_block4_x = pic_pix_x / 4;
				int pic_block4_y = pic_pix_y / 4;

				mode[block4_y / 4][block4_x / 4] = ipredmode4[pic_block4_x][pic_block4_y];
			}
		}

		//筛选可以嵌入数据并计数
		int canModifyNumber = 0;
		int ele_pos[2][50] = {0};

		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int cofmatrix[4][4];
				int m = B8b42M(b8, b4);		//块所位于的【行】
				int n = B8b42N(b8, b4);		//块所位于的【列】

				Lr2Matrix(img->cofAC[b8][b4][0], img->cofAC[b8][b4][1], cofmatrix);
				//if (cofmatrix[0][0] <= 0){continue;}				//跳过

				if (m < 3 && (n == 1 || n == 2))
				{//part 1
					if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
					{	//满足条件1
						ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 0;
						ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 2;
						canModifyNumber++;
						/*
						if (canModifyNumber <= 7)
						{
							ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 2;
							ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 0;
							canModifyNumber++;
						}
						*/
						if (canModifyNumber <= 7)
						{
							ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 2;
							ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 0;
							canModifyNumber++;
						}

					}
					else if ((ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
						&& (ArrayContains(mode[m + 1][n - 1], under_left_block_mode, 7) == 1))
					{	//满足条件2;
						ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 0 * 10 + 3;
						ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 3;
						canModifyNumber++;
					}
				}
				else if (n == 0 & m != 3)
				{//part 2
					if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
					{	//满足条件1
						ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 0;
						ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 2;
						canModifyNumber++;

						/*
						if (canModifyNumber <= 7)
						{
							ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 2;
							ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 0;
							canModifyNumber++;
						}
						*/
					}
					else if (ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
					{	//满足条件2，不用考察左下的块，因为是肯定不会用到当前4x4块的(编码左下块时，当前4x4块还没编码的，所以肯定不会用的)
						ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 0 * 10 + 3;
						ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 3;
						canModifyNumber++;
					}
				}
				else if (n == 3 && m != 3)
				{//part 3
					if ((ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
						&& (ArrayContains(mode[m + 1][n - 1], under_left_block_mode, 7) == 1))
					{	//满足条件2
						ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 0 * 10 + 3;
						ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 3;
						canModifyNumber++;
					}
				}
				else if (m != 3)
				{
					if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
					{	//满足条件1
						ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 0;
						ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 2;
						canModifyNumber++;
						/*
						if (canModifyNumber <= 7)
						{
							ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 2;
							ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 2 * 10 + 0;
							canModifyNumber++;
						}
						*/
					}
				}
				else if (n == 3 && m == 3)
				{
				//	ele_pos[0][canModifyNumber] = b8 * 1000 + b4 * 100 + 3 * 10 + 0;
				//	ele_pos[1][canModifyNumber] = b8 * 1000 + b4 * 100 + 4 * 10 + 4;
				//	canModifyNumber++;
				}
				//**********************************************************
				//ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
			}
		}

		//矩形编码
		int modifyLevel = 0;
		int delta[16];
		int max_value = pow(3, canModifyNumber);
		int embedded_value = rand() % max_value;
		int eles[16] = { 0 };

		for (int i = 0; i < canModifyNumber; i++)
		{
			int b8 = ele_pos[0][i] / 1000;
			int b4 = (ele_pos[0][i] % 1000) / 100;
			int m = (ele_pos[0][i] % 100) / 10;
			int n = ele_pos[0][i] % 10;
			int *pLevel = img->cofAC[b8][b4][0];
			int *pRun = img->cofAC[b8][b4][1];
			int cofmatrix[4][4];

			Lr2Matrix(pLevel, pRun, cofmatrix);
			eles[i] = cofmatrix[m][n];
		}

		int feature_value = get_feature_value(eles, canModifyNumber);

		int d_value = embedded_value - feature_value;
		d_value = mymod(d_value, max_value);
	//	printf("canModifyNumber:%d\n", canModifyNumber);
		if (target_adapter(canModifyNumber, delta, canModifyNumber, d_value) == 0){exit(1);}
		for (int i = 0; i < canModifyNumber; i++)
		{
			eles[i] += delta[i];
		}
		feature_value = get_feature_value(eles, canModifyNumber);

		for (int i = 0; i < canModifyNumber; i++)
		{
			int b8 = ele_pos[0][i] / 1000;
			int b4 = (ele_pos[0][i] % 1000) / 100;
			int m0 = (ele_pos[0][i] % 100) / 10;
			int n0 = ele_pos[0][i] % 10;
			int m1= (ele_pos[1][i] % 100) / 10;
			int n1 = ele_pos[1][i] % 10;
			int *pLevel = img->cofAC[b8][b4][0];
			int *pRun = img->cofAC[b8][b4][1];
			int cofmatrix[4][4];

			Lr2Matrix(pLevel, pRun, cofmatrix);
			cofmatrix[m0][n0] += delta[i];
			if (m1 != 4 && n1 != 4)
			{
				cofmatrix[m1][n1] -= delta[i];
			}

			ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//将会重置level和run 以及 由此带来的新的解码像素
		}

		double bits = log(max_value) / log(2);
		sum += bits;
		printf("canNumber = %d, modifyLevel = %d, %.2f, %.2f\n", canModifyNumber, modifyLevel, bits, sum);
	}
}
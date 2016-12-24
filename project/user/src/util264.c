#include "util264.h"
#include "image.h"
#include <stdlib.h>

int ipredpix[16][16];//记录预测像素像素值

#define IS_RIGHT_MODE(x) (x == 0 || x == 3 || x == 7)//4*4模式定义
#define IS_UNDER_MODE(x) (x == 1 || x == 8 )
#define IS_UNDERL_MODE(x) (x == -2 || x == -1 ||x == 0 || x == 1 || x == 2 || x == 4 || x == 5 || x == 6 || x == 8)
#define IS_UNDERR_MODE(x) (x == 4 || x== 5 || x == 6)
#define Th 1

#define Q_BITS		15
#define DQ_BITS		6
#define DQ_ROUND	(1<<(DQ_BITS-1))

extern const byte SNGL_SCAN[16][2];
extern const byte FIELD_SCAN[16][2];

static const int dequant_coef[6][4][4] = {
	{ { 10, 13, 10, 13 }, { 13, 16, 13, 16 }, { 10, 13, 10, 13 }, { 13, 16, 13, 16 } },
	{ { 11, 14, 11, 14 }, { 14, 18, 14, 18 }, { 11, 14, 11, 14 }, { 14, 18, 14, 18 } },
	{ { 13, 16, 13, 16 }, { 16, 20, 16, 20 }, { 13, 16, 13, 16 }, { 16, 20, 16, 20 } },
	{ { 14, 18, 14, 18 }, { 18, 23, 18, 23 }, { 14, 18, 14, 18 }, { 18, 23, 18, 23 } },
	{ { 16, 20, 16, 20 }, { 20, 25, 20, 25 }, { 16, 20, 16, 20 }, { 20, 25, 20, 25 } },
	{ { 18, 23, 18, 23 }, { 23, 29, 23, 29 }, { 18, 23, 18, 23 }, { 23, 29, 23, 29 } }
};

void ZigZagScan(int b8, int b4, int cofmatrix[4][4], int **cofAC)
{
	//int cof[2][17] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};//游程编码后的值 0:level 1:run
	int sign(int a, int b);
	int cof_count, i, j, i1, j1;
	int ilev, level, run = -1, scan_pos = 0;
	int m5[4], m6[4], m7[4][4];
	//run=-1;
	//scan_pos=0;
	int qp_per, qp_rem, q_bits, qp_const;
	int *ACLevel = cofAC[0];
	int *ACRun = cofAC[1];
	int block_x, block_y;
	Macroblock *currMB = &img->mb_data[img->current_mb_nr];
	qp_per = (currMB->qp - MIN_QP) / 6;
	qp_rem = (currMB->qp - MIN_QP) % 6;
	q_bits = Q_BITS + qp_per;

	block_x = (8 * (b8 % 2) + 4 * (b4 % 2));//++ 当前4*4块左上角像素在本宏块内的横坐标（以像素为单位）
	block_y = (8 * (b8 / 2) + 4 * (b4 / 2));//++ 当前4*4块左上角像素在本宏块内的纵坐标（以像素为单位）

	if (img->type == I_SLICE)
		qp_const = (1 << q_bits) / 3;    // intra
	else
		qp_const = (1 << q_bits) / 6;    // inter
	for (cof_count = 0; cof_count < 16; cof_count++)
	{
		if (img->field_picture || (img->MbaffFrameFlag && currMB->mb_field))
		{
			// Alternate scan for field coding
			i = FIELD_SCAN[cof_count][0];
			j = FIELD_SCAN[cof_count][1];
		}
		else
		{
			i = SNGL_SCAN[cof_count][0];
			j = SNGL_SCAN[cof_count][1];
		}
		run++;
		ilev = 0;
		level = cofmatrix[i][j];
		if (level != 0)
		{
			ACLevel[scan_pos] = level;
			ACRun[scan_pos] = run;
			++scan_pos;
			run = -1;
			ilev = abs(level)*dequant_coef[qp_rem][i][j] << qp_per;
			//ilev = sign(ilev,level);
		}
		m7[i][j] = sign(ilev, level);
	}
	ACLevel[scan_pos] = 0;

	//     IDCT.
	//     horizontal

	for (j = 0; j < BLOCK_SIZE; j++)
	{
		for (i = 0; i < BLOCK_SIZE; i++)
		{
			m5[i] = m7[i][j];
		}
		m6[0] = (m5[0] + m5[2]);
		m6[1] = (m5[0] - m5[2]);
		m6[2] = (m5[1] >> 1) - m5[3];
		m6[3] = m5[1] + (m5[3] >> 1);

		for (i = 0; i < 2; i++)
		{
			i1 = 3 - i;
			m7[i][j] = m6[i] + m6[i1];
			m7[i1][j] = m6[i] - m6[i1];
		}
	}

	//  vertical

	for (i = 0; i < BLOCK_SIZE; i++)
	{
		for (j = 0; j < BLOCK_SIZE; j++)
		{
			m5[j] = m7[i][j];
		}
		m6[0] = (m5[0] + m5[2]);
		m6[1] = (m5[0] - m5[2]);
		m6[2] = (m5[1] >> 1) - m5[3];
		m6[3] = m5[1] + (m5[3] >> 1);

		for (j = 0; j < 2; j++)
		{
			j1 = 3 - j;
			m7[i][j] = min(255, max(0, (m6[j] + m6[j1] + (ipredpix[i + block_x][j + block_y] << DQ_BITS) + DQ_ROUND) >> DQ_BITS));
			m7[i][j1] = min(255, max(0, (m6[j] - m6[j1] + (ipredpix[i + block_x][j1 + block_y] << DQ_BITS) + DQ_ROUND) >> DQ_BITS));
		}
	}

	//  Decoded block moved to frame memory
	for (j = 0; j < BLOCK_SIZE; j++)
		for (i = 0; i < BLOCK_SIZE; i++)
			enc_picture->imgY[img->pix_y + block_y + j][img->pix_x + block_x + i] = m7[i][j];
}

void ReZigZagScan(int **cofAC, int cofmatrix[4][4])
{
	int i, j, k;
	int coftemp[16] = { 0 };
	int count = 0;
	Macroblock *currMB = &img->mb_data[img->current_mb_nr];
	for (k = 0; k < 16;)
	{
		if (cofAC[0][count] == 0)
			break;
		if (cofAC[1][count] != 0)
		{
			k = k + cofAC[1][count];
		}
		coftemp[k++] = cofAC[0][count];
		count++;
	}

	for (k = 0; k < 16; k++)
	{
		if (img->field_picture || (img->MbaffFrameFlag && currMB->mb_field))
		{
			// Alternate scan for field coding
			i = FIELD_SCAN[k][0];
			j = FIELD_SCAN[k][1];
		}
		else
		{
			i = SNGL_SCAN[k][0];
			j = SNGL_SCAN[k][1];
		}
		cofmatrix[i][j] = coftemp[k];
	}
}//帧编码的图像才需要zigzag扫描，场编码的是场扫描，为什么已经进行了反zigzag扫描还要再用场模式和帧模式再扫描一遍呢
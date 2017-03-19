#include "hide.h"
#include "event.h"
#include "util.h"
#include "util264.h"
#include "nsc.h"

const static int maxModifyEleNumber = 7;
const static int isSkipNoDC = 0;
const static int right_block_mode[] = { 0, 3, 7 };
const static int under_block_mode[] = { 1, 8 };
const static int under_left_block_mode[] = { 0, 1, 2, 4, 5, 6, 8 };
const static int under_right_block_mode[] = { 0, 1, 2, 3, 7, 8 };
const static int pairSets[2][3][4] = {
	{ { 3, 0, 3, 2 }, { 2, 2, 2, 0 }, { 1, 2, 1, 0 } },		//��֤���·���Ư�Ƶ����ϵ������
	{ { 0, 3, 2, 3 }, { 2, 2, 0, 2 }, { 2, 1, 0, 1 } } };	//��֤���ұ���Ư�Ƶ����ϵ������

static void LoadMode(int mode[4][4]);
static int filter4x4Block(Macroblock *currentMb, int mode[4][4], int buffer[]);
static int filterEles(int ele_pos[2][50], int buffer[], int blockNumber);

void truehide()
{
	static double sum = 0;

	if (img->mb_data[img->current_mb_nr].mb_type != I4MB){ return; }
	Macroblock *currentMb = &img->mb_data[img->current_mb_nr];

	int mode[4][4];

	LoadMode(mode);		// ���ص�ǰ���������4x4���֡��Ԥ��ģʽ

	//ɸѡ������Ƕ�����ݵ�4x4��
	int block_buffer[50] = { -1 };
	int ele_pos[2][50];
	int modifyEleNumber = filterEles(ele_pos, block_buffer, filter4x4Block(currentMb, mode, block_buffer));

	//���α���
	int delta[16] = { 0 };
	int max_value = pow(3, modifyEleNumber);
	int embedded_value = rand() % max_value;
	int eles[16] = { 0 };

	for (int i = 0; i < modifyEleNumber; i++)
	{
		int b8 = ele_pos[0][i] / 1000;
		int b4 = (ele_pos[0][i] % 1000) / 100;
		int m = (ele_pos[0][i] % 100) / 10;
		int n = ele_pos[0][i] % 10;
		int cofmatrix[4][4];

		Lr2Matrix(img->cofAC[b8][b4][0], img->cofAC[b8][b4][1], cofmatrix);
		eles[i] = cofmatrix[m][n];
	}

	int feature_value = get_feature_value(eles, modifyEleNumber);

	int d_value = embedded_value - feature_value;
	d_value = mymod(d_value, max_value);
	if (target_adapter(modifyEleNumber, delta, modifyEleNumber, d_value) == 0){ exit(1); }
	for (int i = 0; i < modifyEleNumber; i++)
	{
		eles[i] += delta[i];
	}
	feature_value = get_feature_value(eles, modifyEleNumber);

	for (int i = 0; i < modifyEleNumber; i++)
	{
		int b8 = ele_pos[0][i] / 1000;
		int b4 = (ele_pos[0][i] % 1000) / 100;
		int m0 = (ele_pos[0][i] % 100) / 10;
		int n0 = ele_pos[0][i] % 10;
		int m1 = (ele_pos[1][i] % 100) / 10;
		int n1 = ele_pos[1][i] % 10;
		int cofmatrix[4][4];

		Lr2Matrix(img->cofAC[b8][b4][0], img->cofAC[b8][b4][1], cofmatrix);
		cofmatrix[m0][n0] += delta[i];
		cofmatrix[m1][n1] -= delta[i];

		clearArr(img->cofAC[b8][b4][0], 16);
		clearArr(img->cofAC[b8][b4][1], 16);
		ZigZagScan(b8, b4, cofmatrix, img->cofAC[b8][b4]);	//��������level��run �Լ� �ɴ˴������µĽ�������
	}
	//***************************************
	FILE *fp = fopen("embedded.txt", "a");
	fprintf(fp, "%d\n", embedded_value);
	fclose(fp);
	//***************************************

	double bits = log(max_value) / log(2);
	sum += bits;
	printf("canNumber = %d, %.2f, %.2f\n", modifyEleNumber, bits, sum);
}

static int filterEles(int ele_pos[2][50], int buffer[], int blockNumber)
{
	int eleNumber = 0;

	for (int pairNum = 0; pairNum < 3; pairNum++)
	{
		for (int i = 0; i < blockNumber; i++)
		{
			int b8 = buffer[i] / 100;
			int b4 = (buffer[i] % 100) / 10;
			int conditions = buffer[i] % 10;
			int mn[2][2] = { 0 };


			mn[0][0] = pairSets[conditions][pairNum][0];
			mn[0][1] = pairSets[conditions][pairNum][1];
			mn[1][0] = pairSets[conditions][pairNum][2];
			mn[1][1] = pairSets[conditions][pairNum][3];

			ele_pos[0][eleNumber] = b8 * 1000 + b4 * 100 + mn[0][0] * 10 + mn[0][1];
			ele_pos[1][eleNumber] = b8 * 1000 + b4 * 100 + mn[1][0] * 10 + mn[1][1];
			eleNumber += 1;
			if (eleNumber >= maxModifyEleNumber)
			{
				return eleNumber;
			}
		}
	}

	return eleNumber;
}

//ɸѡ����ǰ����п����޸ĵ�4x4�飬�Լ��������Ƕ��������
static int filter4x4Block(Macroblock *currentMb, int mode[4][4], int buffer[])
{
	int canModifyNumber = 0;
	int cbp = currentMb->cbp;

	for (int b8 = 0; b8 < 4; b8++)
	{
		if (!(cbp & (1 << b8))){ continue; }
		for (int b4 = 0; b4 < 4; b4++)
		{
			int cofmatrix[4][4];
			int m = B8b42M(b8, b4);		//����λ�ڵġ��С�
			int n = B8b42N(b8, b4);		//����λ�ڵġ��С�

			Lr2Matrix(img->cofAC[b8][b4][0], img->cofAC[b8][b4][1], cofmatrix);

			if ((cofmatrix[0][0] == 0) && isSkipNoDC){ continue; }				//����

			if (m < 3 && (n == 1 || n == 2))
			{//part 1
				if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
				{	//��������1
					buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 0;
					canModifyNumber++;
				}
				else if ((ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
					&& (ArrayContains(mode[m + 1][n - 1], under_left_block_mode, 7) == 1))
				{	//��������2;
					buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 1;
					canModifyNumber++;
				}
			}
			else if (n == 0 & m != 3)
			{//part 2
				if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
				{	//��������1
					buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 0;
					canModifyNumber++;
				}
				else if (ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
				{	//��������2�����ÿ������µĿ飬��Ϊ�ǿ϶������õ���ǰ4x4���(�������¿�ʱ����ǰ4x4�黹û����ģ����Կ϶������õ�)
					buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 1;
					canModifyNumber++;
				}
			}
			else if (n == 3 && m != 3)
			{//part 3
				if ((ArrayContains(mode[m + 1][n], under_block_mode, 2) == 1)
					&& (ArrayContains(mode[m + 1][n - 1], under_left_block_mode, 7) == 1))
				{	//��������2
					buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 1;
					canModifyNumber++;
				}
			}
			else if (m != 3)
			{
				if (ArrayContains(mode[m][n + 1], right_block_mode, 3) == 1)
				{	//��������1
					buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 0;
					canModifyNumber++;
				}
			}
			else if (m == 3 && n == 3)
			{
				//buffer[canModifyNumber] = b8 * 100 + b4 * 10 + 0;
				//canModifyNumber++;
			}
		}
	}

	return canModifyNumber;
}

void LoadMode(int mode[4][4])
{
	int **ipredmode4 = img->ipredmode;
	for (int b8 = 0; b8 < 4; b8++)
	{
		for (int b4 = 0; b4 < 4; b4++)
		{
			int block4_x = (8 * (b8 % 2) + 4 * (b4 % 2));//++ ��ǰ4*4�����Ͻ������ڱ�����ڵĺ����꣨������Ϊ��λ��
			int block4_y = (8 * (b8 / 2) + 4 * (b4 / 2));//++ ��ǰ4*4�����Ͻ������ڱ�����ڵ������꣨������Ϊ��λ��
			int pic_pix_x = img->pix_x + block4_x;
			int pic_pix_y = img->pix_y + block4_y;
			int pic_block4_x = pic_pix_x / 4;
			int pic_block4_y = pic_pix_y / 4;

			mode[block4_y / 4][block4_x / 4] = ipredmode4[pic_block4_x][pic_block4_y];
		}
	}
}
#include "event.h"
#include "util.h"
#include "util264.h"
#include "hide.h"

void Event_Start()
{

}

void Event_End()
{
	printf("\n-------------------------------------------------End-------------------------------------------------\n");
	while (1);
}

void Event_WriteCoeff4x4_CAVLC(int block_type, int *pLevel, int *pRun)
{
	//ֻ�������4x4����
	if (block_type == LUMA)
	{
		/*
		int coeff[16];
		Lr2Zz(pLevel, pRun, coeff);
		StatisticsLNZIndex(coeff);
		StatisticsLNZ(coeff, 0);
		*/
	}
}

extern InputParameters *input;			//�������
/*
���һ����Ѱ����������ᴥ��һ��
��Ҫע����ǣ������޸ĵ��˶�ʸ��ֻ��Է�skip�Ŀ�������Ӵ
���������ģʽ��ѡ������ģʽʱ���ǻ��漰skipģʽ��Ӵ
*/
void Event_MvSearch(int block_x,		//��Ѱ���ڵ�ǰ������ʼλ�ã�4x4��λ
	int block_y,		//��Ѱ���ڵ�ǰ������ʼλ��
	int list,
	int ref,
	int blocktype,
	int****** all_mv)
{
	int       bsx = input->blc_size[blocktype][0];	//��Ѱ���С, ���ص�λ
	int       bsy = input->blc_size[blocktype][1];	//��Ѱ���С, ���ص�λ

	if (blocktype != 7)
	{
		return ;
	}
	/*
	for (int i = 0; i < (bsx >> 2); i++)
	{
		for (int j = 0; j < (bsy >> 2); j++)
		{
			int mv_x = all_mv[block_x + i][block_y + j][list][ref][blocktype][0];
			int mv_y = all_mv[block_x + i][block_y + j][list][ref][blocktype][1];
		}
	}
	*/
}

void Event_FinishOneMac()
{
	truehide();
	//hide2();
}
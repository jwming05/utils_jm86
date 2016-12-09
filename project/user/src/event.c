#include "event.h"

void Event_Start()
{

}

void Event_End()
{
	printf("\n-------------------------------------------------End-------------------------------------------------\n");
	StatisticsLNZIndex(NULL);			//��null��Ϊ����ʾ���ݣ��䲻���κδ���ġ�
	printf("have embedded %d bit\n", wm_get_instance()->length);
	while (1);
}

void Event_WriteCoeff4x4_CAVLC(int block_type, int *pLevel, int *pRun)
{
	//ֻ�������4x4����
	if (block_type == LUMA)
	{
		int coeff[16];
		Lr2Zz(pLevel, pRun, coeff);
		StatisticsLNZIndex(coeff);
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

	for (int i = 0; i < (bsx >> 2); i++)
	{
		for (int j = 0; j < (bsy >> 2); j++)
		{
			int mv_x = all_mv[block_x + i][block_y + j][list][ref][blocktype][0];
			int mv_y = all_mv[block_x + i][block_y + j][list][ref][blocktype][1];
			
			int bit = wm_get_instance()->read_one_bit(wm_get_instance());

			if (mv_x & 0x01)
			{
				mv_x++;
			}

			if (mv_y & 0x01)
			{
				mv_y++;
			}

			all_mv[block_x + i][block_y + j][list][ref][blocktype][0] = mv_x;
			all_mv[block_x + i][block_y + j][list][ref][blocktype][1] = mv_y;
		}
	}
}
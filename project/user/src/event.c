#include "event.h"

void Event_Start()
{

}

void Event_End()
{
	printf("\n-------------------------------------------------End-------------------------------------------------\n");
	StatisticsLNZIndex(NULL);			//传null是为了显示数据，其不做任何处理的。
	printf("have embedded %d bit\n", wm_get_instance()->length);
	while (1);
}

void Event_WriteCoeff4x4_CAVLC(int block_type, int *pLevel, int *pRun)
{
	//只针对亮度4x4块做
	if (block_type == LUMA)
	{
		int coeff[16];
		Lr2Zz(pLevel, pRun, coeff);
		StatisticsLNZIndex(coeff);
	}
}

extern InputParameters *input;			//输入参数
/*
完成一个搜寻块的搜索，会触发一次
需要注意的是，这里修改的运动矢量只会对非skip的块起作用哟
在最后所有模式中选择最优模式时，是会涉及skip模式的哟
*/
void Event_MvSearch(int block_x,		//搜寻块在当前宏块的起始位置，4x4单位
	int block_y,		//搜寻看在当前宏块的起始位置
	int list,
	int ref,
	int blocktype,
	int****** all_mv)
{
	int       bsx = input->blc_size[blocktype][0];	//搜寻块大小, 像素单位
	int       bsy = input->blc_size[blocktype][1];	//搜寻块大小, 像素单位

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
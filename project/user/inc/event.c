#include "event.h"

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

void Event_MvSearch()
{

}
#include "event.h"

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

void Event_MvSearch()
{

}
#ifndef __EVENT_H
#define __EVENT_H

#include "util.h"
#include "defines.h"

void Event_WriteCoeff4x4_CAVLC(int block_type, int *pLevel, int *pRun);
void Event_MvSearch();

#endif
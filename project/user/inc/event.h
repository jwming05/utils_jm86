#ifndef __EVENT_H
#define __EVENT_H

#include "global.h"
#include "util.h"
#include "defines.h"
#include "watermark.h"

void Event_Start();
void Event_End();
void Event_WriteCoeff4x4_CAVLC(int block_type, int *pLevel, int *pRun);
void Event_MvSearch(int block_x, int block_y, int list, int ref, int blocktype, int****** all_mv);

#endif
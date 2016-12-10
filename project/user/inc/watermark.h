#ifndef __WATERMARK_H
#define __WATERMARK_H

#include "util.h"

typedef struct
{
	int bin;
	int last_bin;
	int length;

	void(*load_one_bit)(void* const self);
	int(*read_one_bit)(void* const self);
	void (* recover_one_bit)(void* const self);
}Watermark;

Watermark *wm_get_instance();
Watermark* init_chunk();
void load_one_bit(Watermark* const self);
int read_one_bit(Watermark* const self);
void recover_one_bit(Watermark* const self);

#endif
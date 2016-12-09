#include "watermark.h"

Watermark *wm_get_instance()
{
	static Watermark* unique_instance = NULL;

	if (unique_instance == NULL)
	{
		unique_instance = init_chunk();
	}

	return unique_instance;
}

Watermark* init_chunk()
{
	Watermark *chunk = NULL;
	
	chunk = (Watermark *)malloc(sizeof(Watermark) * 1);
	chunk->length = 0;
	chunk->bin = -1;
	chunk->last_bin = -1;
	chunk->read_one_bit = read_one_bit;
	chunk->recover_one_bit = recover_one_bit;

	return chunk;
}

int read_one_bit(Watermark* const self)
{
	int wm = rand() & 0x01;
	self->last_bin = self->bin;
	self->bin = wm;
	self->length++;
	return wm;
}


void recover_one_bit(Watermark* const self)
{
	if (self->length == 0 || self->last_bin == -1)
	{
		return;
	}

	self->length--;
	self->bin = self->last_bin;
	self->last_bin = -1;
}
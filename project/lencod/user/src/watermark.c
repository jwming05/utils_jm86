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
	return self->bin;
}

void next_one_bit(Watermark* const self)
{
	self->length++;
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

void load_one_bit(Watermark* const self)
{
	int wm = rand() & 0x01;
	self->bin = wm;
}
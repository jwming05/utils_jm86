#include "hide.h"


int macLevel[4][4][16];
int macRun[4][4][16];

void hide2(struct img_par *img)
{
	static int bits = 0;
	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		Macroblock *currentMb = &img->mb_data[img->current_mb_nr];
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int *level = macLevel[b8][b4];
				int *run = macRun[b8][b4];
				int coeff[16];

				Lr2Zz(level, run, coeff);
				int index = LNZIndex(coeff);
				if (index >= 9)
				{
					bits++;
					int bit = isEven(coeff[index]);

					FILE *fp = fopen("t2.txt", "a");
					fprintf(fp, "%d %d %d %d : ", img->current_mb_nr, b8, b4, bit);
					for (int i = 0; i < 16; i++)
					{
						fprintf(fp, "\t%d", coeff[i]);
					}
					fprintf(fp, "\n");
					fclose(fp);
				}

				for (int i = 0; i < 16; i++)
				{
					level[i] = 0;
					run[i] = 0;
				}
			}
		}
	}
	printf("bits = %d\n", bits);
}




void hide3(struct img_par *img)
{
	static int bits = 0;
	if (img->mb_data[img->current_mb_nr].mb_type == I4MB)
	{
		Macroblock *currentMb = &img->mb_data[img->current_mb_nr];
		for (int b8 = 0; b8 < 4; b8++)
		{
			for (int b4 = 0; b4 < 4; b4++)
			{
				int *level = macLevel[b8][b4];
				int *run = macRun[b8][b4];
				int coeff[16];

				Lr2Zz(level, run, coeff);
				int index = LNZIndex(coeff);

				if (index >= 9)
				{
					bits++;
					int bit = isEven(index);

					FILE *fp = fopen("t2.txt", "a");
					fprintf(fp, "%d %d %d %d : ", img->current_mb_nr, b8, b4, bit);
					for (int i = 0; i < 16; i++)
					{
						fprintf(fp, "\t%d", coeff[i]);
					}
					fprintf(fp, "\n");
					fclose(fp);
				}

				for (int i = 0; i < 16; i++)
				{
					level[i] = 0;
					run[i] = 0;
				}
			}
		}
	}
	printf("bits = %d\n", bits);
}

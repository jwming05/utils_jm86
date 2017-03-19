#include "event.h"
#include "hide.h"

void Event_End()
{
	printf("-------------------------------End-------------------------------");
	while (1);
}

void Event_BeforeOneMac(struct img_par *img)
{
	hide2(img);
}
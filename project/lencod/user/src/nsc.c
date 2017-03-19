#include "nsc.h"
#include "util.h"

int get_feature_value(int *g, int n)
{
	int feature_value = 0;
	int max_value = pow(3, n);
	for (int i = 0; i < n; i++)
	{
		feature_value += pow(3, i) * g[i];
	}
	feature_value = mymod(feature_value, max_value);
	return feature_value;
}

int target_adapter(int n, int *delta, int max_n, int target)
{
	if (n == 0)
	{
		int val = get_feature_value(delta, max_n);
		if (target == val)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		for (int i = -1; i <= 1; i++)
		{
			delta[n - 1] = i;
			if (target_adapter(n - 1, delta, max_n, target) == 1)
			{
				return 1;
			}
		}
		return 0;
	}
}

#ifndef __REL_SPEED__
#define __REL_SPEED__

//iFit_Line()
Bool iFit_Line(short *piPointy, int iPointCnt, float *a, float *b, float *err1)
{
//	long long A = 0;
//	long long B = 0;
//	long long C = 0;
	long long D = 0;
	long long E = 0;
	long long X2 =0;
	long long X = 0;
	long long Y = 0;
//	long long X4 = 0;
//	long long X3 = 0;
//	long long YX2 = 0;
	long long XY = 0;
//	long long dis = 0;
	long long sum1 = 0;
	int dis;
	int i;
	short x,y;
	double aTemp,bTemp;

	*a = 0;
	*b = 0;

	if (iPointCnt < 10)
	{
		return FALSE;
	}

	for(i=0; i<iPointCnt; i++)
	{
		sum1 += piPointy[i];
	}
	sum1 /= iPointCnt;

	for( i=0; i<iPointCnt; i++)
	{
		x = i;
		y = piPointy[i] - sum1;

		X2 += (long long)(x)*(long long)(x);
		Y += y;
		X += x;
		XY += x*y;
	}

	D = (X2 - X*X/iPointCnt);
	E = (XY - Y*X/iPointCnt);

	aTemp = (double)E / (double)D;
//	aTemp = - aTemp;
	bTemp = (Y - aTemp * X)/iPointCnt;
	bTemp += sum1;

	*a = aTemp;
	*b = bTemp;

///* //根据计算误差判断该拟合
	sum1 = 0;
	for(i = 0; i < iPointCnt; i++)
	{
		x = i;
		y = piPointy[i];
		dis = abs((*a)*x+(*b)-y);

		sum1 += dis;
	}
/*
	*err1 = (float)sum1/iPointCnt;
	if (*err1 > 70)
	{
		*a = 0;
		*b = 0;
		return FALSE;
	}
//*/
	return TRUE;
}

#endif



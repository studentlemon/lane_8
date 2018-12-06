// Measure.cpp: implementation of the CMeasure class.
//
//////////////////////////////////////////////////////////////////////

//#define PI 3.1415926
#define HALF_PI 1.57079632679f
#define LANE_ANGLE_TOLERENCE PI/180
#define LANE_WIDTH_WORLD 3600
#define LWD_WARNING_ENABLE_SPEED 16666			// (int)(60*10000/36)
#define LWD_WORKING_ENABLE_SPEED 8333			// (int)(30*10000/36)
#define LWD_MAX_ACCElERATE 300					// Max acceleration in frame in mm
//#define LWD_MAX_LANE_ANGLE (4.5/180*PI)



//20140311-6
// /*
float m_fVanishPoint[2] = {379.340, 207.797};
int Shift_C2W_4096[2] = {-7778304, 0};
int RC2W20Focal = -131376;
int ShiftC2WFocal = 230307840;
int Rotate_C2W[3][3] = {{4095, 55, 92},{-55, 4096, -1},{-92, 0, 4095}};
int Rotate_W2C[3][3] = {{4095, -55, -92},{55, 4096, 0},{92, -1, 4095}};
int Shift_C2W[3] = {-1899, 0, 1260};
int Focal[2] = {1428, 1428};
int Center[2] = {360, 240};
int iRowNum = 479;
// */

int m_iLfDist;
int m_iRtDist;
float m_fLfAngle;
float m_fRtAngle;
int m_iFinalLfDist;
int m_iCurLaneWidth;
int m_iLaneWidthBuff[32];
int m_iLaneWidthBuffIndex;
int m_iLaneWidthBuffCnt;
float m_fFinalAngle;
int m_iCurLaneWidthStdDev;

int m_iProcRowCnt = 200;

LineFunc m_WLfLine;	// 左侧线世界坐标	角度是车道线诔堤屮标系内坐标，车头向左为负，车头向右为正
LineFunc m_WRtLine;
LineFunc m_WMdLine;
LineFunc m_PRefLfLine;
LineFunc m_PRefRtLine;

LineFunc m_NormalLineLeft;
LineFunc m_NormalLineRight;

//函数声明
void AlterLineFunc(LineFunc* line1, LineFunc* line2);

//Point_P2W_I() -U
Bool Point_P2W_I(int i, int j, int* xw, int* yw)
{    
	int xc,yc,zc;
	int p = i-Center[0];																		
	//int q = Center[1]-j;	
	int q=j-Center[1];    //2010-03-15

//	xc = -(ShiftC2WFocal)/((RC2W20Focal+Rotate_C2W[2][1]*p+Rotate_C2W[2][2]*q)/32);
	xc = -(ShiftC2WFocal*2)/((RC2W20Focal+Rotate_C2W[2][1]*p+Rotate_C2W[2][2]*q)/16);
	yc = p*xc/Focal[0];
	zc = q*xc/Focal[0];
	*xw = (Rotate_C2W[0][0]*xc+Rotate_C2W[0][1]*yc+Rotate_C2W[0][2]*zc+Shift_C2W_4096[0])/4096;
	*yw = -(Rotate_C2W[1][0]*xc+Rotate_C2W[1][1]*yc+Rotate_C2W[1][2]*zc+Shift_C2W_4096[1])/4096;
	return TRUE;
}

//Point_W2P_I() -U
Bool Point_W2P_I(int x, int y, int* xp, int * yp)
{
	int xc, yc, zc;

    y = -y;  //2010-03-18
	xc = Rotate_W2C[0][0]*x+Rotate_W2C[0][1]*y-Rotate_W2C[0][0]*Shift_C2W[0]-Rotate_W2C[0][1]*Shift_C2W[1]-Rotate_W2C[0][2]*Shift_C2W[2];
	yc = Rotate_W2C[1][0]*x+Rotate_W2C[1][1]*y-Rotate_W2C[1][0]*Shift_C2W[0]-Rotate_W2C[1][1]*Shift_C2W[1]-Rotate_W2C[1][2]*Shift_C2W[2];
	zc = Rotate_W2C[2][0]*x+Rotate_W2C[2][1]*y-Rotate_W2C[2][0]*Shift_C2W[0]-Rotate_W2C[2][1]*Shift_C2W[1]-Rotate_W2C[2][2]*Shift_C2W[2];
	
	*xp = yc/128*Focal[0]/(xc/128)+Center[0];
	*yp = (zc/128)*Focal[1]/(xc/128)+Center[1];
	
	return TRUE;
}

//LineAngle_I() -U
Bool LineAngle_I(CLine lw, float* a)
{
	if (lw.x2 == lw.x1 && lw.y1 == lw.y2)
		return FALSE;
	if (lw.x2 == lw.x1 && lw.y1 != lw.y2)
	{
		*a = HALF_PI;
		return TRUE;
	}
	*a = (float)atan((float)(lw.y2-lw.y1)/(float)(lw.x2-lw.x1));
	return TRUE;
}


//LOffset_W_I() -U
Bool LOffset_W_I(CLine lw, int* d)
{
	if (lw.x2 == lw.x1)
	{
		*d = (lw.x2);
		return TRUE;
	}
	*d = (int)(lw.y2*(lw.x2-lw.x1)-(lw.y2-lw.y1)*lw.x2)/16/sqrt((lw.y2-lw.y1)/16*(lw.y2-lw.y1)/16+(lw.x2-lw.x1)/16*(lw.x2-lw.x1)/16);
	return TRUE; 

}

void ClearLineFunc(LineFunc* pLinefunc)
{
	pLinefunc->b = 0;
	pLinefunc->k = 0;
	pLinefunc->bValidate = 0;
	pLinefunc->EPoint.x = 0;
	pLinefunc->EPoint.y = 0;
	pLinefunc->SPoint.x = 0;
	pLinefunc->SPoint.y = 0;
	pLinefunc->trust = 0;


	pLinefunc->degree = 0;
	pLinefunc->PointCnt = 0;
	pLinefunc->iFailure = 0;
	pLinefunc->RefOtherSide = 0;
	pLinefunc->bVert = 0;
	pLinefunc->FlagWarning = 0;
}

//CLDWMeasure()
void CLDWMeasure(void)
{
	m_iLfDist = 0;
	m_iRtDist = 0;
	m_fLfAngle = 0;
	m_fRtAngle = 0;
	m_iLaneWidthBuffCnt = 0;
	m_iLaneWidthBuffIndex = 0;
	m_iCurLaneWidth = LANE_WIDTH_WORLD;
	m_fFinalAngle = 0;
	m_iFinalLfDist = 0;	
	ClearLineFunc(&m_WLfLine);
	ClearLineFunc(&m_WRtLine);
	ClearLineFunc(&m_WMdLine);

}


/******************************************
函数名: GetParallelLine
//GetParallelLine()
注释: 在世界坐标系内计算临近车道线的直线方程，并转换到图像坐标系
输入:	wLine		世界坐标系内直线
		iDist		两条车道线的距离
输出:	pRefLine	图像坐标系内直线		
******************************************/
void GetParallelLine(LineFunc wLine, LineFunc* pRefLine, int iDist)
{
	CLine Line;
	int x, y;
	Line.x1 = wLine.SPoint.x;
	Line.y1 = wLine.SPoint.y+iDist;
	Line.x2 = wLine.EPoint.x;
	Line.y2 = wLine.EPoint.y+iDist;

	Point_W2P_I(Line.x1, Line.y1, &x, &y);
	pRefLine->SPoint.x = x;
	pRefLine->SPoint.y = y;

	Point_W2P_I(Line.x2, Line.y2, &x, &y);
	pRefLine->EPoint.x = x;
	pRefLine->EPoint.y = y;

	if(pRefLine->EPoint.x-pRefLine->SPoint.x ==0 )
	{
		pRefLine->bVert = TRUE;
	}
	else
	{
		pRefLine->k = (float)(pRefLine->EPoint.y-pRefLine->SPoint.y)/(pRefLine->EPoint.x-pRefLine->SPoint.x);
		pRefLine->b = pRefLine->EPoint.y-pRefLine->k*pRefLine->EPoint.x;
		pRefLine->bVert = FALSE;
	}
		
}

/******************************************
函数名: LineFuncP2W
//LineFuncP2W() -U
注释: 将图像坐标系内直线转换到世界坐标系，同时复制线的其他相关属性
输入:	linep	图像坐标系内直线
输出:	linew	世界坐标系内直线	
******************************************/
void LineFuncP2W(LineFunc linep, LineFunc* linew)
{
	int x, y;
	Point_P2W_I(linep.SPoint.x, linep.SPoint.y, &x, &y);
	linew->SPoint.x = x;
	linew->SPoint.y = y;
	Point_P2W_I(linep.EPoint.x, linep.EPoint.y, &x, &y);
	linew->EPoint.x = x;
	linew->EPoint.y = y;
	
	linew->bValidate = linep.bValidate;
    linew->iFailure = linep.iFailure;
	linew->degree = linep.degree;
	linew->PointCnt = linep.PointCnt;
	linew->trust = linep.trust;
}

Bool GetLineEndPoint(LineFunc* linefunc, CRect rect)
{
	float x,y;
	y = (float)rect.top;
	x = (y-linefunc->b)/linefunc->k;
	if(x>rect.right)
	{
		linefunc->EPoint.x = rect.right;
		linefunc->EPoint.y = (int)(rect.right*linefunc->k+linefunc->b);
	}
	else if(x<rect.left)
	{
	
		linefunc->EPoint.x = rect.left;
		linefunc->EPoint.y = (int)(rect.left*linefunc->k+linefunc->b);
	}
	else
	{
	
		linefunc->EPoint.x = (int)(x);
		linefunc->EPoint.y = rect.top;
	}

	y = (float)rect.bottom;
	x = (y-linefunc->b)/linefunc->k;
	if(x>rect.right)
	{
		linefunc->EPoint.x = rect.right;
		linefunc->EPoint.y = (int)(rect.right*linefunc->k+linefunc->b);
	}
	else if(x<rect.left)
	{
	
		linefunc->EPoint.x = rect.left;
		linefunc->EPoint.y = (int)(rect.left*linefunc->k+linefunc->b);
	}
	else
	{
	
		linefunc->EPoint.x = (int)(x);
		linefunc->EPoint.y = rect.bottom;
	}
	return TRUE;
}


/******************************************
函数名: LaneMeasure
//LaneMeasure()
注释: 测量入口函数

输入:	lineLf，lineRt	需要重建的图像坐标系内车道线，lineLf，lineRt为探测峁?
		bCrossLine		当前车道线是否在图像中部
		iCrossLineNum	如果车道线在图像中部，该值代表属于左侧或右侧，0-表示左侧，1-表示右侧
输出:	m_iFinalLfDist	最终的左侧车道线重建后距离
		m_fFinalAngle	最终的车道线重建后角度，车头向左该值为负数，车头向右该值为正数

返回值：距离车体较近的车道线编号，返回1-表示左侧距离近，返回2-表示右侧距离近
******************************************/
int LaneMeasure(LineFunc* lineLf, LineFunc* lineRt, Bool bCrossLine, int iCrossLineNum)
{
	CLine lw;
	int iNearestMark = 0;
	CRect RectDraw;
	float dis_cross;
	
	// 规定画图范围,
	RectDraw.left = 1;
	RectDraw.top = m_iProcRowCnt; //200;
	RectDraw.right = 639;
	RectDraw.bottom = 1;	
	
	m_WLfLine.trust = 0;
	m_WRtLine.trust = 0;

//	if(lineLf->trust)
    if(lineLf->bValidate==2)
	{
		LineFuncP2W(*lineLf, &m_WLfLine);
		lw.x1 = m_WLfLine.SPoint.x;
		lw.y1 = m_WLfLine.SPoint.y;
		lw.x2 = m_WLfLine.EPoint.x;
		lw.y2 = m_WLfLine.EPoint.y;
		LineAngle_I(lw, &m_fLfAngle);
		LOffset_W_I(lw, &m_iLfDist);

	    m_WLfLine.k = m_fLfAngle;
		m_WLfLine.b = m_WLfLine.SPoint.y - m_WLfLine.k * m_WLfLine.SPoint.x;
	
		if(m_iLfDist>LANE_WIDTH_WORLD)
		{
			m_WLfLine.trust = 0;
			lineLf->bValidate = 0; //解决识别另外车道的车道线
		}
		else
		{
			if(lineLf->trust)
			{
				m_WLfLine.trust = 1;
			}
		}
	}

	if(lineRt->bValidate==2)
	{
		LineFuncP2W(*lineRt, &m_WRtLine);
		lw.x1 = m_WRtLine.SPoint.x;
		lw.y1 = m_WRtLine.SPoint.y;
		lw.x2 = m_WRtLine.EPoint.x;
		lw.y2 = m_WRtLine.EPoint.y;
		LineAngle_I(lw, &m_fRtAngle);
		LOffset_W_I(lw, &m_iRtDist);

		m_WRtLine.k = m_fRtAngle;
		m_WRtLine.b = m_WRtLine.SPoint.y - m_WRtLine.k * m_WRtLine.SPoint.x;
		if(m_iRtDist<-LANE_WIDTH_WORLD)
		{
			m_WRtLine.trust = 0;
			lineRt->bValidate = 0; // 解决识别另外车道的车道线
		}
		else
		{
			if (lineRt->trust)
			{
				m_WRtLine.trust = 1;
			}
		}

	}
	
	if(bCrossLine)  // 如果在中间
	{
		if (iCrossLineNum == 0)  // 如果是左侧线
		{
			m_fFinalAngle = m_fLfAngle;
			m_iFinalLfDist = m_iLfDist;
			m_WRtLine.trust = 0;
			iNearestMark = 1;
		}
		else     	// 如果是右侧线
		{
			m_fFinalAngle = m_fRtAngle;
			m_iFinalLfDist = m_iRtDist+LANE_WIDTH_WORLD;
			m_WLfLine.trust = 0;
			iNearestMark = 2;
		}
		return iNearestMark;
	}

	if (lineLf->bValidate == 2 && lineRt->bValidate == 2)
	{
		//计算两条车道线的交点，主要是计算交点距离
		dis_cross = 100;
		if (fabs(m_WRtLine.k - m_WLfLine.k) > 0.00001)
		{
			dis_cross = (m_WLfLine.b - m_WRtLine.b) / fabs(m_WRtLine.k - m_WLfLine.k);
		}
		if (dis_cross < 25000)
		{
			if (lineLf->degree > lineRt->degree)
			{
				lineRt->trust = 0;
				lineRt->bValidate = 0;
				m_WRtLine.trust = 0;
			}
			else if (lineRt->degree > lineLf->degree)
			{
				lineLf->trust = 0;
				lineLf->bValidate = 0;
				m_WLfLine.trust = 0;
			}
		}
	}

	if (m_WLfLine.trust && m_WRtLine.trust)  	// 如果两条线均可信,根据测量结果选择或计算最终测量结果
	{
	    // 更新宽度信息
		m_iCurLaneWidth = UpdateBuff(m_iLaneWidthBuff, 32, abs(m_iLfDist-m_iRtDist), &m_iLaneWidthBuffIndex, &m_iLaneWidthBuffCnt);
	
		if((abs(m_iLfDist-m_iRtDist)>(LANE_WIDTH_WORLD-OFFSET_LANE_WIDTH_WORLD)) && (abs(m_iLfDist-m_iRtDist)<(LANE_WIDTH_WORLD+OFFSET_LANE_WIDTH_WORLD)))		// 如果宽度符合要求
		{
			if(fabs(m_fLfAngle-m_fRtAngle)<LANE_ANGLE_TOLERENCE) 	// 如果平行度符合要求
			{
				m_fFinalAngle = (m_fLfAngle+m_fRtAngle)/2;
				m_iFinalLfDist = (m_iLfDist+m_iRtDist)/2+LANE_WIDTH_WORLD/2;
				
			}
			else   	// 如果平行度不符合要求
			{
				m_fFinalAngle = abs(m_iLfDist)>abs(m_iRtDist)?m_fRtAngle:m_fLfAngle;
				m_iFinalLfDist = abs(m_iLfDist)>abs(m_iRtDist)?m_iRtDist+LANE_WIDTH_WORLD:m_iLfDist;
			}
		}
		else  // 如果宽度不符合要求
		{
			AlterLineFunc(&m_WLfLine, &m_WRtLine);
			if(m_WLfLine.trust)
			{
				m_iFinalLfDist = m_iLfDist;
				m_fFinalAngle = m_fLfAngle;
				//iNearestMark = 1;
			}
			else
			{
				m_iFinalLfDist = m_iRtDist+LANE_WIDTH_WORLD;
				m_fFinalAngle = m_fRtAngle;
				//iNearestMark = 2;
			}
		}
	}
	
	else if (m_WLfLine.trust && !m_WRtLine.trust) 	// 如果只有一条车道线可信
	{
		m_iFinalLfDist = m_iLfDist;
		m_fFinalAngle = m_fLfAngle;
        if((!lineRt->bValidate && !lineRt->RefOtherSide) /*|| lineRt->iFailure < 15*/)					// 如果跟踪无效并且非反推状态
		{
			if (m_iCurLaneWidth > LANE_WIDTH_WORLD-OFFSET_LANE_WIDTH_WORLD && m_iCurLaneWidth < LANE_WIDTH_WORLD + OFFSET_LANE_WIDTH_WORLD)
			{
				GetParallelLine(m_WLfLine, lineRt, -m_iCurLaneWidth);
			}
			else
			{
				GetParallelLine(m_WLfLine, lineRt, -LANE_WIDTH_WORLD);
			}
			GetLineEndPoint(lineRt, RectDraw);
			
			if (lineRt->EPoint.y - lineRt->SPoint.y < 10)
			{
				lineRt->bValidate = 0;
			}
			else
			{
				lineRt->bValidate = 1;
			}
			
			lineRt->RefOtherSide = 1;
		}
	}
	else if (!m_WLfLine.trust && m_WRtLine.trust)
	{
		m_iFinalLfDist = m_iRtDist+LANE_WIDTH_WORLD;
		m_fFinalAngle = m_fRtAngle;

        if((!lineLf->bValidate && !lineLf->RefOtherSide) /*|| lineLf->iFailure < 15*/)
		{

			if (m_iCurLaneWidth > LANE_WIDTH_WORLD-OFFSET_LANE_WIDTH_WORLD && m_iCurLaneWidth < LANE_WIDTH_WORLD + OFFSET_LANE_WIDTH_WORLD)
			{
				GetParallelLine(m_WRtLine, lineLf, m_iCurLaneWidth);
			}
			else
			{
				GetParallelLine(m_WRtLine, lineLf, LANE_WIDTH_WORLD);
			}

			GetLineEndPoint(lineLf, RectDraw);
			
			if (lineLf->EPoint.y - lineLf->SPoint.y < 10)
			{
				lineLf->bValidate = 0;
			}
			else
			{
				lineLf->bValidate = 1;
			}
			
			lineLf->RefOtherSide = 1;
		}
	}
	else
	{
		m_iCurLaneWidth = 0;
	}

	if (lineLf->iFailure > 15 || lineRt->iFailure > 15)
	{
		m_iCurLaneWidth = 0;
	}

	if(m_iFinalLfDist>LANE_WIDTH_WORLD/2)  	// 判断较近侧
	{
		iNearestMark = 2;
	}
	else
	{
		iNearestMark = 1;
	}
	return iNearestMark;
}

// 在两条线均为trust，且不符合结构条件时使用
//AlterLineFunc()
void AlterLineFunc(LineFunc* line1, LineFunc* line2)
{
	line1->trust = 0;
	line2->trust = 0;
	if(line1->degree>line2->degree)
	{
		line1->trust = 1;
		line2->trust = 0;
	}
	else if(line1->degree < line2->degree)
	{
		line1->trust = 0;
		line2->trust = 1;
	}
	else
	{
		if(line1->PointCnt>line2->PointCnt)
		{
			line1->trust = 1;
			line2->trust = 0;
		}
		else
		{
			line1->trust = 0;
			line2->trust = 1;
		}
	}
}

//GetCurLaneWidthDev()
int GetCurLaneWidthDev(int* buf, int iCnt, int iAverVal)
{
	int i;
	int sum = 0;
	for(i=0; i<iCnt; i++)
	{
		sum = sum+(buf[i]-iAverVal)*(buf[i]-iAverVal);
	}
	return (int)sqrt(sum/iCnt);
}

//GetParaLine()
void GetParaLine(LineFunc *lineLf, LineFunc *lineRt, Bool bCrossLine, int iCrossLineNum)
{
	LineFunc lineTmp;
	CRect RectDraw;

	lineTmp.bValidate = FALSE;
	
	// 规定画图范围,
	RectDraw.left = 1;
	//2012.09.24 与VC保持一致
	RectDraw.top = m_iProcRowCnt;  //(int)(480- m_fVanishPoint[1] - 0.5);//m_iProcRowCnt; //200;
	RectDraw.right = 719;
	RectDraw.bottom = 1;	

	if(((lineLf->bValidate == 2) && (lineRt->bValidate != 2)) || (bCrossLine && (iCrossLineNum == 0)))
	{
		LineFuncP2W(*lineLf, &lineTmp);
		
		GetParallelLine(lineTmp, lineRt, -LANE_WIDTH_WORLD);
		GetLineEndPoint(lineRt, RectDraw);
		lineRt->bValidate = 1;
		lineRt->RefOtherSide = 1;
		
	}
	else if(((lineLf->bValidate != 2) && (lineRt->bValidate == 2)) || (bCrossLine && (iCrossLineNum != 0)))
	{
		LineFuncP2W(*lineRt, &lineTmp);
		GetParallelLine(lineTmp, lineLf, LANE_WIDTH_WORLD);
		GetLineEndPoint(lineLf, RectDraw);
		lineLf->bValidate = 1;
		lineLf->RefOtherSide = 1;
	}
}

//LaneMeasureInit()
void LaneMeasureInit(void)
{
          
	int xp,yp;
    int wLeft_Sx = 5 * 1000;
	int wLeft_Sy = 1.875 * 1000;
	int wLeft_Ex = 55 * 1000;
	int wLeft_Ey = 1.875 * 1000;
	int wRight_Sx = 5 * 1000;
	int wRight_Sy = -1.875 * 1000;
	int wRight_Ex = 55 * 1000;
	int wRight_Ey = -1.875 * 1000;

	Point_W2P_I(wLeft_Sx,wLeft_Sy,&xp,&yp);
	m_NormalLineLeft.SPoint.x = xp;
	m_NormalLineLeft.SPoint.y = yp;

	Point_W2P_I(wLeft_Ex,wLeft_Ey,&xp,&yp);
	m_NormalLineLeft.EPoint.x = xp;
	m_NormalLineLeft.EPoint.y = yp;

	
	Point_W2P_I(wRight_Sx,wRight_Sy,&xp,&yp);
	m_NormalLineRight.SPoint.x = xp;
	m_NormalLineRight.SPoint.y = yp;

	Point_W2P_I(wRight_Ex,wRight_Ey,&xp,&yp);
	m_NormalLineRight.EPoint.x = xp;
	m_NormalLineRight.EPoint.y = yp;

}








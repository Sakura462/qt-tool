#pragma once
#include <iostream>
class PCPoint
{
public:
	PCPoint()
	{
		//初始化成员变量
		for (int i = 0; i < 3; i++)
		{
			m_Coordinate[i] = 0.0;
			m_Color[i] = 0.0;
		}
		m_Normal = NULL;
		m_ID = 0;
		b_BoundaryPoint = false;
		b_Selected = false;
		b_TVPoint = false;
	};


	~PCPoint()
	{
	};
public: //特性
	double m_Coordinate[3];//坐标值
	double m_Color[3];//颜色
	double* m_Normal;//法矢量
	int m_ID;//编号
	bool b_Selected;//是否被选择
	bool b_BoundaryPoint;//是否为边界点
	bool b_TVPoint;//张量投票特征点
};


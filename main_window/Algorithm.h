#pragma once

#include <QWidget>
#include "ui_Algorithm.h"

class Algorithm : public QWidget
{
	Q_OBJECT

public:
	Algorithm(QWidget* parent = Q_NULLPTR);
	~Algorithm();

private:
	Ui::Algorithm ui;

	void Statistical_Filter_set();
	void Bilateral_Filter_set();
	void Condition_Filter_set();
	void Radius_Filter_set();


	void Random_simplification_set();//随机精简
	void Isometry_simplification_set();//均匀精简
	void Voxel_Grid_simplification_set();//体素精简


	void Gaussian_set();
	void MLS_set();

	void IA_RANSC_set();
	void NDT_set();
	void ICP_set();

	void Cloud_Repair_set();

public slots:
	void Filter_page_set(int a);

signals://信号
	void sent_Str(int a, double b, double c, int d);//滤波信号
	void sent_simplification(int a, double b, double c, double d, int e);//精简信号
	void sent_Smooth(int a, int b, int c, double d, double e);//平滑信号
	void sent_registrarion(double a, double b, double c, double d, int e);//配准信号
	void sent_cloud_Repair();//修复信号



};

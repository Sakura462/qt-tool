#include "Algorithm.h"

Algorithm::Algorithm(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	/*******************************滤波*******************************************/
	QObject::connect(ui.Button_Statistical, &QPushButton::clicked, this, &Algorithm::Statistical_Filter_set);//按下按键
	QObject::connect(ui.Button_Bilateral, &QPushButton::clicked, this, &Algorithm::Bilateral_Filter_set);//按下按键
	QObject::connect(ui.Button_Condition, &QPushButton::clicked, this, &Algorithm::Condition_Filter_set);//按下按键
	QObject::connect(ui.Button_Radius, &QPushButton::clicked, this, &Algorithm::Radius_Filter_set);//按下按键

	/***********************************精简**************************************/
	QObject::connect(ui.Button_Random, &QPushButton::clicked, this, &Algorithm::Random_simplification_set);//按下按键
	QObject::connect(ui.Button_Isometry, &QPushButton::clicked, this, &Algorithm::Isometry_simplification_set);//按下按键
	QObject::connect(ui.Button_Voxel_Grid, &QPushButton::clicked, this, &Algorithm::Voxel_Grid_simplification_set);//按下按键

	/***********************************平滑**************************************/
	QObject::connect(ui.Button_MLS, &QPushButton::clicked, this, &Algorithm::MLS_set);//按下按键
	QObject::connect(ui.Button_Gaussian, &QPushButton::clicked, this, &Algorithm::Gaussian_set);//按下按键

	/***********************************配准**************************************/
	QObject::connect(ui.Button_RANSAC, &QPushButton::clicked, this, &Algorithm::IA_RANSC_set);//按下按键
	QObject::connect(ui.Button_NDT, &QPushButton::clicked, this, &Algorithm::NDT_set);//按下按键
	QObject::connect(ui.Button_ICP, &QPushButton::clicked, this, &Algorithm::ICP_set);//按下按键

	/***********************************点云修复**************************************/
	QObject::connect(ui.Button_Repair, &QPushButton::clicked, this, &Algorithm::Cloud_Repair_set);//按下按键


}

Algorithm::~Algorithm()
{
}

void Algorithm::Filter_page_set(int a)
{
	ui.stackedWidget->setCurrentIndex(a);//切换页面
}

/*******************滤波***************************/
void Algorithm::Statistical_Filter_set()//页面1
{
	int vaule = ui.MeanK_input->text().toInt();
	double vaule_1 = ui.StddevMulThresh_input->text().toDouble();
	emit sent_Str(vaule, vaule_1, 0, 0);//发一个信号到主界面
	this->close();
}
void Algorithm::Bilateral_Filter_set()//页面3
{
	double vaule0 = ui.dist_weigh_input->text().toDouble();
	int k = ui.K_set_input->text().toInt();
	double value1 = ui.normal_weigh_input->text().toDouble();
	emit sent_Str(k, vaule0, value1, 0);//发一个信号到主界面
	this->close();

}
void Algorithm::Condition_Filter_set()//页面2
{
	int index = ui.KeepOrganized_input->currentIndex();//获取当前索引
	emit sent_Str(0, 0, 0, index);//发一个信号到主界面
	this->close();
}
void Algorithm::Radius_Filter_set()//页面0
{
	double vaule0 = ui.RadiusSearch_input->text().toDouble();
	int value1 = ui.NeighborsInRadius_input->text().toInt();
	int  value2 = ui.negative_input->currentIndex();
	emit sent_Str(value1, vaule0, 0, value2);//发一个信号到主界面
	this->close();
}

/*******************精简***************************/
void Algorithm::Random_simplification_set()//随机精简
{

	double value0 = ui.setSample_input->text().toDouble();
	int  value1 = ui.setSeed_input->text().toInt();
	emit sent_simplification(0, value0, 0, 0, value1);//发一个信号到主界面
	this->close();

}
void Algorithm::Isometry_simplification_set()//均匀精简
{
	double value0 = ui.setRadiusSearch_input->text().toDouble();
	emit sent_simplification(0, value0, 0, 0, 0);//发一个信号到主界面
	this->close();
}
void Algorithm::Voxel_Grid_simplification_set()//体素精简
{
	double value0 = ui.setLeafSize_1_input->text().toDouble();
	double value1 = ui.setLeafSize_2_input->text().toDouble();
	double value2 = ui.setLeafSize_3_input->text().toDouble();
	int value3 = ui.setPerVoxel_input->text().toInt();
	emit sent_simplification(0, value0, value1, value2, value3);//发一个信号到主界面
	this->close();

}
/*******************平滑***************************/
void Algorithm::Gaussian_set()
{
	int value0 = ui.setSigma_input->text().toInt();
	int value1 = ui.setRelativeToSigma_input->text().toDouble();
	double value2 = ui.setThreshold_input->text().toDouble();
	int value3 = ui.setNumber_input->text().toInt();
	double value4 = ui.setRadius_input->text().toDouble();
	emit sent_Smooth(value0, value1, value3, value2, value4);//发一个信号到主界面
	this->close();

}

void Algorithm::MLS_set()
{
	int value0 = ui.setComputeNormals_input->currentIndex();
	int value2 = ui.setPolynomialOrder_input->text().toInt();
	double value3 = ui.setSearchRadius_input->text().toDouble();
	emit sent_Smooth(value0, 0, value2, value3, 0);//发一个信号到主界面
	this->close();
}

/*******************配准***************************/
void Algorithm::IA_RANSC_set()
{
	double value0 = ui.SetRadius_input->text().toDouble();
	double value1 = ui.SetRadius_input_2->text().toDouble();
	double value2 = ui.SetRadius_input_3->text().toDouble();
	double value3 = ui.SetRadius_input_4->text().toDouble();
	emit sent_registrarion(value0, value1, value2, value3, 0);//发一个信号到主界面
	this->close();
}

void Algorithm::NDT_set()
{
	double value0 = ui.setEpsilon_input->text().toDouble();
	double value1 = ui.setStepSize_input->text().toDouble();
	double value2 = ui.setResolution_input->text().toDouble();
	int value3 = ui.setMaxIterations_input->text().toInt();
	emit sent_registrarion(value0, value1, value2, 0, value3);//发一个信号到主界面
	this->close();


}

void Algorithm::ICP_set()
{
	double value0 = ui.setMaxCor_input->text().toDouble();
	double value1 = ui.setTrE_input->text().toDouble();
	double value2 = ui.setEuFit_input->text().toDouble();
	int value3 = ui.setMaxIteration_input->text().toInt();
	emit sent_registrarion(value0, value1, value2, 0, value3);//发一个信号到主界面
	this->close();


}

/*******************修复***************************/
void Algorithm::Cloud_Repair_set()
{
	emit sent_cloud_Repair();//发一个信号到主界面
	this->close();
}

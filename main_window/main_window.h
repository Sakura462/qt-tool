#pragma once

//QT部分 
#include <QtWidgets/QMainWindow>
#include <QString>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
//#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <QTextEdit>
#include <QTime>
#include <QMouseEvent> 
#include <QDesktopServices> 
#include <QUrl>
#include "ui_main_window.h"
#include "Algorithm.h"//页面二对应头文件

//C++语言部分
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
//注意这个头文件
#include <io.h>

//PCL部分
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/thread/thread.hpp>

#include <pcl/surface/mls.h>
#include <pcl/filters/convolution_3d.h> 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <pcl/features/boundary.h>
#include <pcl/surface/gp3.h>
// Visualization Toolkit (VTK)
// Visualization Toolkit (VTK)
#include <vtkOutputWindow.h>//为vtkOutputWindow提供头文件支
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkAutoInit.h>//初始化VTK模块
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

using namespace std;




typedef pcl::PointXYZRGBA PointT;//宏定义一个点数据结构
typedef pcl::PointCloud<PointT> PointCloudT;//宏定义一个点云数据结构
//static//初始化背景颜色数据结构体
static QString dark_qss = "QWidget{ background-color: rgb(60, 63, 65); }  QDockWidget{ 	color: rgb(208, 208, 208);	 	background-color: rgb(60, 63, 65);  	border-color: rgb(63, 63, 70); 	border-top-color: rgb(255, 255, 255);	 	font: 10pt \"Microsoft YaHei UI\"; }  QTableWidget{	 	background-color: rgb(43, 43, 43);	 	border-color: rgb(63, 63, 70); 	color: rgb(241, 241, 241); 	alternate-background-color: rgb(85, 85, 85); 	font: 9pt \"Microsoft YaHei UI\"; }  QTreeWidget{ 	background-color: rgb(43, 43, 43);	 	border-color: rgb(63, 63, 70); 	color: rgb(241, 241, 241); 	alternate-background-color: rgb(85, 85, 85); 	font: 9pt \"Microsoft YaHei UI\"; }   QHeaderView::section{ 	background-color: rgb(53, 53, 53); 	color: rgb(241, 241, 241); 	border:0px solid #E0DDDC; 	border-bottom:1px solid #262626; 	height: 30px; }   QToolBar{ 	background-color: rgb(60, 63, 65); 	border-bottom: 1px solid #262626; }  QStatusBar{ 	color: rgb(241, 241, 241); 	font: 9pt \"Microsoft YaHei UI\"; }  QMenuBar{ 	background-color: rgb(60, 63, 65); 	color: rgb(241, 241, 241); 	font: 9pt \"Microsoft YaHei UI\"; 	border-bottom: 1px solid #262626; }  QMenuBar::item:selected{ 	background-color: rgb(75, 110, 175); }  QMenu{ 	font: 9pt \"Microsoft YaHei UI\";	 	color: rgb(241, 241, 241); 	background-color: rgb(60, 63, 65); }  QMenu::item:selected{ 	background-color: rgb(75, 110, 175); }   QLabel{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QCheckBox{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QLCDNumber{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QPushButton{ 	color: rgb(241, 241, 241); 	 	background-color: rgb(73, 78, 80); } ";


//类定义
class MyPointCloud {
public:

	PointCloudT::Ptr cloud;
	int cloud_showUnm = 0;
	bool visible = true;

};

//窗口类定义
class main_window : public QMainWindow
{
	Q_OBJECT

public:
	main_window(QWidget* parent = Q_NULLPTR);

protected:

	//创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;//可视化窗口
	/*PCL visualizer可视化类允许用户在视窗中绘制一般图元，这个类常用于显示点云处理算法的可视化结果，例如 通过可视化球体
		包围聚类得到的点云集以显示聚类结果，shapesVis函数用于实现添加形状到视窗中，添加了四种形状：从点云中的一个点到最后一个点
		之间的连线，原点所在的平面，以点云中第一个点为中心的球体，沿Y轴的椎体*/
		//pcl::visualization::PCLVisualizer::Ptr boost_viewer;//可视化窗口

	std::vector< PointCloudT::Ptr> cloud_show;//点云数据容器

	void Initial();

	//打开文件函数
	void open();
	void getFileNames(string path, vector<string>& files);
	void myLoad();
	void cloud_type_change(PointCloudT::Ptr& cloud_Tag, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_SOR);
	std::string getFileName(std::string file_name);
	void setCloudRGB(PointCloudT::Ptr& cloud, unsigned int r, unsigned int g, unsigned int b);
	void setCloudAlpha(PointCloudT::Ptr& cloud, unsigned int a);
	void setConsole(QString operation, QString detail);//保存日志
	void updatePointcloud();
	std::string model_dirname = "C:\\Users\\User\\Desktop";//初始化的打开路径
	long total_points = 0; //Total amount of points in the viewer
	//保存文件
	void SaveFile();
	//清空点云
	void Clear();
	//退出
	void Exit();
	//更新资源树
	void updateDataTree();


	//Tool工具栏
				//De noise去噪滤波器
	void Radius_Filter();//半径滤波
	void Condition_Filter();//条件滤波
	void Statistical_Filter();//统计滤波
	void Bilateral_Filter();//双边滤波

	//精简simplification
	void Random_simplification();//随机精简
	void Isometry_simplification();//均匀精简
	void Voxel_Grid_simplification();//体素精简
	//配准
	void IA_RANSC();
	void NDT();
	void ICP();

	//平滑
	void Gaussian();
	void MLS();

	void Cloud_Repair();

	void Statistical_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		int set_meank, double set_stddev_multhresh);

	void Radius_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		int negative_select, double set_radius_search, int set_min_neighbors_in_radius);

	void Condition_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		int set_keep_organized);

	void Bilateral_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		double dist_weigh, int K, double normal_weigh);

	void Random_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		double set_sample, int set_seed);

	void Isometry_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		double set_radius_search);

	void Voxel_Grid_compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
		double set_leaf_size_x, double set_leaf_size_y,
		double set_leaf_size_z, int set_perVoxel);

	void Gaussian_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		int set_sigma, int set_relative_sigma,
		double set_threshold, int set_number_of_threads,
		double search_radius);

	void mls_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		int setComputeNormals,
		int polynomial_order,
		double search_radius);

	void IA_RANSC_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		double src_setRadius,
		double tgt_setRadius,
		double fpfh_src_setRadius,
		double fpfh_tgt_setRadius);

	void NDT_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
		double set_epsilon,
		double set_step_size,
		double set_resolution,
		int set_maximum_iterations);

	void ICP_compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
		double set_distance,
		double set_formation_epsilon,
		double set_fitness_epsilon,
		int set_maximum_iterations,
		double tx, double ty, double tz);
	/**************************工具栏结束********************************/

	//右键按钮函数
	void pointcolorChanged();
	void pointcolorRamdom();
	void pointHide();
	void pointShow();

private:
	Ui::main_windowClass ui;
	Algorithm* Algorithm_windw = NULL;//对应第二页类名
	MyPointCloud myPointCloud;
	long points_number = 0;


public slots://该区域声明的槽能与任意信号连接

	void itemSelected(QTreeWidgetItem* item, int count);//左键选择函数
	void itemPopmenu(const QPoint&);//右键菜单函数
	void set_Statistical_Filter(int a, double b, double c, int d);
	void set_Radius_Filter(int a, double b, double c, int d);
	void set_Condition_Filter(int a, double b, double c, int d);
	void set_Bilateral_Filter(int a, double b, double c, int d);

	void set_Random_simplification(int a, double b, double c, double d, int e);//随机精简
	void set_Isometry_simplification(int a, double b, double c, double d, int e);//均匀精简
	void set_Voxel_Grid_simplification(int a, double b, double c, double d, int e);//体素精简
	/********************平滑******************************/

	void set_Gaussian(int a, int b, int c, double d, double e);
	void set_MLS(int a, int b, int c, double d, double e);
	/********************配准******************************/
	void set_IA_RANSC(double a, double b, double c, double d, int e);
	void set_NDT(double a, double b, double c, double d, int e);
	void set_ICP(double a, double b, double c, double d, int e);
	/********************点云修复******************************/
	void set_Cloud_Repair();


signals://信号
	void sent_pageUnm(int a);//发送信号
};


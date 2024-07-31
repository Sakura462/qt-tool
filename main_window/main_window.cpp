#include "main_window.h"
#include "cloud_repair.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

main_window::main_window(QWidget* parent)
	: QMainWindow(parent)
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);//消除vtk版本警告
	ui.setupUi(this);
	total_points = 0;
	Initial();
	/***********************************文件操作***********************************/
//第一个参数是哪一个Qmenu 第二个参数什么信号，第三个参数那么一个界面，第四个参数，连接函数
	QObject::connect(ui.actionOpen, &QAction::triggered, this, &main_window::open);//打开点云文件
	QObject::connect(ui.actionLoad, &QAction::triggered, this, &main_window::myLoad);//加载
	QObject::connect(ui.actionSave, &QAction::triggered, this, &main_window::SaveFile);//保存点云文件
	QObject::connect(ui.actionClear, &QAction::triggered, this, &main_window::Clear);//清除点云文件
	QObject::connect(ui.actionExit, &QAction::triggered, this, &main_window::Exit);//退出
//文件树的左右键
	/***** Slots connection of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked (connect)
	QObject::connect(ui.dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelected(QTreeWidgetItem*, int)));
	// Item in dataTree is right-clicked
	QObject::connect(ui.dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(itemPopmenu(const QPoint&)));
	/***********************************工具栏***********************************/
		//De noise去噪滤波器
	QObject::connect(ui.actionRadius_Filter, &QAction::triggered, this, &main_window::Radius_Filter);//半径滤波
	QObject::connect(ui.actionCondition_Filter, &QAction::triggered, this, &main_window::Condition_Filter);//条件滤波
	QObject::connect(ui.actionStatistical_Filter, &QAction::triggered, this, &main_window::Statistical_Filter);//统计滤波
	QObject::connect(ui.actionBilateral_Filter, &QAction::triggered, this, &main_window::Bilateral_Filter);//双边滤波

	//精简simplification
	QObject::connect(ui.actionRandom, &QAction::triggered, this, &main_window::Random_simplification);//随机精简
	QObject::connect(ui.actionIsometry, &QAction::triggered, this, &main_window::Isometry_simplification);//均匀精简
	QObject::connect(ui.actionVoxel_Grid, &QAction::triggered, this, &main_window::Voxel_Grid_simplification);//体素精简
	//配准
	QObject::connect(ui.actionIA_RANSC, &QAction::triggered, this, &main_window::IA_RANSC);
	QObject::connect(ui.actionNDT, &QAction::triggered, this, &main_window::NDT);
	QObject::connect(ui.actionICP, &QAction::triggered, this, &main_window::ICP);

	//平滑
	QObject::connect(ui.actionGaussian, &QAction::triggered, this, &main_window::Gaussian);//平滑
	QObject::connect(ui.actionMLS, &QAction::triggered, this, &main_window::MLS);//平滑

	//修复
	QObject::connect(ui.actionRepair, &QAction::triggered, this, &main_window::Cloud_Repair);//修复



	/***************************************************************************************************************/

}


void main_window::cloud_type_change(PointCloudT::Ptr& cloud_Tag, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_SOR)//暂未用到
{

	pcl::copyPointCloud(*cloud_SOR, *cloud_Tag);
	if (cloud_Tag->points[0].r == 0 && cloud_Tag->points[0].g == 0 && cloud_Tag->points[0].b == 0)
	{
		setCloudRGB(cloud_Tag, 255, 255, 255);
	}
	setCloudAlpha(cloud_Tag, 255);
	cloud_show.push_back(cloud_Tag);
}
//核心代码
void main_window::getFileNames(string path, vector<string>& files)
{
	//文件句柄
	//注意：我发现有些文章代码此处是long类型，实测运行中会报错访问异常
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,递归查找
			//如果不是,把文件绝对路径存入vector中
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFileNames(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


void main_window::myLoad() //主要就是加了这个函数，实现加载点云的功能，按理说三个四个点云都能加载
{
	vector<string> fileNames;
	vector<string> open_points;
	string path("D:\\dianyun"); 	//自己选择目录测试
	getFileNames(path, fileNames);

	//加载点云文件
	// 清理 vector 和 viewer
	/*	cloud_vector.clear();*/
	int Y_offset = 32; // 和运动控制上位机的偏移数据一致
	MyPointCloud my_addPoint;
	my_addPoint.cloud.reset(new PointCloudT);
	myPointCloud.cloud.reset(new PointCloudT);
	cloud_show.clear();
	total_points = 0;
	int flag = 0;

	for (const auto& ph : fileNames) {
		//更新状态栏
		ui.statusBar->showMessage(" point cloud loading...");

		int status = -1;

		status = pcl::io::loadPLYFile(ph, *myPointCloud.cloud); //这句花的时间比较多，但是应该优化不了，因为这里就是库函数读取

		/*string filename = ph.substr(iPos, ph.length() - iPos);
		open_points.push_back(filename);*/

		if (status != 0)
		{
			QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
			return;
		}

		if (myPointCloud.cloud->points[0].r == 0 && myPointCloud.cloud->points[0].g == 0 && myPointCloud.cloud->points[0].b == 0)
		{
			setCloudRGB(myPointCloud.cloud, 255, 255, 255);
		}
		setCloudAlpha(myPointCloud.cloud, 255);

		// 最后导入的点云的信息
		if (flag == 0) //第一次进来赋值，第二次进来叠加
		{
			*my_addPoint.cloud = *myPointCloud.cloud;
		}
		else
		{
			for (int i = 0; i < myPointCloud.cloud->points.size(); ++i)
			{
				myPointCloud.cloud->points[i].y = myPointCloud.cloud->points[i].y + Y_offset * flag;//这句就是让第二个点云偏移后再叠加起来
			}
			//myPointCloud.cloud->points.y = myPointCloud.cloud->points.y + Y_offset;
			*my_addPoint.cloud = (*my_addPoint.cloud) + (*myPointCloud.cloud);
		}
		total_points += myPointCloud.cloud->points.size();
		flag++;
	}

	cloud_show.push_back(my_addPoint.cloud);  //将点云导入点云容器
	// 输出窗口

	setConsole("Load Cloud(s)", "We now have " + QString::number(total_points) + " Points.");
	ui.statusBar->showMessage("Load Point Cloud Done.");
	updatePointcloud();

}


void main_window::Initial() {



	// 初始化 point cloud
	myPointCloud.cloud.reset(new PointCloudT);
	myPointCloud.cloud->resize(1);

	// 初始化OpenCLNative Widget
	viewer.reset(new pcl::visualization::PCLVisualizer("point cloud viewer", false));


	vtkNew<vtkGenericOpenGLRenderWindow> window;//必须新建一个OpenGLwindow
	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindowInteractor> iren = ui.qvtkWidget->GetRenderWindow()->GetInteractor();
	//设置窗口
	iren->SetRenderWindow(ui.qvtkWidget->GetRenderWindow());
	//设置qvtk的渲染器
	ui.qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
	window->AddRenderer(viewer->getRendererCollection()->GetFirstRenderer());
	ui.qvtkWidget->SetRenderWindow(window.Get());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());//关联两个窗口

	this->Algorithm_windw = new Algorithm;//实例化第二个页面
	QObject::connect(this, &main_window::sent_pageUnm, this->Algorithm_windw, &Algorithm::Filter_page_set);
	// Console 区域设置为 NoSelection
	ui.consoleTable->setSelectionMode(QAbstractItemView::NoSelection);

}

void main_window::open()
{
	QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(model_dirname.c_str()), tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));
	if (filenames.isEmpty())
	{
		points_number = 0;
		QMessageBox::warning(this, "Warning", "请选择文件");
		return;
	}
	//cloud_show.clear();
	//total_points = 0;

	for (int i = 0; i != filenames.size(); i++) {
		// time start
		QString filename = filenames[i];
		std::string file_name = filename.toStdString();
		std::string subname = getFileName(file_name);  //提取全路径中的文件名（带后缀）

		//更新状态栏
		ui.statusBar->showMessage(QString::fromLocal8Bit(subname.c_str()) + ": " + QString::number(i) + "/" + QString::number(filenames.size()) + " point cloud loading...");

		int status = -1;
		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPCDFile(file_name, *myPointCloud.cloud);
		}
		else if (filename.endsWith(".ply", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPLYFile(file_name, *myPointCloud.cloud);
		}
		else if (filename.endsWith(".obj", Qt::CaseInsensitive))
		{
			status = pcl::io::loadOBJFile(file_name, *myPointCloud.cloud);
		}
		else
		{
			//提示：无法读取除了.ply .pcd .obj以外的文件
			QMessageBox::information(this, tr("File format error"),
				tr("Can't open files except .ply .pcd .obj"));
			return;
		}
		//提示：后缀没问题，但文件内容无法读取
		if (status != 0)
		{
			QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
			return;
		}

		if (myPointCloud.cloud->points[0].r == 0 && myPointCloud.cloud->points[0].g == 0 && myPointCloud.cloud->points[0].b == 0)
		{
			setCloudRGB(myPointCloud.cloud, 255, 255, 255);
		}
		setCloudAlpha(myPointCloud.cloud, 255);

		// 最后导入的点云的信息

		cloud_show.push_back(myPointCloud.cloud);  //将点云导入点云容器
		total_points += myPointCloud.cloud->points.size();
	}
	// 输出窗口

	setConsole("Load Cloud(s)", "We now have " + QString::number(total_points) + " Points.");
	ui.statusBar->showMessage("Load Point Cloud Done.");
	updatePointcloud();
}

void main_window::setCloudRGB(PointCloudT::Ptr& cloud, unsigned int r, unsigned int g, unsigned int b)
{
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
		cloud->points[i].a = 255;
	}
}

void main_window::setCloudAlpha(PointCloudT::Ptr& cloud, unsigned int a)
{
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].a = a;
	}
}

//保存操作日志
// Save operation log
void main_window::setConsole(QString operation, QString detail)
{

	int rows = ui.consoleTable->rowCount();
	ui.consoleTable->setRowCount(++rows);
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString time_str = time.toString("MM-dd hh:mm:ss"); //设置显示格式
	ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(time_str));
	ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
	ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(detail));
	ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}

std::string main_window::getFileName(std::string file_name)
{

	std::string subname;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subname.insert(subname.begin(), *i);
	}
	return subname;

}

void main_window::SaveFile() {

	QString fileName = QFileDialog::getSaveFileName(this, tr("Save point cloud"), "",
		tr("Point cloud data (*.pcd)"));
	if (fileName.isEmpty())
		return;
	int return_status;
	if (fileName.endsWith(".pcd", Qt::CaseInsensitive))
		return_status = pcl::io::savePCDFileASCII(fileName.toStdString(), *(myPointCloud.cloud));
	else {
		fileName.append(".pcd");
		return_status = pcl::io::savePCDFileASCII(fileName.toStdString(), *(myPointCloud.cloud));
	}

	if (return_status != 0) {
		setConsole("保存点云/网格", "失败");
		return;
	}
	else {
		setConsole("保存点云/网格", "成功，文件名为：" + fileName);
	}
}

void main_window::Clear() {
	// 更加彻底的移除
	viewer->removeAllPointClouds();
	viewer->removeAllShapes();
	cloud_show.clear();
	updateDataTree();
	ui.qvtkWidget->GetRenderWindow()->Render();
	total_points = 0;
	// operation log
	setConsole("清空点云，并重置记录", "");
}

void main_window::Exit()
{
	int a = 0;
	a = (QMessageBox::information(this, "warning", "sure to exit", QMessageBox::No, QMessageBox::Yes));
	if (a == 16384)
	{
		QApplication* app;
		app->exit(0);
	}
}

/***************更新点云***********************/
void main_window::updateDataTree()
{
	ui.dataTree->clear();
	//更新资源管理树
	for (int i = 0; i != cloud_show.size(); i++)
	{
		QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(std::to_string(i).c_str()));
		cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		ui.dataTree->addTopLevelItem(cloudName);
	}

}

void main_window::updatePointcloud()
{
	viewer->removeAllPointClouds();
	viewer->setBackgroundColor(0, 0, 0); //设置背景

	for (int i = 0; i != cloud_show.size(); i++)
	{
		viewer->addPointCloud(cloud_show[i], "cloud" + QString::number(i).toStdString());
		viewer->updatePointCloud(cloud_show[i], "cloud" + QString::number(i).toStdString());
	}

	// 显示结果图
	viewer->resetCamera();
	ui.qvtkWidget->GetRenderWindow()->Render();
	updateDataTree();
}

/***************更新dataTree***********************/

/********************Tool算法模块***********************************/
void main_window::Statistical_Filter()
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	/*Algorithm_windw->close();*/
	emit sent_pageUnm(1);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Statistical_Filter);

}

void main_window::Radius_Filter()//半径滤波
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(0);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Radius_Filter);
}
void main_window::Condition_Filter()//条件滤波
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(2);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Condition_Filter);

}

void main_window::Bilateral_Filter()
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(3);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Bilateral_Filter);

}


void main_window::Random_simplification()
{

	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(4);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_simplification, this, &main_window::set_Random_simplification);

}

void main_window::Isometry_simplification()
{
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(5);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_simplification, this, &main_window::set_Isometry_simplification);



}

void main_window::Voxel_Grid_simplification()
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(6);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_simplification, this, &main_window::set_Voxel_Grid_simplification);

}

void main_window::IA_RANSC()
{
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(10);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_registrarion, this, &main_window::set_IA_RANSC);


}

void main_window::NDT()
{
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(11);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_registrarion, this, &main_window::set_NDT);


}

void main_window::ICP()
{
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(12);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_registrarion, this, &main_window::set_ICP);

}
void main_window::Gaussian()
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(8);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_Smooth, this, &main_window::set_Gaussian);
}

void main_window::MLS()
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(7);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_Smooth, this, &main_window::set_MLS);


}
void main_window::Cloud_Repair()
{
	//Algorithm_windw->setWindowModality(Qt::ApplicationModal);//设置主界面不可选择
	Algorithm_windw->show();//program窗口显示
	emit sent_pageUnm(9);//发送数据
	////连接算法界面与主界面收到消息就执行
	QObject::connect(this->Algorithm_windw, &Algorithm::sent_cloud_Repair, this, &main_window::set_Cloud_Repair);


}
/*************************右键功能函数与dataTree************************************/

void main_window::itemPopmenu(const QPoint&)
{
	QTreeWidgetItem* curItem = ui.dataTree->currentItem(); //获取当前被点击的节点
	if (curItem == NULL)return;           //这种情况是右键的位置不在treeItem的范围内，即在空白位置右击
	QString name = curItem->text(0);
	int id = ui.dataTree->indexOfTopLevelItem(curItem);
	std::string cloud_id = "cloud" + QString::number(id).toStdString();


	QAction deleteItemAction("Clear", this);
	QAction hideItemAction("Hide", this);
	QAction showItemAction("Show", this);
	QAction changeColorAction("Change color", this);
	QAction randomColorAction("Random color", this);


	connect(&deleteItemAction, &QAction::triggered, this, &main_window::Clear);
	connect(&hideItemAction, &QAction::triggered, this, &main_window::pointHide);
	connect(&showItemAction, &QAction::triggered, this, &main_window::pointShow);
	connect(&changeColorAction, &QAction::triggered, this, &main_window::pointcolorChanged);
	connect(&randomColorAction, &QAction::triggered, this, &main_window::pointcolorRamdom);

	//QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&hideItemAction);
	menu.addAction(&showItemAction);
	menu.addAction(&deleteItemAction);
	menu.addAction(&changeColorAction);
	menu.addAction(&randomColorAction);

	//if (mycloud_vec[id].visible == true) {
	//	menu.actions()[1]->setVisible(false);
	//	menu.actions()[0]->setVisible(true);
	//}
	//else {
	//	menu.actions()[1]->setVisible(true);
	//	menu.actions()[0]->setVisible(false);
	//}

	menu.exec(QCursor::pos()); //在当前鼠标位置显示

}

void main_window::itemSelected(QTreeWidgetItem* item, int count)
{
	count = ui.dataTree->indexOfTopLevelItem(item);  //获取item的行号

	for (int i = 0; i != cloud_show.size(); i++)
	{
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + QString::number(i).toStdString());
	}

	*myPointCloud.cloud = *cloud_show[count];//将容器中我选中的点云传出来
	myPointCloud.cloud_showUnm = count;

	//选中item所对应的点云尺寸变大
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++) {
		int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			2, "cloud" + QString::number(cloud_id).toStdString());
	}
	setConsole("Selected", "We have Selected " + QString::number(myPointCloud.cloud->size()) + " Points.");
	ui.qvtkWidget->GetRenderWindow()->Render();

}

void main_window::pointcolorChanged()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "Select color for point cloud");

	if (color.isValid()) //判断所选的颜色是否有效
	{

		QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
		int selected_item_count = ui.dataTree->selectedItems().size();
		if (selected_item_count == 0) {
			for (int i = 0; i != cloud_show.size(); i++) {
				for (int j = 0; j != cloud_show[i]->points.size(); j++) {
					cloud_show[i]->points[j].r = color.red();
					cloud_show[i]->points[j].g = color.green();
					cloud_show[i]->points[j].b = color.blue();
				}
			}

		}
		else {
			for (int i = 0; i != selected_item_count; i++) {
				int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
				for (int j = 0; j != cloud_show[cloud_id]->size(); j++) {
					cloud_show[cloud_id]->points[j].r = color.red();
					cloud_show[cloud_id]->points[j].g = color.green();
					cloud_show[cloud_id]->points[j].b = color.blue();
				}
			}
			// 输出窗口
			setConsole("Change cloud color", "to " + QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()));
		}
	}
	updatePointcloud();
}

void main_window::pointcolorRamdom()
{
	unsigned int r = 255 * (rand() / (RAND_MAX + 1.0f));
	unsigned int g = 255 * (rand() / (RAND_MAX + 1.0f));
	unsigned int b = 255 * (rand() / (RAND_MAX + 1.0f));
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		for (int i = 0; i != cloud_show.size(); i++) {
			for (int j = 0; j != cloud_show[i]->points.size(); j++) {
				cloud_show[i]->points[j].r = r;
				cloud_show[i]->points[j].g = g;
				cloud_show[i]->points[j].b = b;
			}
		}

	}
	else {
		for (int i = 0; i != selected_item_count; i++) {
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			for (int j = 0; j != cloud_show[cloud_id]->size(); j++) {
				cloud_show[cloud_id]->points[j].r = r;
				cloud_show[cloud_id]->points[j].g = g;
				cloud_show[cloud_id]->points[j].b = b;
			}
		}
		// 输出窗口
		setConsole("Change cloud color", "to random color.");
	}
	updatePointcloud();
}

void main_window::pointHide()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		for (int i = 0; i != cloud_show.size(); i++) {
			setCloudAlpha(cloud_show[i], 0);
		}
		QString str = QString::number(selected_item_count);
		setConsole("Hide", str);

	}
	else if (selected_item_count != 0)
	{
		//for (int i = 0; i != selected_item_count; i++) {
		//	int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
		setCloudAlpha(cloud_show[myPointCloud.cloud_showUnm], 0);
		//}
		// 输出窗口
		QString str = QString::number(myPointCloud.cloud_showUnm);
		setConsole("Hide", str);
	}
	updatePointcloud();
}

void main_window::pointShow()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		for (int i = 0; i != cloud_show.size(); i++) {
			setCloudAlpha(cloud_show[i], 255);
		}
		// 输出窗口
		QString str = QString::number(selected_item_count);
		setConsole("Show", str);
	}
	else if (selected_item_count != 0)
	{
		//for (int i = 0; i != selected_item_count; i++)
		//{
		//	int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
		setCloudAlpha(cloud_show[myPointCloud.cloud_showUnm], 255);
		//}

		QString str = QString::number(myPointCloud.cloud_showUnm);
		setConsole("Show", str);
	}
	updatePointcloud();
}

/*************************滤波器************************************/
void main_window::set_Statistical_Filter(int a, double b, double c, int d)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Statistical_Filter);//端开链接
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_Statistical(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_SOR(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_SOR);
	if (cloud_SOR->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	Statistical_Filter_compute(cloud_SOR, cloud_output, a, b);

	pcl::copyPointCloud(*cloud_output, *cloud_Statistical);
	if (cloud_Statistical->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Statistical->points[0].r == 0 && cloud_Statistical->points[0].g == 0 && cloud_Statistical->points[0].b == 0)
		{
			setCloudRGB(cloud_Statistical, 255, 255, 255);
		}
		setCloudAlpha(cloud_Statistical, 255);
		cloud_show.push_back(cloud_Statistical);
		updatePointcloud();

		setConsole("Statistical Filter", "We now have " + QString::number(cloud_Statistical->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");

	}


}

void main_window::Statistical_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	int set_meank, double set_stddev_multhresh)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//创建统计滤波器对象
	sor.setInputCloud(input);//设置输入点云
	sor.setMeanK(set_meank);   //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(set_stddev_multhresh); //设置判断是否为离群点的阈值，如果一个点的距离超出平均距离一个标准差以上，则该点被标记为离群点，并将被移除。
	sor.filter(*output);//执行滤波，并将结果储存在cloud_filtered中 

}
void main_window::set_Radius_Filter(int a, double b, double c, int d)//半径滤波
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Radius_Filter);
	ui.statusBar->showMessage("Processing.");

	PointCloudT::Ptr cloud_Radius(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ROR(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_ROR);
	if (cloud_ROR->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	Radius_Filter_compute(cloud_ROR, cloud_output, d, b, a);
	pcl::copyPointCloud(*cloud_output, *cloud_Radius);

	if (cloud_Radius->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Radius->points[0].r == 0 && cloud_Radius->points[0].g == 0 && cloud_Radius->points[0].b == 0)
		{
			setCloudRGB(cloud_Radius, 255, 255, 255);
		}
		setCloudAlpha(cloud_Radius, 255);
		cloud_show.push_back(cloud_Radius);
		updatePointcloud();
		setConsole("Radius Filter", "We now have " + QString::number(cloud_Radius->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}
}

void main_window::Radius_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	int negative_select, double set_radius_search, int set_min_neighbors_in_radius)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;	//创建滤波器对象
	ror.setInputCloud(input);						//设置待滤波点云
	ror.setRadiusSearch(set_radius_search);						//设置查询点的半径范围
	ror.setMinNeighborsInRadius(set_min_neighbors_in_radius);					//设置判断是否为离群点的阈值，即半径内至少包括的点数
	ror.setNegative(negative_select);						//默认false，保存内点；true，保存滤掉的外点
	ror.filter(*output);					//执行滤波，保存滤波结果于cloud_filtered

}


void main_window::set_Condition_Filter(int a, double b, double c, int d)//条件滤波
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Condition_Filter);
	ui.statusBar->showMessage("Processing.");

	PointCloudT::Ptr cloud_Condition(new PointCloudT);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_COR(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_COR);
	if (cloud_COR->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	Condition_Filter_compute(cloud_COR, cloud_output, d);

	pcl::copyPointCloud(*cloud_output, *cloud_Condition);
	if (cloud_Condition->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Condition->points[0].r == 0 && cloud_Condition->points[0].g == 0 && cloud_Condition->points[0].b == 0)
		{
			setCloudRGB(cloud_Condition, 255, 255, 255);
		}
		setCloudAlpha(cloud_Condition, 255);
		cloud_show.push_back(cloud_Condition);
		updatePointcloud();
		setConsole("Condition Filter", "We now have " + QString::number(cloud_Condition->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}


}

void main_window::Condition_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	int set_keep_organized)
{
	//-------------------------- 条件滤波 --------------------------
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -400.0)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 400.0)));
	//创建滤波器
	pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(input);
	condrem.setKeepOrganized(set_keep_organized);	//对于散乱点云，不需要执行此语句；若输入点云为有组织的点云，此语句可保持点云的原始组织结构，不会改变行列数，点数也不会减少，被过滤掉的点用 NaN 填充。
	//执行条件滤波
	condrem.filter(*output);

}

void main_window::set_Bilateral_Filter(int a, double b, double c, int d)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_Str, this, &main_window::set_Bilateral_Filter);
	ui.statusBar->showMessage("Processing.");
	//结合点云调整三个参数
	const double dist_weigh = b;
	const double normal_weigh = c;
	const int k = a;

	PointCloudT::Ptr cloud_Bilateral(new PointCloudT);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BOR(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_BOR);
	if (cloud_BOR->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	Bilateral_Filter_compute(cloud_BOR, cloud_output, dist_weigh, k, normal_weigh);

	pcl::copyPointCloud(*cloud_output, *cloud_Bilateral);
	if (cloud_Bilateral->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Bilateral->points[0].r == 0 && cloud_Bilateral->points[0].g == 0 && cloud_Bilateral->points[0].b == 0)
		{
			setCloudRGB(cloud_Bilateral, 255, 255, 255);
		}
		setCloudAlpha(cloud_Bilateral, 255);
		cloud_show.push_back(cloud_Bilateral);
		updatePointcloud();
		setConsole("Bilateral Filter", "We now have " + QString::number(cloud_Bilateral->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}
}

void main_window::Bilateral_Filter_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	double dist_weigh, int K, double normal_weigh)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bf(new pcl::PointCloud<pcl::PointXYZ>);
	//kdtree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treebf(new pcl::search::KdTree<pcl::PointXYZ>);
	treebf->setInputCloud(input);
	//计算法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal> cloud_normals;
	ne.setInputCloud(input);
	ne.setSearchMethod(treebf);
	ne.setKSearch(K);
	ne.compute(cloud_normals);

	for (size_t i = 0; i < input->size(); ++i)
	{
		std::vector<int> indices;
		std::vector<float>dist_square;
		indices.reserve(K);
		indices.reserve(K);

		double zeta = 0.0;
		double sum = 0.0;
		if (treebf->nearestKSearch(input->points[i], K, indices, dist_square) > 0)
		{
			for (size_t j = 1; j < indices.size(); ++j)
			{
				double diffx = input->points[indices[j]].x - input->points[i].x;
				double diffy = input->points[indices[j]].y - input->points[i].y;
				double diffz = input->points[indices[j]].z - input->points[i].z;
				double dn = cloud_normals.at(i).normal_x * diffx +
					cloud_normals.at(i).normal_y * diffy +
					cloud_normals.at(i).normal_z * diffz;
				double w = exp(-1 * dn * dn / (2 * normal_weigh * normal_weigh)) * exp(-1 * dist_square[j] / (2 * dist_weigh * dist_weigh));
				zeta += w * dn;
				sum += w;
			}

		}
		if (sum < 1e-10)
		{
			zeta = 0.0;
		}
		else
		{
			zeta /= sum;
		}
		pcl::PointXYZ smoothed_point;
		smoothed_point.x = input->points[i].x + cloud_normals.at(i).normal_x * zeta;
		smoothed_point.y = input->points[i].y + cloud_normals.at(i).normal_y * zeta;
		smoothed_point.z = input->points[i].z + cloud_normals.at(i).normal_z * zeta;
		cloud_bf->push_back(smoothed_point);
	}
	cloud_bf->width = cloud_bf->size();
	cloud_bf->height = 1;
	cloud_bf->resize(cloud_bf->width * cloud_bf->height);
	*output = *cloud_bf;


}

/*************************精简************************************/
void main_window::set_Random_simplification(int a, double b, double c, double d, int e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_simplification, this, &main_window::set_Random_simplification);
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_Random(new PointCloudT);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rs(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_rs);
	if (cloud_rs->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}
	Random_compute(cloud_rs, cloud_output, b, e);

	pcl::copyPointCloud(*cloud_output, *cloud_Random);
	if (cloud_Random->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Random->points[0].r == 0 && cloud_Random->points[0].g == 0 && cloud_Random->points[0].b == 0)
		{
			setCloudRGB(cloud_Random, 255, 255, 255);
		}
		setCloudAlpha(cloud_Random, 255);
		cloud_show.push_back(cloud_Random);
		updatePointcloud();
		setConsole("Random simplification", "We now have " + QString::number(cloud_Random->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}
}

void main_window::Random_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	double set_sample, int set_seed)
{
	pcl::RandomSample<pcl::PointXYZ> rs;	//创建滤波器对象
	rs.setInputCloud(input);				//设置待滤波点云
	int rsN = set_sample * input->points.size();
	rs.setSample(rsN);					//设置下采样后点云的点数
	rs.setSeed(set_seed);						//设置随机函数种子点
	rs.filter(*output);					//执行下采样滤波，保存滤波结果
}

void main_window::set_Isometry_simplification(int a, double b, double c, double d, int e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_simplification, this, &main_window::set_Isometry_simplification);
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_Isometry(new PointCloudT);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_us(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_us);

	if (cloud_us->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}
	Isometry_compute(cloud_us, cloud_output, b);

	pcl::copyPointCloud(*cloud_output, *cloud_Isometry);
	if (cloud_Isometry->size() < 1)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Isometry->points[0].r == 0 && cloud_Isometry->points[0].g == 0 && cloud_Isometry->points[0].b == 0)
		{
			setCloudRGB(cloud_Isometry, 255, 255, 255);
		}
		setCloudAlpha(cloud_Isometry, 255);
		cloud_show.push_back(cloud_Isometry);
		updatePointcloud();
		setConsole("Isometry simplification", "We now have " + QString::number(cloud_Isometry->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}

}


void main_window::Isometry_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	double set_radius_search)
{
	pcl::UniformSampling<pcl::PointXYZ> us;		//创建滤波器对象
	us.setInputCloud(input);				//设置待滤波点云
	us.setRadiusSearch(set_radius_search);				//设置滤波球半径
	us.filter(*output);				//执行滤波，保存滤波结果
}

void main_window::set_Voxel_Grid_simplification(int a, double b, double c, double d, int e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_simplification, this, &main_window::set_Voxel_Grid_simplification);
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_Voxel_Grid(new PointCloudT);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_vg);

	if (cloud_vg->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	Voxel_Grid_compute(cloud_vg, cloud_output, b, c, d, e);

	pcl::copyPointCloud(*cloud_output, *cloud_Voxel_Grid);
	int zza = cloud_output->points.size();

	if (cloud_Voxel_Grid->size() < 1)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Voxel_Grid->points[0].r == 0 && cloud_Voxel_Grid->points[0].g == 0 && cloud_Voxel_Grid->points[0].b == 0)
		{
			setCloudRGB(cloud_Voxel_Grid, 255, 255, 255);
		}

		setCloudAlpha(cloud_Voxel_Grid, 255);
		cloud_show.push_back(cloud_Voxel_Grid);
		updatePointcloud();
		setConsole("Voxel Grid simplification", "We now have " + QString::number(cloud_Voxel_Grid->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}


}

void  main_window::Voxel_Grid_compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
	double set_leaf_size_x, double set_leaf_size_y,
	double set_leaf_size_z, int set_perVoxel)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;//滤波处理对象
	vg.setInputCloud(input);
	vg.setDownsampleAllData(FALSE);
	vg.setLeafSize(set_leaf_size_x, set_leaf_size_y, set_leaf_size_z);//设置滤波器处理时采用的体素大小的参数
	vg.setMinimumPointsNumberPerVoxel(set_perVoxel);//设置每个体素中的点的个数
	vg.filter(*output);


}

/*************************平滑************************************/
void main_window::set_Gaussian(int a, int b, int c, double d, double e)
{

	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_Smooth, this, &main_window::set_Gaussian);
	ui.statusBar->showMessage("Processing.");

	PointCloudT::Ptr cloud_Gaussian(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gs(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_gs);
	if (cloud_gs->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}
	Gaussian_compute(cloud_gs, cloud_output, a, b, d, c, e);

	pcl::copyPointCloud(*cloud_output, *cloud_Gaussian);
	if (cloud_Gaussian->size() < 1)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{
		if (cloud_Gaussian->points[0].r == 0 && cloud_Gaussian->points[0].g == 0 && cloud_Gaussian->points[0].b == 0)
		{
			setCloudRGB(cloud_Gaussian, 255, 255, 255);
		}
		setCloudAlpha(cloud_Gaussian, 255);
		cloud_show.push_back(cloud_Gaussian);
		updatePointcloud();
		setConsole("Gaussian smoothing", "We now have " + QString::number(cloud_Gaussian->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}


}

void main_window::Gaussian_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	int set_sigma, int set_relative_sigma,
	double set_threshold, int set_number_of_threads,
	double search_radius)
{

	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(set_sigma);//高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(set_relative_sigma);//设置相对Sigma参数的距离阈值
	kernel.setThreshold(set_threshold);//设置距离阈值，若点间距离大于阈值则不予考虑
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input);

	//设置Convolution 相关参数
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);//设置卷积核
	convolution.setInputCloud(input);
	convolution.setNumberOfThreads(set_number_of_threads);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(search_radius);// 设置近邻半径，单位是 mm，越大平滑效果越好
	convolution.convolve(*output);

}

void main_window::set_MLS(int a, int b, int c, double d, double e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_Smooth, this, &main_window::set_MLS);
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_MLS(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ms(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_ms);
	if (cloud_ms->size() < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	mls_compute(cloud_ms, cloud_output, a, c, d);

	pcl::copyPointCloud(*cloud_output, *cloud_MLS);

	if (cloud_MLS->size() < 1)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{

		if (cloud_MLS->points[0].r == 0 && cloud_MLS->points[0].g == 0 && cloud_MLS->points[0].b == 0)
		{
			setCloudRGB(cloud_MLS, 255, 255, 255);
		}
		setCloudAlpha(cloud_MLS, 255);
		cloud_show.push_back(cloud_MLS);
		updatePointcloud();
		setConsole("MLS smoothing", "We now have " + QString::number(cloud_MLS->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}

}

void main_window::mls_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	int setComputeNormals,
	int polynomial_order,
	double search_radius)
{

	pcl::PointCloud<pcl::PointXYZ> colud_mls;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input);
	mls.setComputeNormals(setComputeNormals);//设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(input);
	mls.setPolynomialOrder(polynomial_order); //MLS拟合的阶数，默认是2
	mls.setSearchMethod(tree);
	mls.setSearchRadius(search_radius);  //单位m.设置用于拟合的K近邻半径.这个值越大，输出的点越多,1几乎保留所有点
	mls.process(colud_mls);//格式如果不统一的话，还需要进一步转换10112022

	*output = colud_mls;
}

/*************************点云修复************************************/

void  main_window::set_Cloud_Repair()
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_cloud_Repair, this, &main_window::set_Cloud_Repair);
	ui.statusBar->showMessage("Processing.");
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudT::Ptr Cloud_Repair_target(new PointCloudT);
	// 填入点云数据
	pcl::copyPointCloud(*myPointCloud.cloud, *inputcloud);
	int m_PointSumNumber1 = inputcloud->size();
	if (m_PointSumNumber1 < 1)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}
	PointCloudData* m_myPointCloudData = new PointCloudData();
	PCPoint* tempPointData;//点云数据临时存储器
	tempPointData = (PCPoint*) new PCPoint[m_PointSumNumber1];
	for (int num = 0; num < m_PointSumNumber1; num++)
	{
		tempPointData[num].m_Coordinate[0] = inputcloud->points[num].x;
		tempPointData[num].m_Coordinate[1] = inputcloud->points[num].y;
		tempPointData[num].m_Coordinate[2] = inputcloud->points[num].z;
	}

	m_myPointCloudData->m_PointSumNumber = m_PointSumNumber1;
	m_myPointCloudData->InitPointCloud(tempPointData);//初始化点云数据
	delete[] tempPointData;//释放内存

	////---------------------------------------------------------------------------------------PCL中的模型边界保留算法	
	m_myPointCloudData->RThreadshold = 0.1 * 6;//边界半径搜索参数。不宜太小，否则容易边界点冗余
	m_myPointCloudData->BAngleThreshold = M_PI / 6;//边界判断时的角度阈值
	m_myPointCloudData->PCLBoundaryKeep();

	////---------------------------------------------------------------------------------------边界聚类	
	m_myPointCloudData->m_clusterDistance = 0.1 * 5;//最小聚类距离。zcl
	m_myPointCloudData->m_clusterNum = 2;;//最小聚类个数
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundarycloud(new pcl::PointCloud<pcl::PointXYZ>);
	m_myPointCloudData->BoundaryClustering(boundarycloud);
	////pcl::io::savePCDFileBinary("boundarycloud.pcd", *boundarycloud);

	////m_myPointCloudData->m_BoundaryPointClusters[0];//存储边界聚类集合0

	////-------------------------------------------------------------------------------------------自动修补
	m_myPointCloudData->AutomaticHoleRepair();
	//保存修补点
	int  repair_num = m_myPointCloudData->m_AddedPoingCLoud.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr repaircloud(new pcl::PointCloud<pcl::PointXYZ>);
	repaircloud->width = repair_num;//初始化 PCL 点云
	repaircloud->height = 1;
	repaircloud->points.resize(repaircloud->width * repaircloud->height);

	for (int num = 0; num < repair_num; num++)
	{
		repaircloud->points[num].x = m_myPointCloudData->m_AddedPoingCLoud[num].m_Coordinate[0];
		repaircloud->points[num].y = m_myPointCloudData->m_AddedPoingCLoud[num].m_Coordinate[1];
		repaircloud->points[num].z = m_myPointCloudData->m_AddedPoingCLoud[num].m_Coordinate[2];
	}
	*cloud_temp = (*repaircloud) + (*inputcloud);
	pcl::copyPointCloud(*cloud_temp, *Cloud_Repair_target);
	if (Cloud_Repair_target->size() < 1)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Clouds repair failed"));
		return;
	}
	else
	{

		if (Cloud_Repair_target->points[0].r == 0 && Cloud_Repair_target->points[0].g == 0 && Cloud_Repair_target->points[0].b == 0)
		{
			setCloudRGB(Cloud_Repair_target, 255, 255, 255);
		}
		setCloudAlpha(Cloud_Repair_target, 255);
		cloud_show.push_back(Cloud_Repair_target);
		updatePointcloud();
		setConsole("Cloud Repair", "We now have " + QString::number(Cloud_Repair_target->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}
}

/*************************点云配准************************************/
void main_window::set_IA_RANSC(double a, double b, double c, double d, int e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_registrarion, this, &main_window::set_IA_RANSC);
	ui.statusBar->showMessage("Processing.");

	PointCloudT::Ptr cloud_RANSC(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);//处理之后的点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);//处理之后的点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_0(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_target_0);//在上位机选择目标文件
	//去除NAN点
	std::vector<int> indices_tgt; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_target_0, *cloud_target, indices_tgt);
	myPointCloud.cloud->resize(1);
	open();
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_source_0);//在上位机选择目标文件
	std::vector<int> indices_src; //保存去除的点的索引
	pcl::removeNaNFromPointCloud(*cloud_source_0, *cloud_source, indices_src);

	if (cloud_target->size() < 2 || cloud_source->size() < 2)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}

	IA_RANSC_compute(cloud_source, cloud_target, cloud_output, a, b, c, d);

	pcl::copyPointCloud(*cloud_output, *cloud_RANSC);
	if (cloud_RANSC->size() < 2)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{

		if (cloud_RANSC->points[0].r == 0 && cloud_RANSC->points[0].g == 0 && cloud_RANSC->points[0].b == 0)
		{
			setCloudRGB(cloud_RANSC, 255, 255, 255);
		}
		setCloudAlpha(cloud_RANSC, 255);
		cloud_show.push_back(cloud_RANSC);
		updatePointcloud();
		setConsole("IA RANSC Registration", "We now have " + QString::number(cloud_RANSC->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}

}

void main_window::IA_RANSC_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	double src_setRadius,
	double tgt_setRadius,
	double fpfh_src_setRadius,
	double fpfh_tgt_setRadius)
{
	//---------------------------------- ia_ransac配准 --------------------------
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//定义一个KdTree检索方式
	//计算表面法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_src;
	ne_src.setInputCloud(source);
	ne_src.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	ne_src.setRadiusSearch(src_setRadius);//zcl 表示邻域集的半径，决定计算的表面法线是否垂直表面，网上也有取0.3 0.5
	ne_src.compute(*cloud_src_normals);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_tgt;
	ne_tgt.setInputCloud(target);

	ne_tgt.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	//ne_tgt.setKSearch(20);
	ne_tgt.setRadiusSearch(tgt_setRadius);
	ne_tgt.compute(*cloud_tgt_normals);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
	fpfh_src.setInputCloud(source);
	fpfh_src.setInputNormals(cloud_src_normals);

	fpfh_src.setSearchMethod(tree);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_src.setRadiusSearch(fpfh_src_setRadius);
	fpfh_src.compute(*fpfhs_src);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_tgt;
	fpfh_tgt.setInputCloud(target);
	fpfh_tgt.setInputNormals(cloud_tgt_normals);
	//kdTree加速

	fpfh_tgt.setSearchMethod(tree);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_tgt.setRadiusSearch(fpfh_tgt_setRadius);
	fpfh_tgt.compute(*fpfhs_tgt);

	//SAC配准
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
	scia.setInputSource(source);
	scia.setInputTarget(target);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(1);
	//scia.setNumberOfSamples(2);
	//scia.setCorrespondenceRandomness(20);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sac(new pcl::PointCloud<pcl::PointXYZ>);
	scia.align(*cloud_sac);
	Eigen::Matrix4f sac_trans;
	sac_trans = scia.getFinalTransformation();

	*output = *cloud_sac;
	*output += *target;
}

void main_window::set_NDT(double a, double b, double c, double d, int e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_registrarion, this, &main_window::set_NDT);
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_NDT(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_target);
	myPointCloud.cloud->resize(1);
	open();
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_source);
	if (cloud_target->size() < 2 || cloud_source->size() < 2)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}
	NDT_compute(cloud_source, cloud_target, cloud_output, a, b, c, e);
	pcl::copyPointCloud(*cloud_output, *cloud_NDT);
	if (cloud_NDT->size() < 2)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{

		if (cloud_NDT->points[0].r == 0 && cloud_NDT->points[0].g == 0 && cloud_NDT->points[0].b == 0)
		{
			setCloudRGB(cloud_NDT, 255, 255, 255);
		}
		setCloudAlpha(cloud_NDT, 255);
		cloud_show.push_back(cloud_NDT);
		updatePointcloud();
		setConsole("NDT Registration", "We now have " + QString::number(cloud_NDT->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}

}
void main_window::NDT_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
	double set_epsilon,
	double set_step_size,
	double set_resolution,
	int set_maximum_iterations)
{

	//初始化正态分布变换（NDT）
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//设置依赖尺度NDT参数

	//为终止条件设置最小转换差异
	ndt.setTransformationEpsilon(set_epsilon);
	//为More-Thuente线搜索设置最大步长
	ndt.setStepSize(set_step_size);
	//设置NDT网格结构的分辨率（VoxelGridCovariance）
	ndt.setResolution(set_resolution);
	//设置匹配迭代的最大次数
	ndt.setMaximumIterations(set_maximum_iterations);
	// 设置要配准的点云
	ndt.setInputSource(source);
	//设置点云配准目标
	ndt.setInputTarget(target);
	//计算需要的刚体变换以便将输入的点云匹配到目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*cloud_ndt);//配准，将配准后的点云放至output_cloud
	Eigen::Matrix4f transformation_ndt = ndt.getFinalTransformation();
	*output = *cloud_ndt;
	*output += *target;

}


void main_window::set_ICP(double a, double b, double c, double d, int e)
{
	QObject::disconnect(this->Algorithm_windw, &Algorithm::sent_registrarion, this, &main_window::set_ICP);
	ui.statusBar->showMessage("Processing.");
	PointCloudT::Ptr cloud_ICP(new PointCloudT);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZRGB>);


	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_target);
	myPointCloud.cloud->resize(1);
	open();
	pcl::copyPointCloud(*myPointCloud.cloud, *cloud_source);
	if (cloud_target->size() < 2 || cloud_source->size() < 2)//点云大小小于10即错误
	{
		QMessageBox::critical(this, tr("error"), tr("NO Point Clouds"));
		return;
	}
	ICP_compute(cloud_source, cloud_target, output_cloud, a, b, c, e, 0.0, 0.0, 0.0);//ICP计算
	pcl::copyPointCloud(*output_cloud, *cloud_ICP);

	if (cloud_ICP->size() < 2)//点云大小小于1即错误
	{
		QMessageBox::critical(this, tr("error"), tr("Parameter setting error"));
		return;
	}
	else
	{

		if (cloud_ICP->points[0].r == 0 && cloud_ICP->points[0].g == 0 && cloud_ICP->points[0].b == 0)
		{
			setCloudRGB(cloud_ICP, 255, 255, 255);
		}
		setCloudAlpha(cloud_ICP, 255);
		cloud_show.push_back(cloud_ICP);
		updatePointcloud();
		setConsole("ICP Registration", "We now have " + QString::number(cloud_ICP->size()) + " Points.");
		ui.statusBar->showMessage("Point Cloud Processing Done.");
	}



}

//source:源点云
//target：目标点云
//output : 输出点云
//set_distance：设置对应点对之间的最大距离（影响因素较大）（0.5）
//set_formation_epsilon：设置两次变换矩阵之间的差值（一般设置为1e - 10）
//set_fitness_epsilon：设置收敛条件是均方误差和小于阈值，停止迭代（0.01）
//set_maximum_iterations：设置最大迭代次数，ICP算法最多迭代的次数。（20）
//tx : x轴预设平移距离(源点云坐标系与目标点云坐标系的x轴偏移值)（可设置为0）
//ty : y轴预设平移距离(源点云坐标系与目标点云坐标系的y轴偏移值)（可设置为0）
//tz : z轴预设平移距离(源点云坐标系与目标点云坐标系的z轴偏移值)（可设置为0）
void main_window::ICP_compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
	double set_distance,
	double set_formation_epsilon,
	double set_fitness_epsilon,
	int set_maximum_iterations,
	double tx, double ty, double tz)
{

	Eigen::Affine3f transform_vsule = Eigen::Affine3f::Identity();

	// 1，定义平移
	transform_vsule.translation() << tx, ty, tz;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_output_source(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::transformPointCloud(*source, *transformed_source_cloud, transform_vsule);

	//滤波
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_v(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_v(new pcl::PointCloud<pcl::PointXYZRGB>);

	Voxel_Grid_compute(transformed_source_cloud, cloud_source_v, 0.1, 0.1, 0.1, 5);
	Voxel_Grid_compute(target, cloud_target_v, 0.1, 0.1, 0.1, 5);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;//创建ICP对象	

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSrc(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeTarget(new pcl::search::KdTree<pcl::PointXYZRGB>);

	treeSrc->setInputCloud(cloud_source_v);
	treeTarget->setInputCloud(cloud_target_v);
	icp.setSearchMethodSource(treeSrc);
	icp.setSearchMethodTarget(treeTarget);

	icp.setInputSource(cloud_source_v);//为ICP算法设置输入点云	
	icp.setInputTarget(cloud_target_v);//设置目标点云	
	pcl::PointCloud<pcl::PointXYZRGB> cloud_src_icp;//cloud_src_icp为验证的结果

	icp.setMaxCorrespondenceDistance(set_distance);//当两个点云相距较远时候，距离值要变大，所以一开始需要粗配准。
	icp.setTransformationEpsilon(set_formation_epsilon);//svd奇异值分解，对icp时间影响不大
	icp.setEuclideanFitnessEpsilon(set_fitness_epsilon);//前后两次误差大小，当误差值小于这个值停止迭代
	icp.setMaximumIterations(set_maximum_iterations);//最大迭代次数
	icp.align(cloud_src_icp);//转换后的cloud_src_icp


	Eigen::Matrix4d T(4, 4);
	T = icp.getFinalTransformation().cast<double>();//getFinalTransformation()输出的是float 转成double
	Eigen::Matrix3d R = T.block<3, 3>(0, 0);// = Eigen::Matrix3f::Identity();
	Eigen::Quaterniond q = Eigen::Quaterniond(R);

	Eigen::MatrixXd t_mat = T.topRightCorner(3, 1).transpose();
	Eigen::Vector3d t;
	t << t_mat(0), t_mat(1), t_mat(2);
	Eigen::Matrix4f transformation_icp = icp.getFinalTransformation();
	std::cout << "icp 结果" << transformation_icp << std::endl;
	pcl::transformPointCloud(*source, *transformed_output_source, transformation_icp);



	*output = *transformed_output_source;
	*output += *target;


}


/*************************结束************************************/
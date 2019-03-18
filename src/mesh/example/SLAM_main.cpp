#include "SLAM_main.h"



vector<string> split3(const string &str, const char pattern)
{
	vector<string> res;
	stringstream input(str);   //读取str到字符串流中
	string temp;
	//使用getline函数从字符串流中读取,遇到分隔符时停止,和从cin中读取类似
	//注意,getline默认是可以读取空格的
	while (getline(input, temp, pattern))
	{
		res.push_back(temp);
	}
	return res;
}

int slam_main(bool singleCamera)
{
	int globalID = 0;

	ifstream in("frame_point.txt");
	string filename;
	string line;

	cameras.resize(100);
	for (size_t i = 0; i<100; i++)
	{
		if (singleCamera)
		{
			string name = "./output_pcd/frame" + to_string(i) + ".pcd";

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			//打开点云文件
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(name, *cloud) == -1)
			{
				PCL_ERROR("Couldn't read file test_pcd.pcd \n");
				return (-1);
			}
			//eigen2cv(rotation_matrix, ext);
			//cameraMat.push_back(ext);

			//--------------转换相机信息----------------------
			//NewCameraType tempCamera;
			CameraType tempCamera;

			vector<glm::vec3> points;
			glm::vec3 center3D;



			if (in) // 有该文件
			{
				getline(in, line); // line中不包括每行的换行符
				vector<string> res = split3(line, ' ');
				center3D.x = stof(res[0]);
				center3D.y = stof(res[1]);
				center3D.z = stof(res[2]);;
			}

			for (size_t k = 0; k < cloud->points.size(); ++k)
			{
				glm::vec3 temp_P;
				temp_P.x = cloud->points[k].x;
				temp_P.y = cloud->points[k].y;
				temp_P.z = cloud->points[k].z;
				points.push_back(temp_P);
			}


			tempCamera.idCam = i;
			tempCamera.center = center3D;
			tempCamera.imageWidth = 1241;
			tempCamera.imageHeight = 376;
			//cameras.push_back(tempCamera); // vector插入若原先内存不够，会分配新的内存，将原来的复制过去，所以涉及指针操作可能会出错
			cameras[i] = tempCamera;

			//set<NewPointType> tempFramePoints;
			for (int k = 0; k < points.size(); k++)
			{
				//NewPointType* singlePoint = new NewPointType();
				PointType* singlePoint = new PointType();

				singlePoint->position = points[k];
				singlePoint->idPoint = globalID;
				singlePoint->addCamera(&cameras[i]);

				(cameras[i].visiblePointsT).insert(singlePoint);

				globalID++;
			}
		}

	}

	return 0;
}

/*
int showCloud(string path)
{
	PointCloud::Ptr cloud(new PointCloud());
	if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
	std::cout << "Loaded "
		<< cloud->width*cloud->height
		<< " data points from test_pcd.pcd with the following fields: "
		<< std::endl;
   
	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
	system("pause");
	return 0;
}
*/
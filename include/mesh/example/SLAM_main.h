#ifndef HEADER_SLAM_MAIN_H

#define HEADER_SLAM_MAIN_H

#include <string>
#include <set>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/nonfree/nonfree.hpp> // use this if you want to use SIFT or SURF
#include <opencv2/calib3d/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// Eigen !
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

#include <set>
#include <map>
#include <string>

#include <glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "types_reconstructor.hpp"

//extern vector<NewCameraType> cameras;
extern vector<CameraType> cameras;
extern vector<vector<int> > meshPointsVisibleFromCamN; //每一帧看到的点ID
extern vector<vector<int>> meshCamViewingPointN; //每个点看到的帧集合

extern vector<cv::Point3f> center;//相机中心坐标
extern vector<vector<cv::Point3f>> framePoints; //关键点的点云坐标
extern vector<vector<cv::Point2f>> frameImgPoints;//关键点的像素坐标

extern vector<map<int, int>> matchPoints;
extern vector<vector<cv::Point2f>> imgPoints;

extern vector<vector<glm::vec2>> meshPoint2DoncamViewingPoint;//每个ID点在出现帧的像素集合

int showCloud(string path);

int slam_main(bool singleCamera);


#endif
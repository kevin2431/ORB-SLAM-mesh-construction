//  Copyright 2014 Andrea Romanoni
//
//  This file is part of manifoldReconstructor.
//
//  edgePointSpaceCarver is free software: you can redistribute it
//  and/or modify it under the terms of the GNU General Public License as
//  published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version see
//  <http://www.gnu.org/licenses/>..
//
//  edgePointSpaceCarver is distributed in the hope that it will be
//  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <string>

#include "CameraPointsCollection.h"
#include "Chronometer.h"
#include "Logger.h"
#include "ReconstructFromSLAMData.h"
#include "OpenMvgParser.h"
#include "ConfigParser.h"
#include "types_config.hpp"
#include "types_reconstructor.hpp"
#include "points_filtering.hpp"
#include "SLAM_main.h"
#include "SLAM_type.h"


#include<opencv2/imgproc/imgproc.hpp>

//#include <g2o/core/sparse_optimizer.h>

//g2o::SparseOptimizer globalOptimizer;

void printUsage(char *name);
void readArgs(int argc, char **argv);
void trans_data(SfMData &sfm_data_, CameraPointsCollection &incData, std::vector<bool> &inliers, ManifoldReconstructionConfig &confManif);

int maxIterations_ = 0;
std::string input_file;
std::string config_file;

//vector<NewCameraType> cameras;
vector<CameraType> cameras;
vector<vector<int> > meshPointsVisibleFromCamN; //每一帧看到的点ID
vector<vector<int>> meshCamViewingPointN; //每个点看到的帧集合
map<int, map<int, cv::Point2f>> pointIDCoord;//每一帧的对应像素点集合

//暂时不需要
vector<cv::Point3f> center;//相机中心坐标
vector<vector<cv::Point3f>> framePoints; //关键点的点云坐标
vector<vector<cv::Point2f>> frameImgPoints;//关键点的像素坐标

vector<map<int, int>> matchPoints;
vector<vector<cv::Point2f>> imgPoints;

vector<vector<glm::vec2>> meshPoint2DoncamViewingPoint;//每个ID点在出现帧的像素集合

string depth_path;
string png_path;

int main(int argc, char **argv)
{

	slam_main(true);
	config_file = "./res/config/default.json";

    //std::cout << "input set to: " << input_file << std::endl;
    std::cout << "config set to: " << config_file << std::endl;
    //std::cout << "max_iterations set to: " << maxIterations_ << std::endl;


    std::ofstream statsFile;
    std::ofstream visiblePointsFile;

    ManifoldReconstructionConfig confManif;
    ConfigParser configParser = ConfigParser();
    confManif = configParser.parse(config_file);

    //SfMData sfm_data_;

    //CameraPointsCollection incData;
    //OpenMvgParser op_openmvg(argv[1]);
    //op_openmvg.parse();

    ReconstructFromSLAMData m(confManif);

    //m.setExpectedTotalIterationsNumber((maxIterations_) ? maxIterations_ + 1 : op_openmvg.getSfmData().numCameras_);

    //sfm_data_ = op_openmvg.getSfmData();


    std::vector<bool> inliers;
	//outlierFiltering(inliers, confManif.outlierFilteringThreshold, sfm_data_);

	//trans_data(sfm_data_, incData, inliers, confManif);
	std::cout << "end" << std::endl;

    // Main loop
	confManif.triangulationUpdateEvery = 1;
	confManif.initialTriangulationUpdateSkip = 0;
    //for (auto index_camera : incData.getCameras()) 
	for (int i = 0; i < cameras.size(); i++)
	{
        // If maxIterations_ is set, only execute ReconstructFromSLAMData::addCamera maxIterations_ times
        if (maxIterations_ && m.iterationCount >= maxIterations_) 
		{
            break;
        }

        //m.addCamera(index_camera.second);
		m.addCamera(&cameras[i]);

        // Skip the manifold update for the first confManif.initial_manifold_update_skip cameras
        if (m.iterationCount > confManif.initialTriangulationUpdateSkip && !(m.iterationCount % 1))// confManif.triangulationUpdateEvery)) 
		{
            m.update();
			m.integrityCheck();
        }

        if (m.iterationCount && !(m.iterationCount % 1))//confManif.saveMeshEvery)) 
		{
            int manifold_seq = m.iterationCount / confManif.saveMeshEvery;
            m.saveMesh("output/", "current_" + std::to_string(manifold_seq));
        }
    }

    // Do a last manifold update in case op.numCameras() isn't a multiple of confManif.manifold_update_every
    if (m.iterationCount > confManif.initialTriangulationUpdateSkip) 
	{
        m.update();
    }

    m.saveMesh("output/", "final");

	globalOptimizer.clear();

    return 0;
}

void trans_data(SfMData &sfm_data_, CameraPointsCollection &incData, std::vector<bool> &inliers, ManifoldReconstructionConfig &confManif)
{
	for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) 
	{
		CameraType* camera = &sfm_data_.camerasList_[cameraIndex];
		camera->idCam = cameraIndex;
		incData.addCamera(camera);
	}

	for (int pointIndex = 0; pointIndex < sfm_data_.points_.size(); pointIndex++) 
	{
		if (inliers[pointIndex]) 
		{
			PointType* point = new PointType();
			point->idPoint = pointIndex;
			point->position = sfm_data_.points_[pointIndex];

			incData.addPoint(point);
		}
	}

	for (int cameraIndex = 0; cameraIndex < sfm_data_.camerasList_.size(); cameraIndex++) 
	{
		int inliersCount = 0, inlierPointIndex = 0;
		for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex])
			if (inliers[pointIndex]) 
				inliersCount++;

		for (auto pointIndex : sfm_data_.pointsVisibleFromCamN_[cameraIndex]) 
		{
			if (confManif.maxPointsPerCamera < inliersCount) 
			{
				if (inliers[pointIndex]) 
				{
					if (inlierPointIndex < confManif.maxPointsPerCamera) 
					{
						incData.addVisibility(cameraIndex, pointIndex);
					}

					inlierPointIndex++;
				}

			}
			else 
			{
				if (inliers[pointIndex]) 
					incData.addVisibility(cameraIndex, pointIndex);
			}
		}
	}
}

void printUsage(char *name)
{
    std::cout << name << " sfm_data.json [config_file.json [max_iterations]]" << std::endl;
}


void readArgs(int argc, char **argv)
{
    if (argc == 4) {
        maxIterations_ = atoi(argv[3]);
        input_file = argv[1];
        config_file = argv[2];
    } else if (argc == 3) {
        input_file = argv[1];
        config_file = argv[2];
    } else if (argc == 2) {
        input_file = argv[1];
        config_file = "res/config/default.json";
    }
}

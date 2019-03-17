/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <types_reconstructor.hpp>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::localMap2Mesh(vector<CameraType> &cameras)
{
    // 获取地图的关键帧
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    // 获取关键帧的局部地图点，画出来
    // 直接keypoint转换有问题

    // camera 预先分配内存，用下标访问，而不是push_back
    int n_keyframe = vpKFs.size();
    cameras.resize(n_keyframe);

    int globalID=0; //mesh 里点的标记
    int c_id =0 ;   //相机帧号

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        // 过滤无效帧
        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            pKF = pKF->GetParent();
        }
        
        // 局部地图点转换，另外一个matchpoint好像不行
        std::set<MapPoint*> mspMapPoints= pKF->GetMapPoints();
        vector<MapPoint*> localPoint = vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
        int N=localPoint.size();

        vector<glm::vec3> points;   //存放点的信息，glm格式

        for(size_t i = 0; i < N; i++)
        {
            if(localPoint[i]->isBad() )
                continue;
            cv::Mat pos = localPoint[i]->GetWorldPos();
            
            // 临时的点
            glm::vec3 temp_p;
            temp_p.x=pos.at<float>(0);
            temp_p.y=pos.at<float>(1);
            temp_p.z=pos.at<float>(2);
            points.push_back(temp_p);
            
        }
        // 相机信息
        CameraType tempCamera;
        cv::Mat twc = pKF->GetStereoCenter();   //双目中心？ 是否要相机中心

        glm::vec3 center3D;
        center3D.x = twc.at<float>(0);
        center3D.y = twc.at<float>(1);
        center3D.z = twc.at<float>(2);

		tempCamera.idCam = c_id;
		tempCamera.center = center3D;
		tempCamera.imageWidth = 1241;
		tempCamera.imageHeight = 376;

        //cameras.push_back(tempCamera);
        // vector插入若原先内存不够，会分配新的内存，将原来的复制过去，所以涉及指针操作可能会出错
        cameras[c_id] = tempCamera;

        // 转换到mesh里的点
		for (int k = 0; k < points.size(); k++)
		{
			//NewPointType* singlePoint = new NewPointType();
			PointType* singlePoint = new PointType();

			singlePoint->position = points[k];
			singlePoint->idPoint = globalID;
			singlePoint->addCamera(&cameras[c_id]);

			(cameras[c_id].visiblePointsT).insert(singlePoint);

			globalID++;
		}
        c_id++;
    }
    cout<<"--------------------------------------------------------"<<endl;
    cout<<"转换点数："<<globalID<<endl;
}

void System::localMapPoint2Cloud()
{
    cout<<"正在将图像转换为点云..."<<endl;
    // 定义点云使用的格式:这里用的是 XYZRGB 
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );
    // 获取地图的关键帧
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    // 获取关键帧的局部地图点，画出来
    // 直接keypoint转换有问题

    string filename = "localmap_cloud.txt";
    ofstream f;
    f.open(filename.c_str());
    f << fixed;
    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        // 过滤无效帧
        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            pKF = pKF->GetParent();
        }
        std::set<MapPoint*> mspMapPoints= pKF->GetMapPoints();
        vector<MapPoint*> localPoint = vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
        int N=localPoint.size();
        for(size_t i = 0; i < N; i++)
        {
            if(localPoint[i]->isBad() )
                continue;
            cv::Mat pos = localPoint[i]->GetWorldPos();

            PointT pc ;
            pc.x = pos.at<float>(0);
            pc.y = pos.at<float>(1);
            pc.z = pos.at<float>(2);
            pc.b = 0;
            pc.g = 255;
            pc.r = 0;
            pointCloud->points.push_back( pc );

                //这里讲点的坐标写入 txt文件
            f << setprecision(6) << pc.x << " " << pc.y  << " " << pc.z << " "  <<N<<endl;
        }

    }
    f.close();

    cout<< "特征点转换完毕" <<endl;
    pointCloud->is_dense = false;
    cout<<"点云共有局部地图点"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map_local.pcd", *pointCloud );
}

void System::mapPoint2Cloud()
{
    cout<<"正在将图像转换为点云..."<<endl;
    // 定义点云使用的格式:这里用的是 XYZRGB 
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );

    string filename = "mappoints_coordinate.txt";
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() )
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        //glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

        PointT pc ;
        pc.x = pos.at<float>(0);
        pc.y = pos.at<float>(1);
        pc.z = pos.at<float>(2);
        pc.b = 0;
        pc.g = 255;
        pc.r = 0;
        pointCloud->points.push_back( pc );

            //这里讲点的坐标写入 txt文件
        f << setprecision(6) << pc.x << " " << pc.y  << " " << pc.z << " "  <<endl;
    }
    cout<< "坐标输出完毕"<<endl;

    cout<< "特征点转换完毕" <<endl;
    pointCloud->is_dense = false;
    cout<<"点云共有地图点"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map_point.pcd", *pointCloud );

}

void System::Trans2PointCloud()
{
    cout<<"正在将图像转换为点云..."<<endl;
    // 定义点云使用的格式:这里用的是 XYZRGB 
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );

    string filename = "points coordinate";
    ofstream f;
    f.open(filename.c_str());
    f << fixed;


    // 获取关键帧
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            pKF = pKF->GetParent();
        }

        // 获取frame上计算出来的keypoint
        // 并将keypoint从像素坐标转换到世界坐标（空间）
        // 下面两个量是frame的成员

        // number of keypoint
        int N = pKF->N ;

        for(int i = 0; i < N; i++)
        {
            //cv::Mat UnprojectStereo(int i);
            cv::Mat position = pKF->UnprojectStereo(i);

            // mat 为空的时候
            if(position.dims == 0) continue;

            // point3f x,y,z访问
            PointT pc ;
            pc.x = position.at<float>(0);
            pc.y = position.at<float>(1);
            pc.z = position.at<float>(2);
            pc.b = 0;
            pc.g = 255;
            pc.r = 0;
            pointCloud->points.push_back( pc );

            //这里讲点的坐标写入 txt文件
            f << setprecision(6) << pc.x << " " << pc.y  << " " << pc.z << " "  <<endl;
        }
       
       
    }
    f.close();
    cout<< "坐标输出完毕"<<endl;

    cout<< "特征点转换完毕" <<endl;
    pointCloud->is_dense = false;
    cout<<"点云共有关键点"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("keyframe_point.pcd", *pointCloud );

}

void System::TransPoints2Mesh(vector<CameraType> &cameras)
{
    // 获取关键帧
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    int globalID=0; //mesh 里点的标记
    int c_id =0 ;   //相机帧号
    //vector<CameraType> cameras; //相机信息存储

    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
          // 坏指针时跳过，位姿也需要变化
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        // 获取frame上计算出来的keypoint
        // 并将keypoint从像素坐标转换到世界坐标（空间）

        // number of keypoint
        int N = pKF->N ;
        vector<glm::vec3> points;   //存放点的信息，glm格式

        for(int i = 0; i < N; i++)
        {
            //cv::Mat UnprojectStereo(int i);
            cv::Mat position = pKF->UnprojectStereo(i);

            // mat 为空的时候
            if(position.dims == 0) continue;
            glm::vec3 temp_p;
            temp_p.x=position.at<float>(0);
            temp_p.y=position.at<float>(1);
            temp_p.z=position.at<float>(2);
            points.push_back(temp_p);
        }
        // 相机信息
        CameraType tempCamera;
        glm::vec3 center3D;
        center3D.x = twc.at<float>(0);
        center3D.y = twc.at<float>(1);
        center3D.z = twc.at<float>(2);

		tempCamera.idCam = c_id;
		tempCamera.center = center3D;
		tempCamera.imageWidth = 1241;
		tempCamera.imageHeight = 376;

        cameras.push_back(tempCamera);
        //cameras[c_id] = tempCamera;

        // 转换到mesh里的点
		for (int k = 0; k < points.size(); k++)
		{
			//NewPointType* singlePoint = new NewPointType();
			PointType* singlePoint = new PointType();

			singlePoint->position = points[k];
			singlePoint->idPoint = globalID;
			singlePoint->addCamera(&cameras[c_id]);

			(cameras[c_id].visiblePointsT).insert(singlePoint);

			globalID++;
		}
        c_id++;
       
    }
    cout<<"--------------------------------------------------------"<<endl;
    cout<<"转换点数："<<globalID<<endl;
}

/*
void System::TransPoints2Mesh(const string &strSettingsFile)
{
    cout << "开始转换特征点信息" <<endl;
    // 读取配置信息
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).

    //记录相机 id
    int c_id=0;
    int globalID=0;

    cout<<"正在将图像转换为点云..."<<endl;
    // 定义点云使用的格式:这里用的是 XYZRGB 
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );

    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, c_id++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);


        // 获取frame上计算出来的keypoint
        // 并将keypoint从像素坐标转换到世界坐标（空间）
        // 下面两个量是frame的成员
        std::vector<cv::KeyPoint> keyPoints = pKF->mvKeysUn;
        std::vector<float> depths = pKF->mvDepth;

        //CameraType tempCamera;

		//vector<glm::vec3> points;
		
        // 相机内参信息
        float cx = fsSettings["Camera.cx"];
        float cy = fsSettings["Camera.cy"];
        float fx = fsSettings["Camera.fx"];
        float fy = fsSettings["Camera.fx"];
        // 双目没有这个参数
        //float depthScale;
        float imageWidth = fsSettings["Camera.width"];
        float imageHeight = fsSettings["Camera.height"];

        // 帧内特征点转换
        for(int i = 0; i < keyPoints.size(); i++)
        {   
            // 像素坐标
            float x = keyPoints[i].pt.x;
            float y = keyPoints[i].pt.y;
            float d = depths[i];

            if ( d==0 ) continue; // 为 0 表示没有测量到

            cv::Mat p = cv::Mat(4, 1, CV_32FC1);

			p.ptr<float>(2)[0] = d ;

			p.ptr<float>(0)[0] = (x - cx) * d / fx;
			p.ptr<float>(1)[0] = (y - cy) * d / fy;
			p.ptr<float>(3)[0] = 1;

			cv::Mat temp = cv::Mat(4, 1, CV_32FC1);
			temp = Tcw * p;
			cv::Point3f newCoord = cv::Point3f(temp.ptr<float>(0)[0] / temp.ptr<float>(3)[0],
					temp.ptr<float>(1)[0] / temp.ptr<float>(3)[0], temp.ptr<float>(2)[0] / temp.ptr<float>(3)[0]);

			//points.push_back(fromCV2GLM(newCoord));


            // point3f x,y,z访问
            PointT pc ;
            pc.x = newCoord.x;
            pc.y = newCoord.y;
            pc.z = newCoord.z;
            pc.b = 0;
            pc.g = 255;
            pc.r = 0;
            pointCloud->points.push_back( pc );

        }

        /*
        glm::vec3 center3D;
        center3D.x = twc.at<float>(0);
        center3D.y = twc.at<float>(1);
        center3D.z = twc.at<float>(2);

		tempCamera.idCam = c_id;
		tempCamera.center = center3D;
		tempCamera.imageWidth = imageWidth;
		tempCamera.imageHeight = imageHeight;

        cameras[c_id] = tempCamera;

		//set<NewPointType> tempFramePoints;
        // 转换到mesh里的点
		for (int k = 0; k < points.size(); k++)
		{
			//NewPointType* singlePoint = new NewPointType();
			PointType* singlePoint = new PointType();

			singlePoint->position = points[k];
			singlePoint->idPoint = globalID;
			singlePoint->addCamera(&cameras[c_id]);

			(cameras[c_id].visiblePointsT).insert(singlePoint);

			globalID++;
		}
       
    }

    cout<< "特征点转换完毕" <<endl;
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud );

}
*/

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM

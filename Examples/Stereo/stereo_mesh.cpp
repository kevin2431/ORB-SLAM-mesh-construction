#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

// mesh 部分头分件
#include <cstdlib>
#include <map>
#include <set>
#include <utility>

#include <CameraPointsCollection.h>
#include <Chronometer.h>
#include <Logger.h>
#include <ReconstructFromSLAMData.h>
#include <OpenMvgParser.h>
#include <ConfigParser.h>
#include <types_config.hpp>
#include <types_reconstructor.hpp>

#include <points_filtering.hpp>

//void printUsage(char *name);

int maxIterations_ = 0;
std::string input_file;
std::string config_file;

// slam 部分
using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }


// -----------------------------------
// slam 部分处理图片
// **********************************
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    //SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    //SLAM.Trans2PointCloud();

    vector<CameraType> cameras;
    SLAM.TransPoints2Mesh(cameras);

    // --------------------------------------------
    // 点转换完毕，开始mesh操作
    // ********************************************
    // mesh 配置文件
    config_file = "res/config/default.json";

    ManifoldReconstructionConfig confManif;
    ConfigParser configParser = ConfigParser();
    confManif = configParser.parse(config_file);

    // m 为重建的主要控制器
    ReconstructFromSLAMData m(confManif);
    // 这里设置最大迭代步数
    m.setExpectedTotalIterationsNumber(cameras.size());

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

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

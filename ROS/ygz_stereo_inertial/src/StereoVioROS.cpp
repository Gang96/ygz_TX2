/**
 * This is the Euroc stereo visual odometry program
 * Please specify the dataset directory in the config file
*/
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<sstream>
#include <opencv2/opencv.hpp>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ygz/System.h"
#include "ygz/EurocReader.h"
#include <sensor_msgs/Imu.h>
#include <mutex>
using namespace std;
using namespace ygz;

mutex mtx;
VecIMU vimu;
class ImageGrabber
{
public:
    ImageGrabber(System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    System* mpSLAM;
    cv::Mat M1l,M2l,M1r,M2r;
};

void ImuSave(const sensor_msgs::Imu::ConstPtr& imu)
{
    if(mtx.try_lock())
    {
         double timestamp=imu->header.stamp.toSec();
         printf("imu:%f\n",timestamp);
     	 ygz::IMUData imudata(imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z,imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z,timestamp);
     	 vimu.push_back(imudata);
         mtx.unlock();
    }
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "StereoVIO");
    ros::start();

    if(argc != 2)
    {
        cerr << endl << "Usage: rosrun ygz_stereo_inertial StereoVioROS path_to_settings" << endl;
        ros::shutdown();
        return 1;
    } 

    string configFile(argv[1]);
    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
    if (fsSettings.isOpened() == false) {
        LOG(FATAL) << "Cannot load the config file from " << argv[1] << endl;
    }
    
    System system(argv[1]);
    ImageGrabber igb(&system);
    
    // rectification parameters
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
        D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return 1;
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l,
                                igb.M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r,
                                igb.M2r);
    
    // read TBC
    cv::Mat Rbc, tbc;
    fsSettings["RBC"] >> Rbc;
    fsSettings["TBC"] >> tbc;
    if (!Rbc.empty() && tbc.empty()) {
        Matrix3d Rbc_;
        Vector3d tbc_;
        Rbc_ <<
             Rbc.at<double>(0, 0), Rbc.at<double>(0, 1), Rbc.at<double>(0, 2),
                Rbc.at<double>(1, 0), Rbc.at<double>(1, 1), Rbc.at<double>(1, 2),
                Rbc.at<double>(2, 0), Rbc.at<double>(2, 1), Rbc.at<double>(2, 2);
        tbc_ <<
             tbc.at<double>(0, 0), tbc.at<double>(1, 0), tbc.at<double>(2, 0);

        setting::TBC = SE3d(Rbc_, tbc_);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu0", 1,);
    //imu_sub.registerCallback(boost::bind(&ImuSave,_1));
    ros::Subscriber imu_sub=nh.subscribe("/imu0",100,ImuSave);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    
    ros::spin();

    // Stop all threads
    system.Shutdown();

    ros::shutdown();

    return 0;

}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    if(mtx.try_lock())
    {
         // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrLeft;
	try
	{
            cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
	}
	catch (cv_bridge::Exception& e)
	{
            ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}

	cv_bridge::CvImageConstPtr cv_ptrRight;
	try
	{
	    cv_ptrRight = cv_bridge::toCvShare(msgRight);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}

        cv::Mat imLeft, imRight;
	cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
	cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        double timestamp=cv_ptrLeft->header.stamp.toSec();
        printf("image:%f\n",timestamp);
	VecIMU temp;
        size_t imuIndex = 0;
        double TimeStampPre=0;
        while(1)
        {
            const ygz::IMUData &imudata = vimu[imuIndex];
            if (imudata.mfTimeStamp > timestamp||imudata.mfTimeStamp<TimeStampPre)
                break;
            temp.push_back(imudata);
            imuIndex++;
            printf("vector:%f\n",imudata.mfTimeStamp);
            TimeStampPre=imudata.mfTimeStamp;
        }
        vimu.clear();
	mpSLAM->AddStereoIMU(imLeft,imRight,cv_ptrLeft->header.stamp.toSec(),temp);
        temp.clear();
        mtx.unlock();
    }
}



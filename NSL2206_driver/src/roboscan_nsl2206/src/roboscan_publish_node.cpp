#include "roboscan_publish_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
//#include <sensor_msgs/msg/image_encodings.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "communication_2206.h"


#include "epc_timer.h"
#include "roboscan_nsl2206_image.h"

#include <iostream>
#include <fstream>
#include <sstream>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>

using namespace ComLib;
using namespace std;



std::atomic<int> x_start = -1, y_start = -1;

static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}


//===================================================================
roboscanPublisher::roboscanPublisher() : Node("roboscan_publish_node")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

	imagePublisher1 = this->create_publisher<sensor_msgs::msg::Image>("roboscanGraysacle", qos_profile);   
	imagePublisher2 = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imagePublisher3 = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmplitude", qos_profile);
	pointCloud2Publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 
    cameraInfoPublisher  = this->create_publisher<sensor_msgs::msg::CameraInfo>("roboscanCameraInfo", qos_profile); 
    imageHeaderPublisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("roboscanImageHeader", qos_profile); 
	colorDistancePublisher = this->create_publisher<sensor_msgs::msg::Image>("colorDistance", qos_profile);


    printf("Init\n");

    roboscanPublisher::initialise();
	roboscanPublisher::parameterInit();

	
	initCommunication();
		
	communication.sigReceivedGrayscale.connect(boost::bind(&roboscanPublisher::updateGrayscaleFrame, this, _1));
	communication.sigReceivedDistance.connect(boost::bind(&roboscanPublisher::updateDistanceFrame, this, _1));
	communication.sigReceivedDistanceAmplitude.connect(boost::bind(&roboscanPublisher::updateDistanceAmplitudeFrame, this, _1));
	communication.sigReceivedDistanceGrayscale.connect(boost::bind(&roboscanPublisher::updateDistanceGrayscaleFrame, this, _1));
	communication.sigReceivedLensCalibrationData.connect(boost::bind(&roboscanPublisher::updateLensCalibrationData, this, _1));
	
	imageHeaderMsg.data.resize(42);

	communication.getLensCalibrationData();

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));
    
    runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::thread_callback, this)));
    
}

roboscanPublisher::~roboscanPublisher()
{

    runThread = false;
    publisherThread->join();
    communication.close();

    printf("\nEnd roboscanPublisher()!\n");
}

void roboscanPublisher::thread_callback()
{
    while(runThread)
    {
        roboscanPublisher::update();
    }

	cv::destroyAllWindows();
	printf("end thread_callback\n");
}
	
rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    
    for (const auto &param: parameters)
	{
		if (param.get_name() == "A. imageType")
		{
			lidarParam.imageType= param.as_int();
		}
		else if (param.get_name() == "B. modFrequency")
		{
			lidarParam.modFrequency= param.as_int();
		}
		/*
		else if (param.get_name() == "C. mode")
		{
			lidarParam.mode = param.as_int();
		}
		*/
		else if (param.get_name() == "D. startStream")
		{
			lidarParam.startStream = param.as_bool();
		}
		else if (param.get_name() == "E. triggerSingleShot")
		{
			lidarParam.triggerSingleShot = param.as_bool();
		}
		else if (param.get_name() == "F. integrationTime0")
		{
			lidarParam.integrationTimeATOF1 = param.as_int();
		}
		else if (param.get_name() == "G. integrationTime1")
		{
			lidarParam.integrationTimeATOF2 = param.as_int();
		}
		else if (param.get_name() == "H. integrationTime2")
		{
			lidarParam.integrationTimeATOF3 = param.as_int();
		}
		else if (param.get_name() == "I. integrationTime3")
		{
			lidarParam.integrationTimeATOF4 = param.as_int();
		}
		else if (param.get_name() == "J. integrationTime4")
		{
			lidarParam.integrationTimeBTOF1 = param.as_int();
		}
		else if (param.get_name() == "K. integrationTime5")
		{
			lidarParam.integrationTimeBTOF2 = param.as_int();
		}
		else if (param.get_name() == "L. integrationTimeGray")
		{
			lidarParam.integrationTimeGray = param.as_int();
		}
		else if (param.get_name() == "M. temporalFilterFactor")
		{
			lidarParam.kalmanFactor = param.as_double();
		}
		else if (param.get_name() == "M. temporalFilterThreshold")
		{
			lidarParam.kalmanThreshold = param.as_int();
		}
		else if (param.get_name() == "N. spatialAverageFilter")
		{
			lidarParam.kalmanThreshold = param.as_bool();
		}
		else if (param.get_name() == "O. medianFilter")
		{
			lidarParam.medianFilter = param.as_bool();
		}
		else if (param.get_name() == "P. minAmplitude0")
		{
			lidarParam.minAmplitude1 = param.as_int();
		}
		else if (param.get_name() == "Q. minAmplitude1")
		{
			lidarParam.minAmplitude2 = param.as_int();
		}
		else if (param.get_name() == "R. minAmplitude2")
		{
			lidarParam.minAmplitude3 = param.as_int();
		}
		else if (param.get_name() == "S. minAmplitude3")
		{
			lidarParam.minAmplitude4 = param.as_int();
		}
		else if (param.get_name() == "T. minAmplitude4")
		{
			lidarParam.minAmplitude5 = param.as_int();
		}
		else if (param.get_name() == "U. offsetDistance")
		{
			lidarParam.offsetDistance = param.as_double();
		}
		else if (param.get_name() == "V. roiLeftX")
		{
			lidarParam.roi_leftX= param.as_int();
		}
		else if (param.get_name() == "W. roiTopY")
		{
			lidarParam.roi_topY= param.as_int();
		}
		else if (param.get_name() == "X. roiRightX")
		{
			lidarParam.roi_rightX= param.as_int();
		}
		else if (param.get_name() == "Y. roiBottomY")
		{
			lidarParam.roi_bottomY= param.as_int();
		}
		else if (param.get_name() == "Z. lensAngle")
		{
			lidarParam.angle= param.as_double();
		}
		else if (param.get_name() == "0. cvShow")
		{
			lidarParam.cvShow= param.as_bool();
		}
		else if (param.get_name() == "1. numHdr")
		{
			lidarParam.numHdr= param.as_int();
		}
	}
    

    lidarParam.updateParam = true;
    return result;
}


void roboscanPublisher::parameterInit()
{
    lidarParam.runVideo = false;
	usleep(100);
    switch(image_type){
    case 0: lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_GRAYSCALE;
        break;
    case 1: lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE;
       break;
    case 2: lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_AMPLITUDE;
       break;
    case 3: lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_GRAYSCALE;
       break;
    default: break;
    }

	
    lidarParam.mode = mode;
    lidarParam.integrationTimeATOF1  = integration_time_0;
    lidarParam.integrationTimeATOF2  = integration_time_1;
    lidarParam.integrationTimeATOF3  = integration_time_2;
    lidarParam.integrationTimeATOF4  = integration_time_3;
    lidarParam.integrationTimeBTOF1  = integration_time_4;
    lidarParam.integrationTimeBTOF2  = integration_time_5;
    lidarParam.integrationTimeGray   = integration_time_gray;
    lidarParam.modFrequency = mod_frequency;
    lidarParam.kalmanThreshold = temporal_filter_threshold;
    lidarParam.kalmanFactor  = temporal_filter_factor;
    lidarParam.averageFilter = spatial_average_filter;
    lidarParam.minAmplitude1 = min_amplitude_0;
    lidarParam.minAmplitude2 = min_amplitude_1;
    lidarParam.minAmplitude3 = min_amplitude_2;
    lidarParam.minAmplitude4 = min_amplitude_3;
    lidarParam.minAmplitude5 = min_amplitude_4;
    lidarParam.offsetDistance = offset_distance;
    lidarParam.angle = lens_angle_0;
    lidarParam.enableCartesian = enable_cartesian;    
    lidarParam.enableUndistortion = enable_undistortion;
    lidarParam.enableImages = enable_images;
    lidarParam.enablePointCloud = enable_point_cloud;
    lidarParam.enableImageHeader = enable_image_header;
    lidarParam.roi_leftX = roi_left_x;
    lidarParam.roi_topY = roi_top_y;
    lidarParam.roi_rightX = roi_right_x;
    lidarParam.roi_bottomY = roi_bottom_y;
    
    lidarParam.roi_rightX  -= (lidarParam.roi_rightX - lidarParam.roi_leftX + 1) % 4;
    lidarParam.roi_bottomY -= (lidarParam.roi_bottomY - lidarParam.roi_topY + 1) % 4;

    roi_right_x  = lidarParam.roi_rightX;
    roi_bottom_y = lidarParam.roi_bottomY;



    if(start_stream)
        lidarParam.runVideo = true;

    lidarParam.triggerSingleShot = trigger_single_shot;
    lidarParam.updateParam = true;




	rclcpp::Parameter pImageType("A. imageType", lidarParam.imageType);
	rclcpp::Parameter pModeFrequency("B. modFrequency", lidarParam.modFrequency);
	rclcpp::Parameter pMode("C. mode", lidarParam.mode); //Wide_field_image
	rclcpp::Parameter pStartStream("D. startStream", lidarParam.startStream);
	rclcpp::Parameter pTriggerSingleShot("E. triggerSingleShot", lidarParam.triggerSingleShot);
	rclcpp::Parameter pIntegrationTime0("F. integrationTime0", lidarParam.integrationTimeATOF1);
	rclcpp::Parameter pIntegrationTime1("G. integrationTime1", lidarParam.integrationTimeATOF2);
	rclcpp::Parameter pIntegrationTime2("H. integrationTime2", lidarParam.integrationTimeATOF3);
	rclcpp::Parameter pIntegrationTime3("I. integrationTime3", lidarParam.integrationTimeATOF4);
	rclcpp::Parameter pIntegrationTime4("J. integrationTime4", lidarParam.integrationTimeBTOF1);
	rclcpp::Parameter pIntegrationTime5("K. integrationTime5", lidarParam.integrationTimeBTOF2);
	rclcpp::Parameter pIntegrationTimeGray("L. integrationTimeGray", lidarParam.integrationTimeGray);
	rclcpp::Parameter pTemporalFilterFactor("M. temporalFilterFactor", lidarParam.kalmanFactor);
	rclcpp::Parameter pTemporalFilterThreshold("M. temporalFilterThreshold", lidarParam.kalmanThreshold);
	rclcpp::Parameter pSpatialAverageFilter("N. spatialAverageFilter", lidarParam.averageFilter);
	rclcpp::Parameter pMedianFilter("O. medianFilter", lidarParam.medianFilter);
	rclcpp::Parameter pMinAmplitude0("P. minAmplitude0", lidarParam.minAmplitude1);
	rclcpp::Parameter pMinAmplitude1("Q. minAmplitude1", lidarParam.minAmplitude2);
	rclcpp::Parameter pMinAmplitude2("R. minAmplitude2", lidarParam.minAmplitude3);
	rclcpp::Parameter pMinAmplitude3("S. minAmplitude3", lidarParam.minAmplitude4);
	rclcpp::Parameter pMinAmplitude4("T. minAmplitude4", lidarParam.minAmplitude5);
	rclcpp::Parameter pOffsetDistance("U. offsetDistance", lidarParam.offsetDistance);
	rclcpp::Parameter pRoiLeftX("V. roiLeftX", lidarParam.roi_leftX);
	rclcpp::Parameter pRoiTopY("W. roiTopY", lidarParam.roi_topY);
	rclcpp::Parameter pRoiRightX("X. roiRightX", lidarParam.roi_rightX);
	rclcpp::Parameter pRoiBottomY("Y. roiBottomY", lidarParam.roi_bottomY);
	rclcpp::Parameter pLensAngle("Z. lensAngle", lidarParam.angle);
	rclcpp::Parameter pCvshow("0. cvShow", lidarParam.cvShow);
	rclcpp::Parameter pNumhdr("1. numHdr", lidarParam.numHdr);
	
	this->declare_parameter<int>("A. imageType", lidarParam.iType);
	this->declare_parameter<int>("B. modFrequency", lidarParam.modFrequency);		
	this->declare_parameter<int>("C. mode", lidarParam.mode);
	this->declare_parameter<bool>("D. startStream", lidarParam.startStream);	
	this->declare_parameter<bool>("E. triggerSingleShot", lidarParam.triggerSingleShot);
	this->declare_parameter<int>("F. integrationTime0", lidarParam.integrationTimeATOF1);
	this->declare_parameter<int>("G. integrationTime1", lidarParam.integrationTimeATOF2);	
	this->declare_parameter<int>("H. integrationTime2", lidarParam.integrationTimeATOF3);
	this->declare_parameter<int>("I. integrationTime3", lidarParam.integrationTimeATOF4);
	this->declare_parameter<int>("J. integrationTime4", lidarParam.integrationTimeBTOF1);
	this->declare_parameter<int>("K. integrationTime5", lidarParam.integrationTimeBTOF2);
	this->declare_parameter<int>("L. integrationTimeGray", lidarParam.integrationTimeGray);
	this->declare_parameter<double>("M. temporalFilterFactor", lidarParam.kalmanFactor);
	this->declare_parameter<int>("M. temporalFilterThreshold", lidarParam.kalmanThreshold);
	this->declare_parameter<bool>("N. spatialAverageFilter", lidarParam.averageFilter);
	this->declare_parameter<bool>("O. medianFilter", lidarParam.medianFilter);
	this->declare_parameter<int>("P. minAmplitude0", lidarParam.minAmplitude1);
	this->declare_parameter<int>("Q. minAmplitude1", lidarParam.minAmplitude2);
	this->declare_parameter<int>("R. minAmplitude2", lidarParam.minAmplitude3);
	this->declare_parameter<int>("S. minAmplitude3", lidarParam.minAmplitude4);
	this->declare_parameter<int>("T. minAmplitude4", lidarParam.minAmplitude5);
	this->declare_parameter<double>("U. offsetDistance", lidarParam.offsetDistance);
	this->declare_parameter<int>("V. roiLeftX", lidarParam.roi_leftX);
	this->declare_parameter<int>("W. roiTopY", lidarParam.roi_topY);
	this->declare_parameter<int>("X. roiRightX", lidarParam.roi_rightX);
	this->declare_parameter<int>("Y. roiBottomY", lidarParam.roi_bottomY);
	this->declare_parameter<double>("Z. lensAngle", lidarParam.angle);
	this->declare_parameter<bool>("0. cvShow", lidarParam.cvShow);
	this->declare_parameter<int>("1. numHdr", lidarParam.numHdr);
	

	this->set_parameter(pImageType);
	this->set_parameter(pModeFrequency);	
	//this->set_parameter(pMode);
	this->set_parameter(pStartStream);
	this->set_parameter(pTriggerSingleShot);
	this->set_parameter(pIntegrationTime0);
	this->set_parameter(pIntegrationTime1);
	this->set_parameter(pIntegrationTime2);
	this->set_parameter(pIntegrationTime3);
	this->set_parameter(pIntegrationTime4);
	this->set_parameter(pIntegrationTime5);
	this->set_parameter(pIntegrationTimeGray);
	this->set_parameter(pTemporalFilterFactor);
	this->set_parameter(pTemporalFilterThreshold);
	this->set_parameter(pSpatialAverageFilter);
	this->set_parameter(pMedianFilter);
	this->set_parameter(pMinAmplitude0);
	this->set_parameter(pMinAmplitude1);
	this->set_parameter(pMinAmplitude2);
	this->set_parameter(pMinAmplitude3);
	this->set_parameter(pMinAmplitude4);
	this->set_parameter(pOffsetDistance);
	this->set_parameter(pRoiLeftX);
	this->set_parameter(pRoiTopY);
	this->set_parameter(pRoiRightX);
	this->set_parameter(pRoiBottomY);
	this->set_parameter(pLensAngle);
	this->set_parameter(pCvshow);
	this->set_parameter(pNumhdr);
	


}

bool roboscanPublisher::setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::srv::SetCameraInfo::Response& res)
{
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;
    req.camera_info.k      = cameraInfo.k;
    req.camera_info.d      = cameraInfo.d;


    //cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    return true;
}

void roboscanPublisher::initialise()
{
	printf("Init roboscan_nsl2206 node\n");
	cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, NULL);
	
    lidarParam.runVideo = false;
    lidarParam.updateParam = false;

    angle= 60.0;
    frameSeq = 0;
    imageSize8 = 0;
    imageSize16_1 = 0;
    imageSize16_2 = 0;
    //gSettings = &settings;    
    lidarParam.runVideo = true;
    lidarParam.updateParam = false;
    lastSingleShot = false;
    strFrameID = "roboscan_frame";
    
    cameraInfo.d.resize(8);
    


    printf("initialise ok\n");
}







//NSL-2206 Driver======================================================================

void roboscanPublisher::setAngle(double angle_)
{
    angle = angle_;
    transformKoef(angle);
}

void roboscanPublisher::update()
{
    //printf("update Start\n");
    if(lidarParam.runVideo && !lidarParam.updateParam){
		std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
        roboscanPublisher::updateData(); //streaming
        std::chrono::time_point<std::chrono::system_clock> endTime = std::chrono::system_clock::now();
		float milliSecond = (float)(std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime).count())/1000;
		float fps = 1/milliSecond;
		//printf("fps : %.1f\n", fps);
		
	}else if(lidarParam.updateParam){
        roboscanPublisher::setParameters(); //update parameters

        if(lidarParam.triggerSingleShot && lidarParam.triggerSingleShot != lastSingleShot)
            roboscanPublisher::updateData(); //trigger single shot

        lastSingleShot = lidarParam.triggerSingleShot;
    }
    else 
    {
        //printf("else update\n");
    }
}

void roboscanPublisher::setParameters()
{
	cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, NULL);

    if(lidarParam.updateParam)
    {
        lidarParam.updateParam = false;
        printf("SET PARAMETERS Init\n");

		switch(lidarParam.imageType){
        case 0:
			lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_GRAYSCALE;
            break;
        case 1:
			lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE;
            break;
        case 2:
			lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_AMPLITUDE;
            break;
        case 3:
			lidarParam.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_GRAYSCALE;
            break;
        default: break;
    }
        if(lidarParam.mode == 2) communication.setMode(4); //TODO... temporay
        else communication.setMode(lidarParam.mode);

        communication.setIntegrationTime3d(0, lidarParam.integrationTimeATOF1);
        communication.setIntegrationTime3d(1, lidarParam.integrationTimeATOF2);
        communication.setIntegrationTime3d(2, lidarParam.integrationTimeATOF3);
        communication.setIntegrationTime3d(3, lidarParam.integrationTimeATOF4);
        communication.setIntegrationTime3d(4, lidarParam.integrationTimeBTOF1);
        communication.setIntegrationTimeGrayscale(lidarParam.integrationTimeGray);

		lidarParam.integrationTime3d[0] = lidarParam.integrationTimeATOF1;		
		lidarParam.integrationTime3d[1] = lidarParam.integrationTimeATOF2;		
		lidarParam.integrationTime3d[2] = lidarParam.integrationTimeATOF3;
		lidarParam.integrationTime3d[3] = lidarParam.integrationTimeATOF4;
		lidarParam.integrationTime3d[4] = lidarParam.integrationTimeBTOF1;
		lidarParam.integrationTime3d[5] = lidarParam.integrationTimeGray;

		
		setAngle(lidarParam.angle);

        if(lidarParam.modFrequency == 0) communication.setModulationFrequency( ModulationFrequency_e::MODULATION_FREQUENCY_10MHZ);
        else communication.setModulationFrequency(ModulationFrequency_e::MODULATION_FREQUENCY_20MHz);

        lidarParam.maxDistance = lidarParam.modFrequency == 0 ? 15000.0f : lidarParam.modFrequency == 1 ? 7500.0f : 50000.0f;
        

        communication.setMinimalAmplitude(0, lidarParam.minAmplitude1);

        communication.setMinimalAmplitude(1, lidarParam.minAmplitude2);

        communication.setMinimalAmplitude(2, lidarParam.minAmplitude3);
        communication.setMinimalAmplitude(3, lidarParam.minAmplitude4);
        communication.setMinimalAmplitude(4, lidarParam.minAmplitude5);
        

        communication.setOffset(lidarParam.offsetDistance);

        communication.setFilter(lidarParam.kalmanThreshold, (uint)(lidarParam.kalmanFactor*1000.0));
		communication.setRoi((unsigned int)lidarParam.roi_leftX, (unsigned int)lidarParam.roi_topY, (unsigned int)lidarParam.roi_rightX, (unsigned int)lidarParam.roi_bottomY);
        communication.setDcsFilter(lidarParam.averageFilter);

		
        if(lidarParam.numHdr == 1)
			lidarParam.numHdr = 0;
		
        communication.setHdr(lidarParam.numHdr);
		
		lidarParam.numIntegrationTime = 0;
		/*
		for (unsigned int i = 0; i < NUM_INTEGRATION_TIME_3D; i++)
		{
    		if( lidarParam.integrationTime3d[i] != 0 ) 
				lidarParam.numIntegrationTime++;
			else break;
    	}	
		*/
        if(lidarParam.startStream)
        {
        	lidarParam.runVideo = true;
        }
		else
		{
			lidarParam.runVideo = false;
		}
		
		lidarParam.updateParam = false;
        printf("SET PARAMETERS OK\n");
    }

}

void roboscanPublisher::updateLensCalibrationData(std::vector<uint8_t> data)
{
    unsigned int numberOfElements = data.size() / sizeof(double);   
    unsigned int numD = numberOfElements - 9; //9 ->3x3 cameraMatrix
    cameraInfo.d.resize(numD);
    uint8_t *p = (uint8_t *)data.data();
    for(unsigned int i = 0; i < numD; i++)
    {
        cameraInfo.d.at(i) = *((double*)(&p[i * sizeof(double)]));
        //double value = cameraInfo.d.at(i);
    }

    for(unsigned int i = numD; i < numberOfElements; i++)
    {
        cameraInfo.k.at(i-numD) = *((double*)(&p[i * sizeof(double)]));
        //double value = cameraInfo.k.at(i-numD);
    }
    updateCameraCalibration();
}


void roboscanPublisher::updateData()
{
    switch(lidarParam.iType){
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_GRAYSCALE:
              communication.getGrayscale();
              break;
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE:
              communication.getDistance();
              break;
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_AMPLITUDE:
              communication.getDistanceAmplitude();
              break;
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_GRAYSCALE:
              communication.getDistanceGrayscale();
              break;
        default: break;
    }

}

void roboscanPublisher::initCommunication(){

  communication.open(); //open serial port

  printf("set mode 0\n");
  communication.setMode(0);

  printf("ROI(%d, %d, %d, %d)\n", lidarParam.roi_leftX, lidarParam.roi_topY, lidarParam.roi_rightX, lidarParam.roi_bottomY);
  communication.setRoi((unsigned int)lidarParam.roi_leftX, (unsigned int)lidarParam.roi_topY, (unsigned int)lidarParam.roi_rightX, (unsigned int)lidarParam.roi_bottomY);

  unsigned int minor, major;
  communication.getFirmwareRelease(major, minor);
  printf("Firmware release:  major= %d  minor= %d \n",major, minor);
  
  unsigned char red, green, blue;
  colorVector = vector<Vec3b>();
  for(uint32_t i = 0; i < CommunicationConstants::PixelNsl2206::NUM_COLORS; i++)
  {
    createColorMap(CommunicationConstants::PixelNsl2206::NUM_COLORS, i, red, green, blue);    
    colorVector.push_back(Vec3b(blue, green, red)); 
  }

  //lenstransform
  lensInitialisation(sensorPointSizeMM, CommunicationConstants::PixelNsl2206::IMAGE_WIDTH, CommunicationConstants::PixelNsl2206::IMAGE_HEIGHT, 0, 0);
  
  
}


void roboscanPublisher::publishImageHeader(std::shared_ptr<Nsl2206Image> image)
{

    if(lidarParam.enableImageHeader)
    {
        imageHeaderMsg.data.at(0)  = image->getHeaderVersion();
        imageHeaderMsg.data.at(1)  = image->getFrameCounter();
        imageHeaderMsg.data.at(2)  = image->getTimestamp();
        imageHeaderMsg.data.at(3)  = image->getFirmwareVersion();
        imageHeaderMsg.data.at(4)  = image->getHardwareVersion();
        imageHeaderMsg.data.at(5)  = image->getChipID();
        imageHeaderMsg.data.at(6)  = image->getWidth();
        imageHeaderMsg.data.at(7)  = image->getHeight();
        imageHeaderMsg.data.at(8)  = image->getOriginX();
        imageHeaderMsg.data.at(9)  = image->getOriginY();
        imageHeaderMsg.data.at(10) = image->getCurrentIntegrationTime3DWF();
        imageHeaderMsg.data.at(11) = image->getCurrentIntegrationTime3DNF();
        imageHeaderMsg.data.at(12) = image->getCurrentIntegrationTimeGrayscale();
        imageHeaderMsg.data.at(13) = image->getIntegrationTimeGrayscale();
        imageHeaderMsg.data.at(14) = image->getIntegrationTime3d(0);
        imageHeaderMsg.data.at(15) = image->getIntegrationTime3d(1);
        imageHeaderMsg.data.at(16) = image->getIntegrationTime3d(2);
        imageHeaderMsg.data.at(17) = image->getIntegrationTime3d(3);
        imageHeaderMsg.data.at(18) = image->getIntegrationTime3d(4);
        imageHeaderMsg.data.at(19) = image->getIntegrationTime3d(5);
        imageHeaderMsg.data.at(20) = image->getIntegrationTime3d(6);
        imageHeaderMsg.data.at(21) = image->getIntegrationTime3d(7);
        imageHeaderMsg.data.at(22) = image->getAmplitudeLimit(0);
        imageHeaderMsg.data.at(23) = image->getAmplitudeLimit(1);
        imageHeaderMsg.data.at(24) = image->getAmplitudeLimit(2);
        imageHeaderMsg.data.at(25) = image->getAmplitudeLimit(3);
        imageHeaderMsg.data.at(26) = image->getAmplitudeLimit(4);
        imageHeaderMsg.data.at(27) = image->getOffset();
        imageHeaderMsg.data.at(28) = image->getBinningType();
        imageHeaderMsg.data.at(29) = image->getTemporalFilterDistance().getFactor();
        imageHeaderMsg.data.at(30) = image->getTemporalFilterDistance().getThreshold();

        imageHeaderMsg.data.at(31) = image->getTemporalFilterSingleValue().getFactor();
        imageHeaderMsg.data.at(32) = image->getTemporalFilterSingleValue().getThreshold();
        imageHeaderMsg.data.at(33) = image->getModulation().getFrequencyMhz();
        imageHeaderMsg.data.at(34) = image->getModulation().getChannel();
        imageHeaderMsg.data.at(35) = image->getHeaderFlags().getFlags();
        imageHeaderMsg.data.at(36) = image->getTemperature();
        imageHeaderMsg.data.at(37) = image->getBeamType();
        imageHeaderMsg.data.at(38) = image->getSingleValueDistance();
        imageHeaderMsg.data.at(39) = image->getSingleValueAmplitude();
        imageHeaderMsg.data.at(40) = image->getSingleValue(0);
        imageHeaderMsg.data.at(41) = image->getSingleValue(1);

        imageHeaderPublisher->publish(imageHeaderMsg);
    }

}


void roboscanPublisher::updateGrayscaleFrame(std::shared_ptr<ComLib::Nsl2206Image> image){

    publishImageHeader(image);
    
    static rclcpp::Clock s_rclcpp_clock;
    auto data_stamp = s_rclcpp_clock.now();

	cv::Mat imageGray(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	
    if(lidarParam.enableImages)
    {
        //img8.header.seq = frameSeq++;
        img8.header.stamp = data_stamp;
        img8.header.frame_id = strFrameID;
        img8.height = static_cast<uint32_t>(image->getHeight());
        img8.width = static_cast<uint32_t>(image->getWidth());
        img8.encoding = sensor_msgs::image_encodings::MONO8;
        img8.step = img8.width;
        img8.is_bigendian = 0;
        int numPix = img8.width * img8.height;

        if(imageSize8 != numPix){
            imageSize8 = numPix;
            img8.data.resize(numPix);
        }
		cv::Mat inImage;
        cv::Mat outImage;
		unsigned int x,y,l;
		uint8_t val = 0;
		
		if(lidarParam.enableUndistortion)
        {
            inImage.create(img8.height, img8.width, CV_8U);
            outImage.create(img8.height, img8.width, CV_8U);

            for(l=0, y=0; y< img8.height; y++ )
                for(x=0; x< img8.width; x++, l++)
                    inImage.at<uint8_t>(y,x)= image->getGrayscaleOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img8.width,  img8.height));

            for(l=0, y=0; y< img8.height; y++ )
                for(x=0; x< img8.width; x++, l++)
                    img8.data[l] = outImage.at<uint8_t>(y,x);

        }else{

            for(int i=0; i< numPix; i++ )
                img8.data[i] = image->getGrayscaleOfPixel(i);
        }

		int p;
    	for(p=0, y=0; y< image->getHeight(); y++)
    	{	
        	for(x=0; x< image->getWidth(); x++, p++)
        	{
            	if(lidarParam.enableUndistortion) val = outImage.at<uint8_t>(y,x);
            	else val = image->getGrayscaleOfPixel(p);

				grayData[(int)image->getWidth() * y + x] = val;
				getGrayscaleColor(imageGray, x, y, val, 255); //2048
        	}
    	}
		
        imagePublisher1->publish(img8);
		

    } //end if enableImages

	if(lidarParam.cvShow == true)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		imageGray = addDistanceInfo(imageGray, grayData, img8.width);
		cv::imshow(WIN_NAME, imageGray);
	}
	else{
		cv::destroyAllWindows();
	}
	waitKey(1);
}

    
void roboscanPublisher::updateDistanceFrame(std::shared_ptr<ComLib::Nsl2206Image> image){
    int x, y, i, l;
    static rclcpp::Clock s_rclcpp_clock2;
    auto data_stamp = s_rclcpp_clock2.now();

	//hdrHandler.onDistanceReceived(image);


    //cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	
    publishImageHeader(image);
    updateCameraInfo(image);

    cv::Mat inImage;
    cv::Mat outImage;

    if(lidarParam.enableImages)
    { 
        //img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = data_stamp;
        img16_1.header.frame_id = "roboscan_frame"; //strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());        
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        int numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(numPix * 2);
        }

        uint16_t val = 0;
		uint16_t hvl = 0;

        if(lidarParam.enableUndistortion){

           
            inImage.create(img16_1.height, img16_1.width, CV_16U);
            outImage.create(img16_1.height, img16_1.width, CV_16U);            

            for(l=0, y=0; y< (int)img16_1.height; y++ )
                for(x=0; x< (int)img16_1.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_1.width,  img16_1.height));

            for(i=0, y=0; y< (int)img16_1.height; y++ )
                for(x=0; x< (int)img16_1.width; x++, i+=2){
                    val = outImage.at<uint16_t>(y,x);
                    img16_1.data[i] =  val & 0xff;
                    img16_1.data[i+1] = (val>>8) & 0xff;
                }
        }else{

            for(i=0, l=0; l< numPix; l++, i+=2){
                val = image->getDistanceOfPixel(l);
                img16_1.data[i] =  val & 0xff;
                img16_1.data[i+1] = (val>>8) & 0xff;
            }
        }

	
    int p;
    for(p=0, y=0; y< (int)image->getHeight(); y++)
    {
        for(x=0; x< (int)image->getWidth(); x++, p++)
        {
        	//
            if(lidarParam.enableUndistortion)
            {
				val = outImage.at<uint16_t>(y,x);
            }
			else 
			{
				val = image->getDistanceOfPixel(p);
			}
            getDistanceColor(imageLidar, x, y, val);
			distData[((int)image->getWidth())* y + x] = val;
			lidarParam.integrationDistance[lidarParam.numIntegrationTime][((int)image->getWidth())* y + x] = val;
						
		}
    }

	
	if(lidarParam.numHdr == CommunicationConstants::ModeHdr::HDR_MODE_TEMPORAL)
	{
		for(p=0, y=0; y < (int)image->getHeight(); y++)
    	{
			for(x=0; x< (int)image->getWidth(); x++, p++)
        	{
				if(image->getNumIntegrationTimeUsed() == lidarParam.numIntegrationTime+1)
				{
					for (int i = 0; i < image->getNumIntegrationTimeUsed(); i++)
					{
						if(lidarParam.integrationDistance[i][((int)image->getWidth())* y + x] < CommunicationConstants::PixelNsl2206::LIMIT_VALID_PIXEL) 
						{
							val = lidarParam.integrationDistance[i][((int)image->getWidth())* y + x];
							getDistanceColor(imageLidar, x, y, val);
							distData[((int)image->getWidth())* y + x] = val;
						}
						
					}
				}
			}
		}
	
		lidarParam.numIntegrationTime++;

		if(lidarParam.numIntegrationTime < image->getNumIntegrationTimeUsed())
			return;
		if(image->getNumIntegrationTimeUsed() == lidarParam.numIntegrationTime)
			lidarParam.numIntegrationTime = 0;
	
	}

	
    imagePublisher2->publish(img16_1);
    
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image::SharedPtr msgColor;
	msgColor = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageLidar).toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    colorDistancePublisher->publish(*msgColor.get());
    
  	
  	
  
    }


    if(lidarParam.enablePointCloud)
    {
        static unsigned int sz_pc = 0;
        const size_t nPixel = image->getWidth() * image->getHeight();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
        cloud->width = static_cast<uint32_t>(image->getWidth());
		cloud->height = static_cast<uint32_t>(image->getHeight());
		cloud->is_dense = false;
		cloud->points.resize(nPixel);
		
        if(sz_pc != nPixel){
            cloud->points.resize(nPixel);
            sz_pc = nPixel;
        }

        unsigned int x,y,k;
        double px, pz, py;

        //double width2 =  image->getWidth()/2;
        //double height2 = image->getHeight()/2;
        //double alfa0 = (lens_angle * 3.14159265359) / 360.0;  // grad -> rad
        //double step = alfa0 / width2;

        uint16_t val;
        cv::Mat inImage;
        cv::Mat outImage;
	
        if(lidarParam.enableUndistortion){
            inImage.create(cloud->height, cloud->width, CV_16U);
            outImage.create(cloud->height, cloud->width, CV_16U);
            int x,y,l;

            for(l=0, y=0; y< (int)cloud->height; y++ )
                for(x=0; x< (int)cloud->width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(cloud->width,  cloud->height));
        }

        for(k=0, y=0; y < image->getHeight(); y++){
            for(x=0; x < image->getWidth(); x++, k++){
                pcl::PointXYZI &p = cloud->points[k];

                if(lidarParam.enableUndistortion) val = outImage.at<uint16_t>(y,x);
                else val = image->getDistanceOfPixel(k);


                if(val > 0 && val < lidarParam.maxDistance)
                {
                    if(lidarParam.enableCartesian){
                        //transformPixelOpt(x, y, val, px, py, pz, width2, height2, step);
                        //transformPixelOpt1(x, y, val, px, py, pz);
                        lensTransformPixel(x, y, val, px, py, pz, 0, 1);
                        p.x = pz / 1000.0;
                        p.y = px / 1000.0;
                        p.z = -py / 1000.0;
                        p.intensity = pz / 1000.0;
                    }else{
                        p.x = val/1000.0;
                        p.y = -(80-x) * 100.0 * 5;
                        p.z = (30-y) * 100.0 * 5;
                        p.intensity = val / 1000.0;
                    }
                }
                else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }

          } //ensd for x
      } //end for y

	  sensor_msgs::msg::PointCloud2 msg;
	  pcl::toROSMsg(*cloud, msg);
      pointCloud2Publisher->publish(msg);

    } //end if enablePointCloud

	if(lidarParam.cvShow == true)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		imageLidar = addDistanceInfo(imageLidar, distData, img16_1.width);

		cv::imshow(WIN_NAME, imageLidar);
	}
	else{
		cv::destroyAllWindows();
	}
	
	waitKey(1);

}

void roboscanPublisher::updateDistanceAmplitudeFrame(std::shared_ptr<Nsl2206Image> image)
{
    int x,y,i,k,l;
    static rclcpp::Clock s_rclcpp_clock3;
    auto data_stamp = s_rclcpp_clock3.now();
    uint16_t val, dvl;
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	cv::Mat amplitudeLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	
    publishImageHeader(image);

    if(lidarParam.enableImages){
        cv::Mat inImage;
        cv::Mat outImage;
        //img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = data_stamp;
        img16_1.header.frame_id = "roboscan_frame"; //strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        int numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(numPix * 2);
        }


        if(lidarParam.enableUndistortion)
        {
            inImage.create(img16_1.height, img16_1.width, CV_16U);
            outImage.create(img16_1.height, img16_1.width, CV_16U);
            int x,y,l;

            for(l=0, y=0; y< (int)img16_1.height; y++ )
                for(x=0; x< (int)img16_1.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getAmplitudeOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_1.width,  img16_1.height));

            for(i=0, y=0; y< (int)img16_1.height; y++ )
                for(x=0; x< (int)img16_1.width; x++, i+=2){
                    uint16_t val = outImage.at<uint16_t>(y,x);
                    img16_1.data[i] =  val & 0xff;
                    img16_1.data[i+1] = (val>>8) & 0xff;
                }

        }else{

            for(i=0, l=0; l< numPix; l++, i+=2 ){
                val = image->getAmplitudeOfPixel(l);
                img16_1.data[i] =  val & 0xff;
                img16_1.data[i+1] = (val>>8) & 0xff;
            }
        }
		
		int p;
		for(p=0, y=0; y< (int)image->getHeight(); y++)
		{
			for(x=0; x< (int)image->getWidth(); x++, p++)
			{
				if(lidarParam.enableUndistortion) val = outImage.at<uint16_t>(y,x);
				else val = image->getAmplitudeOfPixel(p);

				getAmplitudeColor(amplitudeLidar, x, y, val, 10000); //2897
				amplData[(int)image->getWidth() * y + x] = val;
				lidarParam.integartionAmplitude[lidarParam.numIntegrationTime][((int)image->getWidth())* y + x] = val;
			}
		}	

        


        //img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = data_stamp;
        img16_2.header.frame_id = strFrameID;
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2;
        img16_2.is_bigendian = 0;
        numPix = img16_2.width * img16_2.height;

        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(numPix * 2);
        }

        

        if(lidarParam.enableUndistortion)
        {
            cv::Mat inImage;
            cv::Mat outImage;
            inImage.create(img16_2.height, img16_2.width, CV_16U);
            outImage.create(img16_2.height, img16_2.width, CV_16U);
            int x,y,l;

            for(l=0, y=0; y< (int)img16_2.height; y++ )
                for(x=0; x< (int)img16_2.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_2.width,  img16_2.height));

            for(i=0, y=0; y< (int)img16_2.height; y++ )
                for(x=0; x< (int)img16_2.width; x++, i+=2){
                    val = outImage.at<uint16_t>(y,x);
                    img16_2.data[i] =  val & 0xff;
                    img16_2.data[i+1] = (val>>8) & 0xff;
                }
        }else{

            for(i=0, l=0; l< numPix; l++, i+=2 ){
                uint16_t val = image->getDistanceOfPixel(l);
                img16_2.data[i] =  val & 0xff;
                img16_2.data[i+1] = (val>>8) & 0xff;
            }
        }

		
        for(p=0, y=0; y< (int)image->getHeight(); y++)
        {
            for(x=0; x< (int)image->getWidth(); x++, p++)
            {
                if(lidarParam.enableUndistortion) val = outImage.at<uint16_t>(y,x);
                else val = image->getDistanceOfPixel(p);

                getDistanceColor(imageLidar, x, y, val);
				distData[(int)image->getWidth() * y + x] = val;
				lidarParam.integrationDistance[lidarParam.numIntegrationTime][((int)image->getWidth())* y + x] = val;
            }
        }   
		
		if(lidarParam.numHdr == CommunicationConstants::ModeHdr::HDR_MODE_TEMPORAL)
		{
			for(p=0, y=0; y < (int)image->getHeight(); y++)
			{
				for(x=0; x< (int)image->getWidth(); x++, p++)
				{
					if(image->getNumIntegrationTimeUsed() == lidarParam.numIntegrationTime+1)
					{
						for (int i = 0; i < image->getNumIntegrationTimeUsed(); i++)
						{
							if(lidarParam.integrationDistance[i][((int)image->getWidth())* y + x] < CommunicationConstants::PixelNsl2206::LIMIT_VALID_PIXEL) 
							{
								dvl = lidarParam.integartionAmplitude[i][((int)image->getWidth())* y + x];
								val = lidarParam.integrationDistance[i][((int)image->getWidth())* y + x];

								getAmplitudeColor(amplitudeLidar, x, y, dvl, 10000);
								getDistanceColor(imageLidar, x, y, val);

								amplData[((int)image->getWidth())* y + x] = dvl;
								distData[((int)image->getWidth())* y + x] = val;								
							}
							
						}
					}
				}
			}
		
			lidarParam.numIntegrationTime++;
		
			if(lidarParam.numIntegrationTime < image->getNumIntegrationTimeUsed())
				return;
			if(image->getNumIntegrationTimeUsed() == lidarParam.numIntegrationTime)
				lidarParam.numIntegrationTime = 0;
		
		}

		
		imagePublisher3->publish(img16_1);
        imagePublisher2->publish(img16_2);

        

        
		rclcpp::TimerBase::SharedPtr timer_;
    	sensor_msgs::msg::Image::SharedPtr msgColor;
		msgColor = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageLidar).toImageMsg();
 
    	// Publish the image to the topic defined in the publisher
    	colorDistancePublisher->publish(*msgColor.get());

		
    }


    if(lidarParam.enablePointCloud)
    {
        const size_t nPixel = image->getWidth() * image->getHeight();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;
		cloud->points.resize(nPixel);
		
        static int szAmp= 0;
        if(szAmp != (int)nPixel){
            szAmp = nPixel;
            cloud->points.resize(nPixel);
        }

        double px, pz, py;
        uint16_t val, ampl;
        cv::Mat inImage, outImage;
        cv::Mat inAmplImage, outAmplImage;

        if(lidarParam.enableUndistortion)
        {
            inImage.create(cloud->height, cloud->width, CV_16U);
            outImage.create(cloud->height, cloud->width, CV_16U);
            inAmplImage.create(cloud->height, cloud->width, CV_16U);
            outAmplImage.create(cloud->height, cloud->width, CV_16U);

            for(l=0, y=0; y< (int)cloud->height; y++ )
                for(x=0; x< (int)cloud->width; x++, l++){
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);
                    inAmplImage.at<uint16_t>(y,x)= image->getAmplitudeOfPixel(l);
                }

            cameraCalibration.undistortion(inImage, outImage, cv::Size(cloud->width,  cloud->height));
            cameraCalibration.undistortion(inAmplImage, outAmplImage, cv::Size(cloud->width,  cloud->height));
        }

        for(k=0, y=0; y < (int)image->getHeight(); y++){
            for(x=0; x < (int)image->getWidth(); x++, k++){
                pcl::PointXYZI &p = cloud->points[k];

                if(lidarParam.enableUndistortion){
                    val = outImage.at<uint16_t>(y,x);
                    ampl=  outAmplImage.at<uint16_t>(y,x);
                } else {
                    val = image->getDistanceOfPixel(k);
                    ampl= image->getAmplitudeOfPixel(k);
                }

                if(val > 0 && val < lidarParam.maxDistance)
                {
                    if(lidarParam.enableCartesian){
                        //transformPixelOpt1(x, y, val, px, py, pz);
                        lensTransformPixel(x, y, val, px, py, pz, 0, 1);
                        p.x = pz / 1000.0;
                        p.y = px / 1000.0;
                        p.z = -py / 1000.0;
                        p.intensity = ampl / 1000.0;
                    }else{
                        p.x = val / 1000.0;
                        p.y = -(80-x) / 100.0 * 5;
                        p.z = (30-y) /100.0 * 5;
                        p.intensity = ampl / 1000.0;
                    }
                }
                else{
                    p.x = std::numeric_limits<float>::quiet_NaN();
                    p.y = std::numeric_limits<float>::quiet_NaN();
                    p.z = std::numeric_limits<float>::quiet_NaN();
                }
            } //ensd for x
        } //end for y


		sensor_msgs::msg::PointCloud2 msg;
	    pcl::toROSMsg(*cloud, msg);
        pointCloud2Publisher->publish(msg);
    } //end if enable point cloud
    
	if(lidarParam.cvShow == true)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		cv::hconcat(imageLidar, amplitudeLidar, imageLidar);
		imageLidar = addDistanceInfo(imageLidar, distData, img16_1.width);

		cv::imshow(WIN_NAME, imageLidar);
	}
	else{
		cv::destroyAllWindows();
	}
	waitKey(1);
	
}


void roboscanPublisher::updateDistanceGrayscaleFrame(std::shared_ptr<Nsl2206Image> image)
{

    static rclcpp::Clock s_rclcpp_clock4;
    auto data_stamp = s_rclcpp_clock4.now();
    
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	cv::Mat imageGray(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));

    if(lidarParam.enableImages)
    {
        cv::Mat inImage;
        cv::Mat outImage;
        //img8.header.seq = frameSeq;
        img8.header.stamp = data_stamp;
        img8.header.frame_id = strFrameID; //std::to_string(0);
        img8.height = static_cast<uint32_t>(image->getHeight());
        img8.width = static_cast<uint32_t>(image->getWidth());
        img8.encoding = sensor_msgs::image_encodings::MONO8;
        img8.step = img8.width;
        img8.is_bigendian = 0;
        int numPix = img8.width * img8.height;
        int x,y,i,l;
		uint16_t val;

        if(imageSize8 != numPix){
            imageSize8 = numPix;
            img8.data.resize(numPix);
        }

        if(lidarParam.enableUndistortion)
        {

            inImage.create(img8.height, img8.width, CV_8U);
            outImage.create(img8.height, img8.width, CV_8U);

            for(l=0, y=0; y< (int)img8.height; y++ )
                for(x=0; x< (int)img8.width; x++, l++)
                    inImage.at<uint8_t>(y,x)= image->getGrayscaleOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img8.width,  img8.height));

            for(l=0, y=0; y< (int)img8.height; y++ )
                for(x=0; x< (int)img8.width; x++, l++)
                    img8.data[l] = outImage.at<uint8_t>(y,x);

        }else{

          for(l=0; l< numPix; l++)
              img8.data[l] = image->getGrayscaleOfPixel(l);
        }


		int p;
    	for(p=0, y=0; y< (int)image->getHeight(); y++)
    	{	
        	for(x=0; x< (int)image->getWidth(); x++, p++)
        	{
            	if(lidarParam.enableUndistortion) val = outImage.at<uint8_t>(y,x);
            	else val = image->getGrayscaleOfPixel(p);

				grayData[(int)image->getWidth() * y + x] = val;
				getGrayscaleColor(imageGray, x, y, val, 255);
            
        	}
    	}
		
        imagePublisher1->publish(img8);


        //img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = data_stamp;
        img16_2.header.frame_id = strFrameID;  //std::to_string(0);
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2; //f->px_size;
        img16_2.is_bigendian = 0;
        numPix = img16_2.width * img16_2.height;
        

        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(numPix * 2);
        }

        if(lidarParam.enableUndistortion)
        {
            cv::Mat inImage;
            cv::Mat outImage;
            inImage.create(img16_2.height, img16_2.width, CV_16U);
            outImage.create(img16_2.height, img16_2.width, CV_16U);

            for(l=0, y=0; y< (int)img16_2.height; y++ )
                for(x=0; x< (int)img16_2.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_2.width,  img16_2.height));

            for(i=0, y=0; y< (int)img16_2.height; y++)
                for(x=0; x< (int)img16_2.width; x++, i+=2){
                    val = outImage.at<uint16_t>(y,x);
                    img16_2.data[i] =  val & 0xff;
                    img16_2.data[i+1] = (val>>8) & 0xff;
                }
        }else{

            for(i=0, l=0; l< numPix; l++, i+=2){
                val = image->getDistanceOfPixel(l);
                img16_2.data[i] =  val & 0xff;
                img16_2.data[i+1] = (val>>8) & 0xff;
            }
        }


		for(p=0, y=0; y< (int)image->getHeight(); y++)
        {
            for(x=0; x< (int)image->getWidth(); x++, p++)
            {
                if(lidarParam.enableUndistortion) val = outImage.at<uint16_t>(y,x);
                else val = image->getDistanceOfPixel(p);

                getDistanceColor(imageLidar, x, y, val);
				distData[(int)image->getWidth() * y + x] = val;
				lidarParam.integrationDistance[lidarParam.numIntegrationTime][((int)image->getWidth())* y + x] = val;
            }
        }

		if(lidarParam.numHdr == CommunicationConstants::ModeHdr::HDR_MODE_TEMPORAL)
		{
			for(p=0, y=0; y < (int)image->getHeight(); y++)
			{
				for(x=0; x< (int)image->getWidth(); x++, p++)
				{
					if(image->getNumIntegrationTimeUsed() == lidarParam.numIntegrationTime+1)
					{
						for (int i = 0; i < image->getNumIntegrationTimeUsed(); i++)
						{
							if(lidarParam.integrationDistance[i][((int)image->getWidth())* y + x] < CommunicationConstants::PixelNsl2206::LIMIT_VALID_PIXEL) 
							{
								val = lidarParam.integrationDistance[i][((int)image->getWidth())* y + x];
								getDistanceColor(imageLidar, x, y, val);
								distData[((int)image->getWidth())* y + x] = val;
							}
							
						}
					}
				}
			}
		
			lidarParam.numIntegrationTime++;
		
			if(lidarParam.numIntegrationTime < image->getNumIntegrationTimeUsed())
				return;
			if(image->getNumIntegrationTimeUsed() == lidarParam.numIntegrationTime)
				lidarParam.numIntegrationTime = 0;
		
		}
		
        imagePublisher2->publish(img16_2);

        
        
		rclcpp::TimerBase::SharedPtr timer_;
    	sensor_msgs::msg::Image::SharedPtr msgColor;
		msgColor = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageLidar).toImageMsg();
 
    	// Publish the image to the topic defined in the publisher
    	colorDistancePublisher->publish(*msgColor.get());

	}

    if(lidarParam.enablePointCloud)
    {
        const size_t nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;
		cloud->points.resize(nPixel);
		
        static int szAmp= 0;
        if(szAmp != (int)nPixel){
            szAmp = nPixel;
            cloud->points.resize(nPixel);
        }

        int x,y,k,l;
        double px, pz, py;

        uint16_t val;
        uint8_t gray;
        cv::Mat inImage, outImage;
        cv::Mat inAmplImage, outAmplImage;

        if(lidarParam.enableUndistortion)
        {
            inImage.create(cloud->height, cloud->width, CV_16U);
            outImage.create(cloud->height, cloud->width, CV_16U);
            inAmplImage.create(cloud->height, cloud->width, CV_16U);
            outAmplImage.create(cloud->height, cloud->width, CV_16U);

            for(l=0, y=0; y< (int)cloud->height; y++ )
                for(x=0; x< (int)cloud->width; x++, l++){
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);
                    inAmplImage.at<uint16_t>(y,x)= image->getAmplitudeOfPixel(l);
                }

            cameraCalibration.undistortion(inImage, outImage, cv::Size(cloud->width,  cloud->height));
            cameraCalibration.undistortion(inAmplImage, outAmplImage, cv::Size(cloud->width,  cloud->height));
        }

        for(k=0, y=0; y < (int)image->getHeight(); y++){
            for(x=0; x < (int)image->getWidth(); x++, k++){
                pcl::PointXYZI &p = cloud->points[k];

                if(lidarParam.enableUndistortion){
                    val  = outImage.at<uint16_t>(y,x);
                    gray =  outAmplImage.at<uint16_t>(y,x);
                }else{
                    val  = image->getDistanceOfPixel(k);
                    gray = image->getGrayscaleOfPixel(k);
                }

                if(lidarParam.enableCartesian){
                    //transformPixelOpt1(x, y, val, px, py, pz);
                    lensTransformPixel(x, y, val, px, py, pz, 0, 1);

                    p.x = px;
                    p.y = py;
                    p.z = pz;
                    p.intensity = gray;
                }else{
                    p.x = x * 5;
                    p.y = y * 5;
                    p.z = val;
                    p.intensity = gray;
                }

            } //ensd for x
        } //end for y
		sensor_msgs::msg::PointCloud2 msg;
	    pcl::toROSMsg(*cloud, msg);
        pointCloud2Publisher->publish(msg);
    }
    
	if(lidarParam.cvShow == true)
	{
		getMouseEvent(mouseXpos, mouseYpos);
		cv::hconcat(imageLidar, imageGray, imageLidar);
		imageLidar = addDistanceInfo(imageLidar, distData, img16_2.width);

		cv::imshow(WIN_NAME, imageLidar);
	}
	else{
		cv::destroyAllWindows();
	}
	waitKey(1);
}


void roboscanPublisher::transformPixel(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width, double height, double angleGrad )
{
    double alfa0 = (angleGrad * 3.14159265359) / 360.0;  // grad -> rad
    double step = alfa0 / (width/2);
    double beta = (srcY - height/2) * step;
    double alfa = (srcX - width/2) * step;
    destX = srcZ * cos(beta) * sin(alfa) + width/2;
    destY = srcZ * sin(beta) + height/2;
    destZ = srcZ * cos(alfa) * cos(beta);
}


void roboscanPublisher::transformPixelOpt(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width2, double height2, double step)
{
    double beta = (srcY - height2) * step;
    double alfa = (srcX -  width2) * step;
    double cos_beta = cos(beta);
    destX = srcZ * cos_beta * sin(alfa) + width2;
    destY = srcZ * sin(beta) + height2;
    destZ = srcZ * cos(alfa) * cos_beta;
}


void roboscanPublisher::transformPixelOpt1(int srcX, int srcY, double srcZ, double &destX, double &destY, double &destZ)
{
    destX = srcZ * koefX[srcY][srcX] + 80;
    destY = srcZ * koefY[srcY][srcX] + 30;
    destZ = srcZ * koefZ[srcY][srcX];
}


void roboscanPublisher::transformKoef(double angleGrad)
{
    double alfa0 = ((angleGrad/2) * 3.14159265359) / 360.0;  // grad -> rad
    double step = alfa0 / (160/2);

    for(int y=0; y<60; y++){
        double beta = (y - 60/2) * step;
        for(int x=0; x<160; x++){
            double alfa = (x - 160/2) * step;
            koefX[y][x] = cos(beta) * sin(alfa);
            koefY[y][x] = sin(beta);
            koefZ[y][x] = cos(alfa) * cos(beta);
        }
    }

}

void roboscanPublisher::updateCameraInfo(std::shared_ptr<ComLib::Nsl2206Image> image){

    //cameraInfo.header = header;
    //The image dimensions with which the camera was calibrated.
    cameraInfo.width = image->getWidth();
    cameraInfo.height = image->getHeight();
    cameraInfo.roi.x_offset = 0;
    cameraInfo.roi.y_offset = 0;
    cameraInfo.roi.width  = image->getWidth();
    cameraInfo.roi.height = image->getHeight();
}

void roboscanPublisher::updateCameraCalibration()
{
    printf("updateCameraCalibration Start\n");
    cv::Mat cameraMatrix = cameraCalibration.getCameraMatrix();
    cv::Mat distCoeffs = cameraCalibration.getDistortionCoeffs();

    int rows, cols, l;
    for(l=0, rows = 0; rows<3; rows++)
        for(cols=0; cols<3; cols++, l++)
            cameraMatrix.at<double>(rows, cols) = cameraInfo.k.at(l);

    for(l=0; l< (int)cameraInfo.d.size(); l++)
        distCoeffs.at<double>(l, 0) = cameraInfo.d.at(l);

    cameraCalibration.setCameraMatrix(cameraMatrix);
    cameraCalibration.setDistortionCoeffs(distCoeffs);
    printf("updateCameraCalibration End\n");
}

void roboscanPublisher::getDistanceColor(cv::Mat &imageLidar, int x, int y, int value)
{   
    double indexDistFactorColor = CommunicationConstants::PixelNsl2206::NUM_COLORS / lidarParam.maxDistance;
    if (value == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 128;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 255;
        imageLidar.at<Vec3b>(y, x)[1] = 14;
        imageLidar.at<Vec3b>(y, x)[2] = 169; 
    }
    else if(value == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 0;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else if(value == CommunicationConstants::PixelNsl2206::INTERFERENCE)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 255;
        imageLidar.at<Vec3b>(y, x)[1] = 255;
        imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value == CommunicationConstants::PixelNsl2206::EDGE_DETECTED)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 0;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else if(value == 0)
    {
        imageLidar.at<Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
    }
    else if (value < 0)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 0;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else if (value > lidarParam.maxDistance)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 0;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else{
        int index = colorVector.size() - (value*indexDistFactorColor);
        if( index < 0 || index > (int)colorVector.size() ){
            printf("error index = %d\n", index);
            index = 0;
        }
        imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
    }
}


void roboscanPublisher::getAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if( value == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == CommunicationConstants::PixelNsl2206::EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * (CommunicationConstants::PixelNsl2206::NUM_COLORS / end_range);
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index >= (int)colorVector.size() ){
			index = colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}



void roboscanPublisher::getGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range)
{   
    if (value == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 128;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 255;
        imageLidar.at<Vec3b>(y, x)[1] = 14;
        imageLidar.at<Vec3b>(y, x)[2] = 169; 
    }
    else if (value > end_range)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 255;
        imageLidar.at<Vec3b>(y, x)[1] = 255;
        imageLidar.at<Vec3b>(y, x)[2] = 255; 
    }
    else if (value < 0)
    {
        imageLidar.at<Vec3b>(y, x)[0] = 0;
        imageLidar.at<Vec3b>(y, x)[1] = 0;
        imageLidar.at<Vec3b>(y, x)[2] = 0; 
    }
    else
    {
        int color = value * (255/end_range);
        //printf("color index = %d\n", color);
        imageLidar.at<Vec3b>(y, x)[0] = color;
        imageLidar.at<Vec3b>(y, x)[1] = color;
        imageLidar.at<Vec3b>(y, x)[2] = color; 
    }
}



void roboscanPublisher::createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    /*double B0 = -0.125;
    double B1 = B0 + 0.25;
    double B2 = B1 + 0.25;
    double B3 = B2 + 0.25;
    double G0 = B1;
    double G1 = G0 + 0.25;
    double G2 = G1 + 0.25;
    double G3 = G2 + 0.25;
    double R0 = B2;
    double R1 = R0 + 0.25;
    double R2 = R1 + 0.25;
    double R3 = R2 + 0.25;*/
    double k = 1;
    double BB0 = -0.125 * k - 0.25;
    double B1 = BB0 + 0.25 * k;
    double B2 = B1 + 0.25 * k;
    double B3 = B2 + 0.25 * k;
    double G0 = B1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;
    double R0 = B2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;
    double i = (double)indx/(double)numSteps - 0.25 * k;
    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }
    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }
    if( i>= BB0 && i < B1 ){
        blue = interpolate(i, BB0, 0, B1, 255);
    } else if((i >= B1)&&(i < B2)){
        blue = 255;
    } else if((i >= B2)&&(i < B3)) {
        blue = interpolate(i, B2, 255, B3, 0);
    } else{
        blue = 0;
    }
}

void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}


cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, int distData[], int width)
{
	if( mouseXpos > 0 || (mouseYpos > 0 && mouseYpos < 60)){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(mouseXpos-10, mouseYpos), cv::Point(mouseXpos+10, mouseYpos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(mouseXpos, mouseYpos-10), cv::Point(mouseXpos, mouseYpos+10), cv::Scalar(255, 255, 0), 2);

		if( mouseXpos >= 160 ) mouseXpos -= 160;

		std::string xy_caption;
		std::string dist_caption;
		
		
		//int real_xpos = 159-mouseXpos;
		unsigned int real_dist = distData[width*mouseYpos + mouseXpos];
		if( real_dist > CommunicationConstants::PixelNsl2206::LIMIT_VALID_PIXEL ){
			if( real_dist == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE )
			{
				xy_caption = cv::format("X:%d, Y:%d", mouseXpos, mouseYpos);
				dist_caption = cv::format("LOW_AMPLITUDE");
			}
			else if( real_dist == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW )
			{
				xy_caption = cv::format("X:%d, Y:%d", mouseXpos, mouseYpos);
				dist_caption = cv::format("ADC_OVERFLOW");
			}
			else if( real_dist == CommunicationConstants::PixelNsl2206::VALUE_SATURATION )
			{
				xy_caption = cv::format("X:%d, Y:%d", mouseXpos, mouseYpos);
				dist_caption = cv::format("SATURATION");
			}
			else if( real_dist == CommunicationConstants::PixelNsl2206::INTERFERENCE )
			{
				xy_caption = cv::format("X:%d, Y:%d", mouseXpos, mouseYpos);
				dist_caption = cv::format("INTERFERENCE");
			}
			else if( real_dist == CommunicationConstants::PixelNsl2206::EDGE_DETECTED)
			{
				xy_caption = cv::format("X:%d, Y:%d", mouseXpos, mouseYpos);
				dist_caption = cv::format("EDGE_DETECTED");

			}
		}
		else{
			xy_caption = cv::format("X:%d, Y:%d", mouseXpos, mouseYpos);

			if(lidarParam.iType != 0)
				dist_caption = cv::format("%d mm", distData[width*mouseYpos + mouseXpos]);
			else
				dist_caption = cv::format("%d ", distData[width*mouseYpos + mouseXpos]);
		}

		putText(infoImage, xy_caption.c_str(), cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 0));
		putText(infoImage, dist_caption.c_str(), cv::Point(10, 40), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 0));
		
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));
		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}


double roboscanPublisher::interpolate(double x, double x0, double y0, double x1, double y1)
{  
    if( x1 == x0 )
    {
        return y0;
    }
    else
    {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }
}


void roboscanPublisher::initLensDistortionTable()
{
    if(lidarParam.mode == 1) // 50
    {
        distortionTableSize = 46;

        lens_angle[ 0   ] =     0   ;
        lens_angle[ 1   ] =     1.546388367 ;
        lens_angle[ 2   ] =     3.092776734 ;
        lens_angle[ 3   ] =     4.6391651   ;
        lens_angle[ 4   ] =     6.185553467 ;
        lens_angle[ 5   ] =     7.731941834 ;
        lens_angle[ 6   ] =     9.278330201 ;
        lens_angle[ 7   ] =     10.82471857 ;
        lens_angle[ 8   ] =     12.37110693 ;
        lens_angle[ 9   ] =     13.9174953  ;
        lens_angle[ 10  ] =     15.46388367 ;
        lens_angle[ 11  ] =     17.01027203 ;
        lens_angle[ 12  ] =     18.5566604  ;
        lens_angle[ 13  ] =     20.10304877 ;
        lens_angle[ 14  ] =     21.64943713 ;
        lens_angle[ 15  ] =     23.1958255  ;
        lens_angle[ 16  ] =     24.74221387 ;
        lens_angle[ 17  ] =     26.28860223 ;
        lens_angle[ 18  ] =     27.8349906  ;
        lens_angle[ 19  ] =     29.38137897 ;
        lens_angle[ 20  ] =     30.92776734 ;
        lens_angle[ 21  ] =     32.4741557  ;
        lens_angle[ 22  ] =     34.02054407 ;
        lens_angle[ 23  ] =     35.56693244 ;
        lens_angle[ 24  ] =     37.1133208  ;
        lens_angle[ 25  ] =     38.65970917 ;
        lens_angle[ 26  ] =     40.20609754 ;
        lens_angle[ 27  ] =     41.7524859  ;
        lens_angle[ 28  ] =     43.29887427 ;
        lens_angle[ 29  ] =     44.84526264 ;
        lens_angle[ 30  ] =     46.391651   ;
        lens_angle[ 31  ] =     47.93803937 ;
        lens_angle[ 32  ] =     49.48442774 ;
        lens_angle[ 33  ] =     51.0308161  ;
        lens_angle[ 34  ] =     52.57720447 ;
        lens_angle[ 35  ] =     54.12359284 ;
        lens_angle[ 36  ] =     55.6699812  ;
        lens_angle[ 37  ] =     57.21636957 ;
        lens_angle[ 38  ] =     58.76275794 ;
        lens_angle[ 39  ] =     60.3091463  ;
        lens_angle[ 40  ] =     61.85553467 ;
        lens_angle[ 41  ] =     63.40192304 ;
        lens_angle[ 42  ] =     64.9483114  ;
        lens_angle[ 43  ] =     66.49469977 ;
        lens_angle[ 44  ] =     68.04108814 ;
        lens_angle[ 45  ] =     69.5874765  ;

        //size mm
        lens_rp[    0   ] =     0   ;
        lens_rp[    1   ] =     0.1 ;
        lens_rp[    2   ] =     0.2 ;
        lens_rp[    3   ] =     0.3 ;
        lens_rp[    4   ] =     0.4 ;
        lens_rp[    5   ] =     0.5 ;
        lens_rp[    6   ] =     0.6 ;
        lens_rp[    7   ] =     0.7 ;
        lens_rp[    8   ] =     0.8 ;
        lens_rp[    9   ] =     0.9 ;
        lens_rp[    10  ] =     1   ;
        lens_rp[    11  ] =     1.1 ;
        lens_rp[    12  ] =     1.2 ;
        lens_rp[    13  ] =     1.3 ;
        lens_rp[    14  ] =     1.4 ;
        lens_rp[    15  ] =     1.5 ;
        lens_rp[    16  ] =     1.6 ;
        lens_rp[    17  ] =     1.7 ;
        lens_rp[    18  ] =     1.8 ;
        lens_rp[    19  ] =     1.9 ;
        lens_rp[    20  ] =     2   ;
        lens_rp[    21  ] =     2.1 ;
        lens_rp[    22  ] =     2.2 ;
        lens_rp[    23  ] =     2.3 ;
        lens_rp[    24  ] =     2.4 ;
        lens_rp[    25  ] =     2.5 ;
        lens_rp[    26  ] =     2.6 ;
        lens_rp[    27  ] =     2.7 ;
        lens_rp[    28  ] =     2.8 ;
        lens_rp[    29  ] =     2.9 ;
        lens_rp[    30  ] =     3   ;
        lens_rp[    31  ] =     3.1 ;
        lens_rp[    32  ] =     3.2 ;
        lens_rp[    33  ] =     3.3 ;
        lens_rp[    34  ] =     3.4 ;
        lens_rp[    35  ] =     3.5 ;
        lens_rp[    36  ] =     3.6 ;
        lens_rp[    37  ] =     3.7 ;
        lens_rp[    38  ] =     3.8 ;
        lens_rp[    39  ] =     3.9 ;
        lens_rp[    40  ] =     4   ;
        lens_rp[    41  ] =     4.1 ;
        lens_rp[    42  ] =     4.2 ;
        lens_rp[    43  ] =     4.3 ;
        lens_rp[    44  ] =     4.4 ;
        lens_rp[    45  ] =     4.5 ;
    }
    else if(lidarParam.mode == 0) //110
    {
        distortionTableSize = 46;

        lens_angle[ 0   ] =     0   ;
        lens_angle[ 1   ] =     3.458333333 ;
        lens_angle[ 2   ] =     6.916666667 ;
        lens_angle[ 3   ] =     10.375  ;
        lens_angle[ 4   ] =     13.83333333 ;
        lens_angle[ 5   ] =     17.29166667 ;
        lens_angle[ 6   ] =     20.75   ;
        lens_angle[ 7   ] =     24.20833333 ;
        lens_angle[ 8   ] =     27.66666667 ;
        lens_angle[ 9   ] =     31.125  ;
        lens_angle[ 10  ] =     34.58333333 ;
        lens_angle[ 11  ] =     38.04166667 ;
        lens_angle[ 12  ] =     41.5    ;
        lens_angle[ 13  ] =     44.95833333 ;
        lens_angle[ 14  ] =     48.41666667 ;
        lens_angle[ 15  ] =     51.875  ;
        lens_angle[ 16  ] =     55.33333333 ;
        lens_angle[ 17  ] =     58.79166667 ;
        lens_angle[ 18  ] =     62.25   ;
        lens_angle[ 19  ] =     65.70833333 ;
        lens_angle[ 20  ] =     69.16666667 ;
        lens_angle[ 21  ] =     72.625  ;
        lens_angle[ 22  ] =     76.08333333 ;
        lens_angle[ 23  ] =     79.54166667 ;
        lens_angle[ 24  ] =     83  ;
        lens_angle[ 25  ] =     86.45833333 ;
        lens_angle[ 26  ] =     89.91666667 ;
        lens_angle[ 27  ] =     93.375  ;
        lens_angle[ 28  ] =     96.83333333 ;
        lens_angle[ 29  ] =     100.2916667 ;
        lens_angle[ 30  ] =     103.75  ;
        lens_angle[ 31  ] =     107.2083333 ;
        lens_angle[ 32  ] =     110.6666667 ;
        lens_angle[ 33  ] =     114.125 ;
        lens_angle[ 34  ] =     117.5833333 ;
        lens_angle[ 35  ] =     121.0416667 ;
        lens_angle[ 36  ] =     124.5   ;
        lens_angle[ 37  ] =     127.9583333 ;
        lens_angle[ 38  ] =     131.4166667 ;
        lens_angle[ 39  ] =     134.875 ;
        lens_angle[ 40  ] =     138.3333333 ;
        lens_angle[ 41  ] =     141.7916667 ;
        lens_angle[ 42  ] =     145.25  ;
        lens_angle[ 43  ] =     148.7083333 ;
        lens_angle[ 44  ] =     152.1666667 ;
        lens_angle[ 45  ] =     155.625 ;



        //size mm
        lens_rp[    0   ] =     0   ;
        lens_rp[    1   ] =     0.1 ;
        lens_rp[    2   ] =     0.2 ;
        lens_rp[    3   ] =     0.3 ;
        lens_rp[    4   ] =     0.4 ;
        lens_rp[    5   ] =     0.5 ;
        lens_rp[    6   ] =     0.6 ;
        lens_rp[    7   ] =     0.7 ;
        lens_rp[    8   ] =     0.8 ;
        lens_rp[    9   ] =     0.9 ;
        lens_rp[    10  ] =     1   ;
        lens_rp[    11  ] =     1.1 ;
        lens_rp[    12  ] =     1.2 ;
        lens_rp[    13  ] =     1.3 ;
        lens_rp[    14  ] =     1.4 ;
        lens_rp[    15  ] =     1.5 ;
        lens_rp[    16  ] =     1.6 ;
        lens_rp[    17  ] =     1.7 ;
        lens_rp[    18  ] =     1.8 ;
        lens_rp[    19  ] =     1.9 ;
        lens_rp[    20  ] =     2   ;
        lens_rp[    21  ] =     2.1 ;
        lens_rp[    22  ] =     2.2 ;
        lens_rp[    23  ] =     2.3 ;
        lens_rp[    24  ] =     2.4 ;
        lens_rp[    25  ] =     2.5 ;
        lens_rp[    26  ] =     2.6 ;
        lens_rp[    27  ] =     2.7 ;
        lens_rp[    28  ] =     2.8 ;
        lens_rp[    29  ] =     2.9 ;
        lens_rp[    30  ] =     3   ;
        lens_rp[    31  ] =     3.1 ;
        lens_rp[    32  ] =     3.2 ;
        lens_rp[    33  ] =     3.3 ;
        lens_rp[    34  ] =     3.4 ;
        lens_rp[    35  ] =     3.5 ;
        lens_rp[    36  ] =     3.6 ;
        lens_rp[    37  ] =     3.7 ;
        lens_rp[    38  ] =     3.8 ;
        lens_rp[    39  ] =     3.9 ;
        lens_rp[    40  ] =     4   ;
        lens_rp[    41  ] =     4.1 ;
        lens_rp[    42  ] =     4.2 ;
        lens_rp[    43  ] =     4.3 ;
        lens_rp[    44  ] =     4.4 ;
        lens_rp[    45  ] =     4.5 ;
    }
    else
    {
        distortionTableSize = 46;

        lens_angle[ 0   ] =     0   ;
        lens_angle[ 1   ] =     1.546388367 ;
        lens_angle[ 2   ] =     3.092776734 ;
        lens_angle[ 3   ] =     4.6391651   ;
        lens_angle[ 4   ] =     6.185553467 ;
        lens_angle[ 5   ] =     7.731941834 ;
        lens_angle[ 6   ] =     9.278330201 ;
        lens_angle[ 7   ] =     10.82471857 ;
        lens_angle[ 8   ] =     12.37110693 ;
        lens_angle[ 9   ] =     13.9174953  ;
        lens_angle[ 10  ] =     15.46388367 ;
        lens_angle[ 11  ] =     17.01027203 ;
        lens_angle[ 12  ] =     18.5566604  ;
        lens_angle[ 13  ] =     20.10304877 ;
        lens_angle[ 14  ] =     21.64943713 ;
        lens_angle[ 15  ] =     23.1958255  ;
        lens_angle[ 16  ] =     24.74221387 ;
        lens_angle[ 17  ] =     26.28860223 ;
        lens_angle[ 18  ] =     27.8349906  ;
        lens_angle[ 19  ] =     29.38137897 ;
        lens_angle[ 20  ] =     30.92776734 ;
        lens_angle[ 21  ] =     32.4741557  ;
        lens_angle[ 22  ] =     34.02054407 ;
        lens_angle[ 23  ] =     35.56693244 ;
        lens_angle[ 24  ] =     37.1133208  ;
        lens_angle[ 25  ] =     38.65970917 ;
        lens_angle[ 26  ] =     40.20609754 ;
        lens_angle[ 27  ] =     41.7524859  ;
        lens_angle[ 28  ] =     43.29887427 ;
        lens_angle[ 29  ] =     44.84526264 ;
        lens_angle[ 30  ] =     46.391651   ;
        lens_angle[ 31  ] =     47.93803937 ;
        lens_angle[ 32  ] =     49.48442774 ;
        lens_angle[ 33  ] =     51.0308161  ;
        lens_angle[ 34  ] =     52.57720447 ;
        lens_angle[ 35  ] =     54.12359284 ;
        lens_angle[ 36  ] =     55.6699812  ;
        lens_angle[ 37  ] =     57.21636957 ;
        lens_angle[ 38  ] =     58.76275794 ;
        lens_angle[ 39  ] =     60.3091463  ;
        lens_angle[ 40  ] =     61.85553467 ;
        lens_angle[ 41  ] =     63.40192304 ;
        lens_angle[ 42  ] =     64.9483114  ;
        lens_angle[ 43  ] =     66.49469977 ;
        lens_angle[ 44  ] =     68.04108814 ;
        lens_angle[ 45  ] =     69.5874765  ;

        //size mm
        lens_rp[    0   ] =     0   ;
        lens_rp[    1   ] =     0.1 ;
        lens_rp[    2   ] =     0.2 ;
        lens_rp[    3   ] =     0.3 ;
        lens_rp[    4   ] =     0.4 ;
        lens_rp[    5   ] =     0.5 ;
        lens_rp[    6   ] =     0.6 ;
        lens_rp[    7   ] =     0.7 ;
        lens_rp[    8   ] =     0.8 ;
        lens_rp[    9   ] =     0.9 ;
        lens_rp[    10  ] =     1   ;
        lens_rp[    11  ] =     1.1 ;
        lens_rp[    12  ] =     1.2 ;
        lens_rp[    13  ] =     1.3 ;
        lens_rp[    14  ] =     1.4 ;
        lens_rp[    15  ] =     1.5 ;
        lens_rp[    16  ] =     1.6 ;
        lens_rp[    17  ] =     1.7 ;
        lens_rp[    18  ] =     1.8 ;
        lens_rp[    19  ] =     1.9 ;
        lens_rp[    20  ] =     2   ;
        lens_rp[    21  ] =     2.1 ;
        lens_rp[    22  ] =     2.2 ;
        lens_rp[    23  ] =     2.3 ;
        lens_rp[    24  ] =     2.4 ;
        lens_rp[    25  ] =     2.5 ;
        lens_rp[    26  ] =     2.6 ;
        lens_rp[    27  ] =     2.7 ;
        lens_rp[    28  ] =     2.8 ;
        lens_rp[    29  ] =     2.9 ;
        lens_rp[    30  ] =     3   ;
        lens_rp[    31  ] =     3.1 ;
        lens_rp[    32  ] =     3.2 ;
        lens_rp[    33  ] =     3.3 ;
        lens_rp[    34  ] =     3.4 ;
        lens_rp[    35  ] =     3.5 ;
        lens_rp[    36  ] =     3.6 ;
        lens_rp[    37  ] =     3.7 ;
        lens_rp[    38  ] =     3.8 ;
        lens_rp[    39  ] =     3.9 ;
        lens_rp[    40  ] =     4   ;
        lens_rp[    41  ] =     4.1 ;
        lens_rp[    42  ] =     4.2 ;
        lens_rp[    43  ] =     4.3 ;
        lens_rp[    44  ] =     4.4 ;
        lens_rp[    45  ] =     4.5 ;
    }
}


double roboscanPublisher::lensInterpolate(double x_in, double x0, double y0, double x1, double y1){

    if(fabs(x1 - x0) < std::numeric_limits<double>::epsilon())  return y0;
    else return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double roboscanPublisher::lensGetAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(unsigned int i=1; i<(unsigned int)distortionTableSize; i++)
    {
        if(radius >= lens_rp[i-1] && radius <= lens_rp[i]){

            alfaGrad = lensInterpolate(radius, lens_rp[i-1], lens_angle[i-1], lens_rp[i], lens_angle[i]);
        }
    }

    return alfaGrad;
}


void roboscanPublisher::lensTransformPixel(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle)
{
    double y = srcZ * lens_yUA[srcX][srcY];
    double z = srcZ * lens_zUA[srcX][srcY];
    destX    = srcZ * lens_xUA[srcX][srcY];
    destY = z * sin_angle + y * cos_angle;
    destZ = z * cos_angle - y * sin_angle;
}


//void LensTransform::initialisation(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY, bool lensType, std::string fname, int fov)
void roboscanPublisher::lensInitialisation(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    initLensDistortionTable();

    int r0 = 1 - numRows/2 + offsetY; //lens optical center offset
    int c0 = 1 - numCols/2 + offsetX;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = lensGetAngle(c, r, sensorPointSizeMM);
            double angleRad =  angleGrad * 3.14159265 / 180.0;

            double lens_rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            lens_xUA[x][y] = c * rUA / lens_rp;
            lens_yUA[x][y] = r * rUA / lens_rp;
            lens_zUA[x][y] = cos(angleRad);
        }
    }

}




//=====================================================================================

int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);

    auto node = std::make_shared<roboscanPublisher>();
    
    printf("ROS Finish\n");
    rclcpp::spin(node);
    rclcpp::shutdown();

}















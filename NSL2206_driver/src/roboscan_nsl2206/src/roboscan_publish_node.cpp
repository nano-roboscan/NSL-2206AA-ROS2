#include <cstdio>
#include <chrono>
#include <functional>
#include <filesystem>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <pcl/conversions.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

#include "roboscan_publish_node.hpp"

using namespace NslOption;
using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

#define WIN_NAME "NSL-2206AA IMAGE"

#define DISTANCE_INFO_HEIGHT	80
#define VIEWER_SCALE_SIZE		4

std::atomic<int> x_start = -1, y_start = -1;
std::unique_ptr<NslPCD> latestFrame = std::make_unique<NslPCD>();


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

roboscanPublisher::roboscanPublisher() : 
	Node("roboscan_nsl2206_publish_node")
#ifdef image_transfer_function
	,nodeHandle(std::shared_ptr<roboscanPublisher>(this, [](auto *) {}))
	,imageTransport(nodeHandle)
	,imagePublisher(imageTransport.advertise("roboscanImage", 1000))
#endif	
{ 

    RCLCPP_INFO(this->get_logger(), "start roboscanPublisher...\n");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 

//	yaml_path_ = std::string(std::getenv("HOME")) + "/lidar_params.yaml";
	yaml_path_ = ament_index_cpp::get_package_share_directory("roboscan_nsl2206") + "/lidar_params.yaml";

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));

	reconfigure = false;
	mouseXpos = -1;
	mouseYpos = -1;
	runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::threadCallback, this)));


    RCLCPP_INFO(this->get_logger(), "\nRun rqt to view the image!\n");
} 

roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();

	nsl_close();

    RCLCPP_INFO(this->get_logger(), "\nEnd roboscanPublisher()!\n");
}

void roboscanPublisher::initNslLibrary()
{
	nslConfig.lidarAngleV = viewerParam.lidarAngleV;
	nslConfig.lidarAngleH = viewerParam.lidarAngleH;
	nsl_handle = nsl_open(viewerParam.devName.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
	if( nsl_handle < 0 ){
		std::cout << "nsl_open::handle open error::" << nsl_handle << std::endl;
		return;
	}

	nslConfig.medianOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.gaussOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	nslConfig.temporalFactorValue = 300;
	nslConfig.temporalThresholdValue = 200;
	nslConfig.edgeThresholdValue = 100;
	nslConfig.interferenceDetectionLimitValue = 0;
	nslConfig.interferenceDetectionLastValueOpt = NslOption::FUNCTION_OPTIONS::FUNC_ON;
	
	nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
	nsl_set3DFilter(nsl_handle, viewerParam.pointCloudEdgeThreshold);
	nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, viewerParam.grayScale ? NslOption::FUNCTION_OPTIONS::FUNC_ON : NslOption::FUNCTION_OPTIONS::FUNC_OFF);
	startStreaming();

	RCLCPP_INFO(this->get_logger(),"end initNslLibrary() nsl_handle = %d\n", nsl_handle);
}

void roboscanPublisher::threadCallback()
{
	auto lastTime = chrono::steady_clock::now();
	int frameCount = 0;

	while(runThread){

		if( reconfigure ){
			reconfigure = false;
			setReconfigure();
		}

		if( nsl_getPointCloudData(nsl_handle, latestFrame.get(), 0) == NSL_ERROR_TYPE::NSL_SUCCESS )
		{
			frameCount++;		
			publishFrame(latestFrame.get());
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		auto now = chrono::steady_clock::now();
		auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
		if( elapsed >= 1000 ){
			viewerParam.frameCount = frameCount;
			frameCount = 0;
			lastTime = now;
			RCLCPP_INFO(this->get_logger(), "frame = %d fps\n", viewerParam.frameCount);
			//printf("frame = %d fps\r\n", viewerParam.frameCount);
		}
		
	}

	cv::destroyAllWindows();
	RCLCPP_INFO(this->get_logger(), "end threadCallback\n");
}


rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	
	// Here update class attributes, do some actions, etc.
	for (const auto &param: parameters)
	{
		if (param.get_name() == "A. cvShow")
		{
			
			bool showCv = param.as_bool();
			if( viewerParam.cvShow != showCv ){
				viewerParam.cvShow = showCv;
				viewerParam.changedCvShow = true;
			}
			
		}
		else if (param.get_name() == "B. grayScale")
		{
			
			bool grayScale = param.as_bool();
			if( viewerParam.grayScale != grayScale ){
				viewerParam.grayScale = grayScale;
			}
			
		}
		else if (param.get_name() == "C. imageType")
		{
			string strImgType = param.as_string();
			auto itMode = modeStrMap.find(strImgType);
			int imgType = (itMode != modeStrMap.end()) ? itMode->second : 2; // defeault DISTANCE_AMPLITUDE

			if( viewerParam.imageType != imgType ){
				viewerParam.imageType = imgType;
				viewerParam.changedImageType = true;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "D. hdr_mode")
		{
			string strHdrType = param.as_string();
			auto itHdr = hdrStrMap.find(strHdrType);
			int hdr_opt = (itHdr != hdrStrMap.end()) ? itHdr->second : 0; // Hdr OFF

			nslConfig.hdrOpt = static_cast<NslOption::HDR_OPTIONS>(hdr_opt);
		}
		else if (param.get_name() == "E. int0")
		{
			nslConfig.integrationTime3D[0] = param.as_int();
		}
		else if (param.get_name() == "F. int1")
		{
			nslConfig.integrationTime3D[1] = param.as_int();
		}
		else if (param.get_name() == "G. int2")
		{
			nslConfig.integrationTime3D[2] = param.as_int();
		}
		else if (param.get_name() == "G. int3")
		{
			nslConfig.integrationTime3D[3] = param.as_int();
		}
		else if (param.get_name() == "H. intGr")
		{
			nslConfig.integrationTimeGrayScale = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude0")
		{
			nslConfig.minAmplitude[0] = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude1")
		{
			nslConfig.minAmplitude[1] = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude2")
		{
			nslConfig.minAmplitude[2] = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude3")
		{
			nslConfig.minAmplitude[3] = param.as_int();
		}
		else if (param.get_name() == "J. modIndex")
		{
			string strFreqType = param.as_string();
			auto itFreq = modulationStrMap.find(strFreqType);
			int freq_opt = (itFreq != modulationStrMap.end()) ? itFreq->second : 0; // 10Mhz

			nslConfig.mod_frequencyOpt = static_cast<NslOption::MODULATION_OPTIONS>(freq_opt);
		}
		else if (param.get_name() == "K. channel")
		{
			int ch_opt = param.as_int();
			if( ch_opt > 15 || ch_opt < 0 ) ch_opt = 0;
			nslConfig.mod_channelOpt = static_cast<NslOption::MODULATION_CH_OPTIONS>(ch_opt);
		}
		else if (param.get_name() == "L. roi_leftX")
		{
			int x1_tmp = param.as_int();
			nslConfig.roiXMin = x1_tmp;

		}
		else if (param.get_name() == "N. roi_rightX")
		{
			int x2_tmp = param.as_int();
			nslConfig.roiXMax = x2_tmp;
		}
		else if (param.get_name() == "M. roi_topY")
		{
			int y1_tmp = param.as_int();	
			nslConfig.roiYMin = y1_tmp;
		}
		else if (param.get_name() == "O. roi_bottomY")
		{
			int y2_tmp = param.as_int();
			nslConfig.roiYMax = y2_tmp;
		}
		else if (param.get_name() == "P. transformAngleV")
		{
			int lidarAngleV = param.as_double();
			if( viewerParam.lidarAngleV != lidarAngleV ){
				viewerParam.lidarAngleV = lidarAngleV;
				viewerParam.reOpenLidar = true;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "P. transformAngleH")
		{
			int lidarAngleH = param.as_double();
			if( viewerParam.lidarAngleH != lidarAngleH ){
				viewerParam.lidarAngleH = lidarAngleH;
				viewerParam.reOpenLidar = true;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "Q. frameID")
		{
			string tmpId = param.as_string();
			if( tmpId != viewerParam.frame_id ) {
				RCLCPP_INFO(this->get_logger(), "changed frameID %s -> %s\n", viewerParam.frame_id.c_str(), param.as_string().c_str());
				viewerParam.frame_id = tmpId;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "R. medianFilter")
		{
			nslConfig.medianOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "S. averageFilter")
		{
			nslConfig.gaussOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "T. temporalFilterFactor")
		{
			nslConfig.temporalFactorValue = static_cast<int>(param.as_double()*1000);
			if( nslConfig.temporalFactorValue > 1000 ) nslConfig.temporalFactorValue = 1000;
			if( nslConfig.temporalFactorValue < 0 ) nslConfig.temporalFactorValue = 0;
		}
		else if (param.get_name() == "T. temporalFilterFactorThreshold")
		{
			nslConfig.temporalThresholdValue = param.as_int();
			if( nslConfig.temporalThresholdValue < 0 ) nslConfig.temporalThresholdValue = 0;
		}
		else if (param.get_name() == "U. edgeFilterThreshold")
		{
			nslConfig.edgeThresholdValue = param.as_int();
			if( nslConfig.edgeThresholdValue < 0 ) nslConfig.edgeThresholdValue = 0;
		}
		/*
		else if (param.get_name() == "W. temporalEdgeThresholdLow")
		{
			lidarParam.temporalEdgeThresholdLow = param.as_int();
		}
		else if (param.get_name() == "X. temporalEdgeThresholdHigh")
		{
			lidarParam.temporalEdgeThresholdHigh = param.as_int();
		}
		*/
		else if (param.get_name() == "V. interferenceDetectionLimit")
		{
			nslConfig.interferenceDetectionLimitValue = param.as_int();
			if( nslConfig.interferenceDetectionLimitValue > 1000 ) nslConfig.interferenceDetectionLimitValue = 1000;
			if( nslConfig.interferenceDetectionLimitValue < 0 ) nslConfig.interferenceDetectionLimitValue = 0;
		}
		else if (param.get_name() == "V. useLastValue")
		{
			nslConfig.interferenceDetectionLastValueOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
		}
		else if (param.get_name() == "Y. PointColud EDGE")
		{
			int tmpThreshold = param.as_int();
			if( viewerParam.pointCloudEdgeThreshold != tmpThreshold ){
				viewerParam.pointCloudEdgeThreshold = tmpThreshold;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "Z. MaxDistance")
		{
			int tmpDistance = param.as_int();
			if( viewerParam.maxDistance != tmpDistance ){
				viewerParam.maxDistance = tmpDistance;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "0. devName")
		{
			string devName = param.as_string();
			if( devName != viewerParam.devName ) {
				RCLCPP_INFO(this->get_logger(), "changed IP addr %s -> %s\n", viewerParam.devName.c_str(), devName.c_str());

				viewerParam.saveParam = true;
				viewerParam.reOpenLidar = true;
				viewerParam.devName = devName;
			}
		}
	}

	reconfigure = true;
	return result;
}

void roboscanPublisher::timeDelay(int milli)
{
	auto start = std::chrono::steady_clock::now();
	while ( runThread != 0 ) {
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= milli) {
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void roboscanPublisher::renewParameter()
{
	this->set_parameter(rclcpp::Parameter("0. devName", viewerParam.devName));
	this->set_parameter(rclcpp::Parameter("C. imageType", modeIntMap.at(viewerParam.imageType)));
	this->set_parameter(rclcpp::Parameter("D. hdr_mode", hdrIntMap.at(static_cast<int>(nslConfig.hdrOpt))));
	this->set_parameter(rclcpp::Parameter("E. int0", nslConfig.integrationTime3D[0]));
	this->set_parameter(rclcpp::Parameter("F. int1", nslConfig.integrationTime3D[1]));
	this->set_parameter(rclcpp::Parameter("G. int2", nslConfig.integrationTime3D[2]));
	this->set_parameter(rclcpp::Parameter("G. int3", nslConfig.integrationTime3D[3]));
	this->set_parameter(rclcpp::Parameter("H. intGr", nslConfig.integrationTimeGrayScale));
	this->set_parameter(rclcpp::Parameter("I. minAmplitude0", nslConfig.minAmplitude[0]));
	this->set_parameter(rclcpp::Parameter("I. minAmplitude1", nslConfig.minAmplitude[1]));
	this->set_parameter(rclcpp::Parameter("I. minAmplitude2", nslConfig.minAmplitude[2]));
	this->set_parameter(rclcpp::Parameter("I. minAmplitude3", nslConfig.minAmplitude[3]));
	this->set_parameter(rclcpp::Parameter("J. modIndex", modulationIntMap.at(static_cast<int>(nslConfig.mod_frequencyOpt))));
	this->set_parameter(rclcpp::Parameter("K. channel", static_cast<int>(nslConfig.mod_channelOpt)));
	this->set_parameter(rclcpp::Parameter("L. roi_leftX", nslConfig.roiXMin));
	this->set_parameter(rclcpp::Parameter("M. roi_topY", nslConfig.roiYMin));
	this->set_parameter(rclcpp::Parameter("N. roi_rightX", nslConfig.roiXMax));
	this->set_parameter(rclcpp::Parameter("P. transformAngleV", viewerParam.lidarAngleV));
	this->set_parameter(rclcpp::Parameter("P. transformAngleH", viewerParam.lidarAngleH));
	this->set_parameter(rclcpp::Parameter("Q. frameID", viewerParam.frame_id));
	this->set_parameter(rclcpp::Parameter("R. medianFilter", static_cast<int>(nslConfig.medianOpt)));
	this->set_parameter(rclcpp::Parameter("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt)));
	this->set_parameter(rclcpp::Parameter("T. temporalFilterFactor", nslConfig.temporalFactorValue/1000.0));
	this->set_parameter(rclcpp::Parameter("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue));
	this->set_parameter(rclcpp::Parameter("U. edgeFilterThreshold", nslConfig.edgeThresholdValue));
	
	this->set_parameter(rclcpp::Parameter("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue));
	this->set_parameter(rclcpp::Parameter("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt)));
	this->set_parameter(rclcpp::Parameter("Y. PointColud EDGE", viewerParam.pointCloudEdgeThreshold));
	this->set_parameter(rclcpp::Parameter("Z. MaxDistance", viewerParam.maxDistance));
	
	

}

void roboscanPublisher::setReconfigure()
{	
	if( viewerParam.saveParam )
	{
		viewerParam.saveParam = false;
		save_params();
	}

	if( !viewerParam.changedCvShow )
	{
		nsl_streamingOff(nsl_handle);
		
		std::cout << " nsl_handle = "<< nsl_handle << "nsl_open :: reOpenLidar = "<< viewerParam.reOpenLidar << std::endl;
		
		if( nsl_handle < 0 && viewerParam.reOpenLidar ){

			nslConfig.lidarAngleV = viewerParam.lidarAngleV;
			nslConfig.lidarAngleH = viewerParam.lidarAngleH;
			nsl_handle = nsl_open(viewerParam.devName.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
			viewerParam.reOpenLidar = false;

			if( nsl_handle >= 0 ){
				renewParameter();
			}
		}
		
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, viewerParam.grayScale ? NslOption::FUNCTION_OPTIONS::FUNC_ON : NslOption::FUNCTION_OPTIONS::FUNC_OFF);		
		nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude[0], nslConfig.minAmplitude[1], nslConfig.minAmplitude[2], nslConfig.minAmplitude[3]);
		nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D[0], nslConfig.integrationTime3D[1], nslConfig.integrationTime3D[2], nslConfig.integrationTime3D[3], nslConfig.integrationTimeGrayScale);
		nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
		nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
		nsl_set3DFilter(nsl_handle, viewerParam.pointCloudEdgeThreshold);
		nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt);
		nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
		
		nsl_saveConfiguration(nsl_handle);

		startStreaming();
	}

	setWinName();
	std::cout << "end setReconfigure"<< std::endl;

}

void roboscanPublisher::setWinName()
{
	bool changedCvShow = viewerParam.changedCvShow || viewerParam.changedImageType;
	viewerParam.changedCvShow = false;
	viewerParam.changedImageType = false;
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( viewerParam.cvShow == false || changedCvShow == false ) return;
	
	if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_MODE)){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE)){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else{
		sprintf(winName,"%s(READY)", WIN_NAME);
	}
	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}

rcl_interfaces::msg::ParameterDescriptor roboscanPublisher::create_Slider(const std::string &description, int from, int to, int step)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;

    rcl_interfaces::msg::IntegerRange range;
    range.from_value = from;
    range.to_value = to ;
    range.step = step;

    desc.integer_range.push_back(range);
    return desc;
}

rcl_interfaces::msg::ParameterDescriptor roboscanPublisher::create_Slider(const std::string &description, double from, double to, double step)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;

    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    range.step = step;

    desc.floating_point_range.push_back(range);
    return desc;
}

void roboscanPublisher::initialise()
{
	std::cout << "Init roboscan_ns2206 node\n"<< std::endl;

	viewerParam.saveParam = false;
	viewerParam.frameCount = 0;
	viewerParam.cvShow = false;
	viewerParam.grayScale = false;
	viewerParam.changedCvShow = true;
	viewerParam.changedImageType = false;
	viewerParam.reOpenLidar = false;
	viewerParam.maxDistance = 15000;
	viewerParam.pointCloudEdgeThreshold = 200;
	viewerParam.imageType = 2;
	viewerParam.lidarAngleV = 0;
	viewerParam.lidarAngleH = 0;

	viewerParam.frame_id = "roboscan_nsl2206_frame";
	viewerParam.devName = "/dev/ttyNsl2206";

	load_params();
	initNslLibrary();
	setWinName();
	
	rclcpp::Parameter pDevName("0. devName", viewerParam.devName);
	rclcpp::Parameter pCvShow("A. cvShow", viewerParam.cvShow);
	rclcpp::Parameter pGrayscale("B. grayScale", viewerParam.grayScale);
	rclcpp::Parameter pImageType("C. imageType", modeIntMap.at(viewerParam.imageType));
	rclcpp::Parameter pHdr_mode("D. hdr_mode", hdrIntMap.at(static_cast<int>(nslConfig.hdrOpt)));
	rclcpp::Parameter pInt0("E. int0", nslConfig.integrationTime3D[0]);
	rclcpp::Parameter pInt1("F. int1", nslConfig.integrationTime3D[1]);
	rclcpp::Parameter pInt2("G. int2", nslConfig.integrationTime3D[2]);
	rclcpp::Parameter pInt3("G. int3", nslConfig.integrationTime3D[3]);
	rclcpp::Parameter pIntGr("H. intGr", nslConfig.integrationTimeGrayScale);
	rclcpp::Parameter pMinAmplitude0("I. minAmplitude0", nslConfig.minAmplitude[0]);
	rclcpp::Parameter pMinAmplitude1("I. minAmplitude1", nslConfig.minAmplitude[1]);
	rclcpp::Parameter pMinAmplitude2("I. minAmplitude2", nslConfig.minAmplitude[2]);
	rclcpp::Parameter pMinAmplitude3("I. minAmplitude3", nslConfig.minAmplitude[3]);
	rclcpp::Parameter pModIndex("J. modIndex", modulationIntMap.at(static_cast<int>(nslConfig.mod_frequencyOpt)));
	rclcpp::Parameter pChannel("K. channel", static_cast<int>(nslConfig.mod_channelOpt));
	rclcpp::Parameter pRoi_leftX("L. roi_leftX", nslConfig.roiXMin);
	rclcpp::Parameter pRoi_topY("M. roi_topY", nslConfig.roiYMin);
	rclcpp::Parameter pRoi_rightX("N. roi_rightX", nslConfig.roiXMax);
	rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", nslConfig.roiYMax);
	
	rclcpp::Parameter pTransformAngleV("P. transformAngleV", viewerParam.lidarAngleV);
	rclcpp::Parameter pTransformAngleH("P. transformAngleH", viewerParam.lidarAngleH);
	rclcpp::Parameter pFrameID("Q. frameID", viewerParam.frame_id);
	rclcpp::Parameter pMedianFilter("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	rclcpp::Parameter pAverageFilter("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	rclcpp::Parameter pTemporalFilterFactor("T. temporalFilterFactor", nslConfig.temporalFactorValue/1000.0);
	rclcpp::Parameter pTemporalFilterThreshold("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue);
	rclcpp::Parameter pEdgeFilterThreshold("U. edgeFilterThreshold", nslConfig.edgeThresholdValue);
	rclcpp::Parameter pInterferenceDetectionLimit("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue);
	rclcpp::Parameter pUseLastValue("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));
	rclcpp::Parameter pPCEdgeFilter("Y. PointColud EDGE", viewerParam.pointCloudEdgeThreshold);
	rclcpp::Parameter pMaxDistance("Z. MaxDistance", viewerParam.maxDistance);

	this->declare_parameter<string>("0. devName", viewerParam.devName);
	this->declare_parameter<bool>("A. cvShow", viewerParam.cvShow);
	this->declare_parameter<bool>("B. grayScale", viewerParam.grayScale);
	this->declare_parameter<string>("C. imageType", modeIntMap.at(viewerParam.imageType));
	this->declare_parameter<string>("D. hdr_mode", hdrIntMap.at(static_cast<int>(nslConfig.hdrOpt)));
	
	auto int_0 = create_Slider("Defaut integration time", 0, 2000, 1);
	this->declare_parameter<int>("E. int0", nslConfig.integrationTime3D[0], int_0);
	auto int_1 = create_Slider("HDR integration time1", 0, 2000, 1);
	this->declare_parameter<int>("F. int1", nslConfig.integrationTime3D[1], int_1);
	auto int_2 = create_Slider("HDR integration time2", 0, 2000, 1);
	this->declare_parameter<int>("G. int2", nslConfig.integrationTime3D[2], int_2);
	auto int_3 = create_Slider("HDR integration time3", 0, 2000, 1);
	this->declare_parameter<int>("G. int3", nslConfig.integrationTime3D[3], int_3);
	auto int_Gr = create_Slider("Grayscale time", 0, 10000, 1);	
	this->declare_parameter<int>("H. intGr",nslConfig.integrationTimeGrayScale, int_Gr);
	
	auto min_Amplitude0 = create_Slider("minimum Amplitude 0", 0, 1000, 1);
	this->declare_parameter<int>("I. minAmplitude0", nslConfig.minAmplitude[0], min_Amplitude0);
	auto min_Amplitude1 = create_Slider("minimum Amplitude 1", 0, 1000, 1);
	this->declare_parameter<int>("I. minAmplitude1", nslConfig.minAmplitude[1], min_Amplitude1);
	auto min_Amplitude2 = create_Slider("minimum Amplitude 2", 0, 1000, 1);
	this->declare_parameter<int>("I. minAmplitude2", nslConfig.minAmplitude[2], min_Amplitude2);
	auto min_Amplitude3 = create_Slider("minimum Amplitude 3", 0, 1000, 1);
	this->declare_parameter<int>("I. minAmplitude3", nslConfig.minAmplitude[3], min_Amplitude3);
	this->declare_parameter<string>("J. modIndex", modulationIntMap.at(static_cast<int>(nslConfig.mod_frequencyOpt)));
	auto channelOpt = create_Slider("Channel", 0, 15, 1);
	this->declare_parameter<int>("K. channel", static_cast<int>(nslConfig.mod_channelOpt), channelOpt);
	auto roi_LeftX = create_Slider("roi LeftX", 0, 159, 1);
	this->declare_parameter<int>("L. roi_leftX", nslConfig.roiXMin, roi_LeftX);
	auto roi_TopY = create_Slider("roi TopY", 0, 52, 1);
	this->declare_parameter<int>("M. roi_topY",  nslConfig.roiYMin, roi_TopY);
	auto roi_RightX = create_Slider("roi rightX", 0, 159, 1);
	this->declare_parameter<int>("N. roi_rightX", nslConfig.roiXMax, roi_RightX);
	auto roi_BottomY = create_Slider("roi bottomY", 0, 59, 1);
	this->declare_parameter<int>("O. roi_bottomY", nslConfig.roiYMax, roi_BottomY);

	auto transform_AngleV = create_Slider("Angle Vertical", -90.0, 90.0, 9.0);
	this->declare_parameter<double>("P. transformAngleV", viewerParam.lidarAngleV, transform_AngleV);
	auto transform_AngleH = create_Slider("Angle Horizontal", -90.0, 90.0, 9.0);
	this->declare_parameter<double>("P. transformAngleH", viewerParam.lidarAngleH, transform_AngleH);
	this->declare_parameter<string>("Q. frameID", viewerParam.frame_id);
	this->declare_parameter<bool>("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	this->declare_parameter<bool>("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	auto temporal_FactorValue = create_Slider("temporal FactorValue", 0.0, 1.0, 0.01);
	this->declare_parameter<double>("T. temporalFilterFactor", nslConfig.temporalFactorValue/1000.0, temporal_FactorValue);
	auto temporal_Threshold = create_Slider("temporal Threshold", 0, 1000, 1);
	this->declare_parameter<int>("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue, temporal_Threshold);
	auto edge_Threshold = create_Slider("edge Threshold", 0, 5000, 1);
	this->declare_parameter<int>("U. edgeFilterThreshold", nslConfig.edgeThresholdValue, edge_Threshold);
	auto interference_DetectionLimit = create_Slider("interference DetectionLimit", 0, 10000, 1);
	this->declare_parameter<int>("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue,interference_DetectionLimit);
	this->declare_parameter<bool>("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));
	auto pointCloud_EdgeThreshold = create_Slider("pointCloud EdgeThreshold", 0, 10000, 1);
	this->declare_parameter<int>("Y. PointColud EDGE", viewerParam.pointCloudEdgeThreshold, pointCloud_EdgeThreshold);
	auto max_Distance = create_Slider("max Distance", 0, 50000, 1);
	this->declare_parameter<int>("Z. MaxDistance", viewerParam.maxDistance, max_Distance);

	this->set_parameter(pDevName);
	this->set_parameter(pFrameID);
	this->set_parameter(pImageType);
	this->set_parameter(pHdr_mode);
	this->set_parameter(pInt0);
	this->set_parameter(pInt1);
	this->set_parameter(pInt2);
	this->set_parameter(pInt3);
	this->set_parameter(pIntGr);
	this->set_parameter(pMinAmplitude0);
	this->set_parameter(pMinAmplitude1);
	this->set_parameter(pMinAmplitude2);
	this->set_parameter(pMinAmplitude3);
	this->set_parameter(pModIndex);
	this->set_parameter(pChannel);
	this->set_parameter(pRoi_leftX);
	this->set_parameter(pRoi_topY);
	this->set_parameter(pRoi_rightX);
	this->set_parameter(pRoi_bottomY);
	this->set_parameter(pTransformAngleV);
	this->set_parameter(pTransformAngleH);
	this->set_parameter(pMedianFilter);
	this->set_parameter(pAverageFilter);
	this->set_parameter(pTemporalFilterFactor);
	this->set_parameter(pTemporalFilterThreshold);
	this->set_parameter(pEdgeFilterThreshold);
	this->set_parameter(pInterferenceDetectionLimit);
	this->set_parameter(pUseLastValue);
	this->set_parameter(pCvShow);
	this->set_parameter(pGrayscale);	
	this->set_parameter(pPCEdgeFilter);
	this->set_parameter(pMaxDistance);

	viewerParam.saveParam = false;
	reconfigure = false;
	
	RCLCPP_INFO(this->get_logger(),"end initialise()\n");
}


void roboscanPublisher::startStreaming()
{	
	if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
	}
	else{
		std::cout << "operation mode NONE~~~"<< std::endl;
	}
}

cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, NslPCD *ptNslPCD, int lidarWidth, int lidarHeight)
{
	int width = ptNslPCD->width;
	int height = ptNslPCD->height;
	int viewer_xpos = mouseXpos;
	int viewer_ypos = mouseYpos;
	float textSize = 0.8f;
	//int xMin = ptNslPCD->roiXMin;
	int yMin = ptNslPCD->roiYMin;
	int xpos = viewer_xpos/VIEWER_SCALE_SIZE;
	int ypos = viewer_ypos/VIEWER_SCALE_SIZE;

	if( (ypos >= yMin && ypos < lidarHeight)){
		
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		line(distMat, Point(viewer_xpos-10, viewer_ypos), Point(viewer_xpos+10, viewer_ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(viewer_xpos, viewer_ypos-10), Point(viewer_xpos, viewer_ypos+10), Scalar(255, 255, 0), 2);

		if( xpos >= lidarWidth ){ 
			xpos -= lidarWidth;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

		int distance2D = ptNslPCD->distance2D[ypos][xpos];
		if( distance2D > NSL_LIMIT_FOR_VALID_DATA ){

			if( distance2D == NSL_ADC_OVERFLOW )
				dist2D_caption = format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( distance2D == NSL_SATURATION )
				dist2D_caption = format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( distance2D == NSL_BAD_PIXEL )
				dist2D_caption = format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( distance2D == NSL_INTERFERENCE )
				dist2D_caption = format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( distance2D == NSL_EDGE_DETECTED )
				dist2D_caption = format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist2D_caption = format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( ptNslPCD->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ) {
				dist2D_caption = format("2D X:%d Y:%d %dmm/%dlsb", xpos, ypos, ptNslPCD->distance2D[ypos][xpos], ptNslPCD->amplitude[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", ptNslPCD->distance3D[OUT_X][ypos][xpos], ptNslPCD->distance3D[OUT_Y][ypos][xpos], ptNslPCD->distance3D[OUT_Z][ypos][xpos]);
			}
			else{
				dist2D_caption = format("2D X:%d Y:%d <%d>mm", xpos, ypos, ptNslPCD->distance2D[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", ptNslPCD->distance3D[OUT_X][ypos][xpos], ptNslPCD->distance3D[OUT_Y][ypos][xpos], ptNslPCD->distance3D[OUT_Z][ypos][xpos]);
			}
		}

		info_caption = format("%s:%dx%d %dfps %.2f'C", toString(ptNslPCD->operationMode), width, height, viewerParam.frameCount, ptNslPCD->temperature);

		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist2D_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, distMat);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d %dfps %.2f'C", toString(ptNslPCD->operationMode), width, height, viewerParam.frameCount, ptNslPCD->temperature);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, textSize, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}


void roboscanPublisher::setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color.b;
	image.at<Vec3b>(y,x)[1] = color.g;
	image.at<Vec3b>(y,x)[2] = color.r;
}

void roboscanPublisher::publishFrame(NslPCD *frame)
{
	static rclcpp::Clock s_rclcpp_clock;
	auto data_stamp = s_rclcpp_clock.now();

	cv::Mat distanceMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat amplitudeMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude

	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE )
	{
		sensor_msgs::msg::Image imgDistance;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->distance2D[y+yMin][x+xMin] & 0xFF));		 // LSB
				result.push_back(static_cast<uint8_t>((frame->distance2D[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(distanceMat, x+xMin, y+yMin, nsl_getDistanceColor(frame->distance2D[y+yMin][x+xMin]));
			}
		}

		imgDistance.header.stamp = data_stamp;
		imgDistance.header.frame_id = viewerParam.frame_id;
		imgDistance.height = static_cast<uint32_t>(frame->height);
		imgDistance.width = static_cast<uint32_t>(frame->width);
		imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
		imgDistance.step = imgDistance.width * 2;
		imgDistance.is_bigendian = 0;
		imgDistance.data = result;
		imgDistancePub->publish(imgDistance);
	}

	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE )
	{
		sensor_msgs::msg::Image imgAmpl;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgAmpl.header.stamp = data_stamp;
		imgAmpl.header.frame_id = viewerParam.frame_id;
		imgAmpl.height = static_cast<uint32_t>(frame->height);
		imgAmpl.width = static_cast<uint32_t>(frame->width);
		imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
		imgAmpl.step = imgAmpl.width * 2;
		imgAmpl.is_bigendian = 0;
		imgAmpl.data = result;
		imgAmplPub->publish(imgAmpl);
	}	

#ifdef image_transfer_function
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr->header.stamp = data_stamp;
	cv_ptr->header.frame_id = viewerParam.frame_id;
	cv_ptr->image = distanceMat;
	cv_ptr->encoding = "bgr8";

	imagePublisher.publish(cv_ptr->toImageMsg());		
#endif	

	const size_t nPixel = frame->width * frame->height;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	cloud->header.frame_id = viewerParam.frame_id;
	cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
	//cloud->header.stamp = static_cast<uint64_t>(data_stamp.nanoseconds());
	cloud->width = static_cast<uint32_t>(frame->width);
	cloud->height = static_cast<uint32_t>(frame->height);
	cloud->is_dense = false;
	cloud->points.resize(nPixel);

	int xMin = frame->roiXMin;
	int yMin = frame->roiYMin;

	for(int y = 0, index = 0; y < frame->height; y++)
	{
		for(int x = 0; x < frame->width; x++, index++)
		{
			pcl::PointXYZI &point = cloud->points[index];

			if( frame->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA )
			{
				point.x = (double)(frame->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
				point.y = (double)(-frame->distance3D[OUT_X][y+yMin][x+xMin]/1000);
				point.z = (double)(-frame->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
				point.intensity = frame->amplitude[y+yMin][x+xMin];
			}
			else{
				point.x = std::numeric_limits<float>::quiet_NaN();
				point.y = std::numeric_limits<float>::quiet_NaN();
				point.z = std::numeric_limits<float>::quiet_NaN();
				point.intensity = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}

	
	sensor_msgs::msg::PointCloud2 msg;
	pcl::toROSMsg(*cloud, msg);
	msg.header.stamp = data_stamp;
	msg.header.frame_id = viewerParam.frame_id;
	pointcloudPub->publish(msg);  
	
	if(viewerParam.cvShow == true)
	{	
		int distanceWidth = NSL_LIDAR_WIDTH*VIEWER_SCALE_SIZE;
		int distanceHeight = NSL_LIDAR_HEIGHT*VIEWER_SCALE_SIZE;

		getMouseEvent(mouseXpos, mouseYpos);
		
		if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
			distanceWidth *=2;
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
		}

		cv::resize( distanceMat, distanceMat, cv::Size( distanceWidth, distanceHeight ), 0, 0, INTER_LINEAR );

		distanceMat = addDistanceInfo(distanceMat, frame, NSL_LIDAR_WIDTH, NSL_LIDAR_HEIGHT);
		imshow(winName, distanceMat);
		waitKey(1);
	}
	
}


void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}

/*
	ubuntu usb device
	
	sudo apt-get install libopencv-dev
	sudo apt-get install libpcl-dev(1.8.1)

	$ sudo vi /etc/udev/rules.d/defined_lidar.rules
	KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", OWNER:="nanosystem", GROUP:="nanosystem", SYMLINK+="ttyNsl2206

	$ service udev reload
	$ service udev restart
*/

int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;
	
	rclcpp::init(argc, argv);

	auto node = std::make_shared<roboscanPublisher>();
	node->initialise();
	
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

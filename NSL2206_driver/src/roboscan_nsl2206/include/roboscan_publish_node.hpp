//#ifndef CAMERA635_DRIVER_H
//#define CAMERA635_DRIVER_H

#include "roboscan_nsl2206_image.h"
#include "communication_2206.h"
#include "camera_calibration.h"
#include "HdrHandler.h"

//#include <dynamic_reconfigure/server.h>
//#include <roboscan_nsl2206/roboscan_nsl2206h>

#include <cmath>
#include <unistd.h>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>


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

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
//#include "communication_2206.h"
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>
#include "epc_timer.h"
#include "roboscan_nsl2206_image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"

#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

//image_transport::Publisher imagePublisher;
#define WIN_NAME 					"LiDAR"
//#define  MAX_LEVELS  9
//#define NUM_COLORS     		30000
#define NUM_INTEGRATION_TIME_3D 	6




//#include <image_transport/image_transport.h>

using std::vector;
using namespace cv;

struct Settings{

  int imageType = 1;
  bool startStream = true;
  bool triggerSingleShot = false;
  bool runVideo= false;
  bool updateParam = false;
  int  mode = 0;
  int  integrationTimeATOF1 = 777;
  int  integrationTimeATOF2 = 0;
  int  integrationTimeATOF3 = 0;
  int  integrationTimeATOF4 = 0;
  int  integrationTimeBTOF1;
  int  integrationTimeBTOF2;
  int  integrationTimeGray;

  int modFrequency = 0;
  double kalmanFactor = 0.0;
  int kalmanThreshold = false;
  bool averageFilter = false;
  bool medianFilter = false;
  int minAmplitude1 = 50;
  int minAmplitude2 = 0;
  int minAmplitude3 = 0;
  int minAmplitude4 = 0;
  int minAmplitude5 = 0;
  int offsetDistance = 0;  
  double angle;
  bool calibrate;
  bool enableCartesian;
  bool enableUndistortion;
  bool enableImages;
  bool enablePointCloud;
  bool enableImageHeader;

  int roi_leftX = 1;
  int roi_topY = 1;
  int roi_rightX = 158;
  int roi_bottomY = 58;

  double maxDistance = 7500;
  double maxAmplitude = 2890;

  bool cvShow = true;
  int numHdr = 0;
  int numIntegrationTime = 0;
  int integrationTime3d[NUM_INTEGRATION_TIME_3D] = {0, };
  int integrationDistance[NUM_INTEGRATION_TIME_3D][160*60] = {0, };
  int integartionAmplitude[NUM_INTEGRATION_TIME_3D][160*60] = {0, };

  ComLib::Nsl2206Image::Nsl2206ImageType_e iType;

};



class roboscanPublisher : public rclcpp::Node 
{
		
public:
    roboscanPublisher();
    ~roboscanPublisher();

    int image_type = 1;
    int mode = 0; // 0 10MHz
    int integration_time_0 = 500;
    int integration_time_1 = 0;
    int integration_time_2 = 0;
    int integration_time_3 = 0;
    int integration_time_4 = 200;
    int integration_time_5 = 0;
    int integration_time_gray = 0;
    int mod_frequency = 0;
    double temporal_filter_threshold = 0;
    int temporal_filter_factor = 0;
    bool spatial_average_filter = false;
    int min_amplitude_0 = 50;
    int min_amplitude_1 = 0;
    int min_amplitude_2 = 0;
    int min_amplitude_3 = 0;
    int min_amplitude_4 = 0;
    int offset_distance = 0;
    int lens_angle_0 = 110;
    bool enable_cartesian = true;    
    bool enable_undistortion = false;
    bool enable_images = true;
    bool enable_point_cloud = true;
    bool enable_image_header = true;
    int roi_left_x = 0;
    int roi_top_y = 0;
    int roi_right_x = 159;
    int roi_bottom_y = 59;
    bool start_stream = false;
    bool trigger_single_shot = false;
	
	
	//

    //Settings settings;


    void update();    
    void initCommunication();
    void setAngle(double angle_);
    void updateCameraCalibration();
    
    void thread_callback();
    boost::scoped_ptr<boost::thread> publisherThread;
    bool runThread;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher1;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher2;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher3;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr imageHeaderPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorDistancePublisher;
	//rclcpp::image_transport::Publisher colorDistancePublisher;


private:
    //
    

    double koefX[60][160];
    double koefY[60][160];
    double koefZ[60][160];

    double angle;
    bool lastSingleShot;
    unsigned int frameSeq;
    Settings lidarParam;


	//lens_transform
	int distortionTableSize = 46;
	int numCols;
	int numRows;
	double sensorPointSizeMM = 0.02f;
	double lens_angleH;
	double lens_angle[101];
	double lens_rp[101];
	double lens_xUA[160][60];
	double lens_yUA[160][60];
	double lens_zUA[160][60];
/*
    const ros::Publisher &imagePublisher1;
    const ros::Publisher &imagePublisher2;
    const ros::Publisher &imageHeaderPublisher;
    const ros::Publisher &pointCloud2Publisher;
*/

    
	//image_transport::Publisher &imagePublisher;	

    int imageSize8;
    int imageSize16_1;
    int imageSize16_2;
	
	int distData[160*60] = {0, };
	int amplData[160*60] = {0, };
	int grayData[160*60] = {0, };
	
    std::string strFrameID;
    sensor_msgs::msg::Image img8;
	cv_bridge::CvImagePtr cv_ptr;
	
    sensor_msgs::msg::Image img16_1;
    sensor_msgs::msg::Image img16_2;

    sensor_msgs::msg::CameraInfo cameraInfo;

    CameraCalibration cameraCalibration;
    ComLib::Communication2206 communication;
    std_msgs::msg::Int32MultiArray imageHeaderMsg;
	HdrHandler2206 hdrHandler;

	vector<Vec3b> colorVector;


    void initialise();
    bool setCameraInfo(sensor_msgs::srv::SetCameraInfo::Request& req, sensor_msgs::srv::SetCameraInfo::Response& res);
    void parameterInit();
    //
	void updateData();
    void setParameters();
    void updateLensCalibrationData(std::vector<uint8_t> data);
    void publishImageHeader(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateCameraInfo(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateGrayscaleFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateDistanceFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateDistanceAmplitudeFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateDistanceGrayscaleFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void transformKoef(double angleGrad);
    void transformPixel(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width, double height, double angleGrad);
    void transformPixelOpt(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width2, double height2, double step);
    void transformPixelOpt1(int srcX, int srcY, double srcZ, double &destX, double &destY, double &destZ);
	void getDistanceColor(cv::Mat &imageLidar, int x, int y, int value);
	void getAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range);
	void getGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range);
	void createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
	double interpolate(double x, double x0, double y0, double x1, double y1);
	void initLensDistortionTable();
	double lensInterpolate(double x_in, double x0, double y0, double x1, double y1);
	double lensGetAngle(double x, double y, double sensorPointSizeMM);
	void lensTransformPixel(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle);
	void lensInitialisation(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY);
	
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);

	void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
	cv::Mat addDistanceInfo(cv::Mat distMat, int distData[], int width);
	int mouseXpos, mouseYpos;
	
};



//#endif // CAMERA635_DRIVER_H

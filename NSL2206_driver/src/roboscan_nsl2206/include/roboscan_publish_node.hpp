#include <cstdio>
#include <chrono>
#include <functional>
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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nanolib.h"

#define image_transfer_function

#ifdef image_transfer_function
#include <image_transport/image_transport.hpp>
#endif

namespace nanosys {

	struct ViewerParameter {
		int	frameCount;
		int maxDistance;
		int pointCloudEdgeThreshold;
		int imageType;
		double lidarAngleV;
		double lidarAngleH;
		
		bool cvShow;
		bool grayScale;
		bool changedCvShow;
		bool changedImageType;
		bool reOpenLidar;
		bool saveParam;
		
		std::string	frame_id;
		std::string	devName;
    };


	class roboscanPublisher : public rclcpp::Node { 

	public:
		roboscanPublisher();
		~roboscanPublisher();

		void initialise();
		void threadCallback();
		void setReconfigure();
		void publishFrame(NslPCD *frame);
		void startStreaming();

		//static rclcpp::Time timeNow;

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDistancePub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgAmplPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgGrayPub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;

#ifdef image_transfer_function
		rclcpp::Node::SharedPtr nodeHandle;
		image_transport::ImageTransport imageTransport;
		image_transport::Publisher imagePublisher;
#endif
		ViewerParameter viewerParam;

		boost::scoped_ptr<boost::thread> publisherThread;
		bool runThread;
		NslConfig 		nslConfig;
		int 			nsl_handle;
	private:
		std::string yaml_path_;
	
		// load yaml
		void load_params()
		{
			const std::unordered_map<std::string, int> modeMap = {
			    {"DISTANCE", 1},
				{"DISTANCE_AMPLITUDE", 2},
			};

			RCLCPP_INFO(this->get_logger(),"Loaded params: path=%s\n", yaml_path_.c_str());
			
			if (std::ifstream(yaml_path_))
			{
				YAML::Node config = YAML::LoadFile(yaml_path_);
				viewerParam.devName = config["devName"] ? config["devName"].as<std::string>() : "/dev/ttyNsl2206";
				viewerParam.frame_id = config["FrameID"] ? config["FrameID"].as<std::string>() : "roboscan_frame";
				viewerParam.maxDistance = config["MaxDistance"] ? config["MaxDistance"].as<int>() : 12500;
				viewerParam.pointCloudEdgeThreshold = config["PointColud EDGE"] ? config["PointColud EDGE"].as<int>() : 200;
				std::string tmpModeStr = config["ImageType"] ? config["ImageType"].as<std::string>() : "DISTANCE_AMPLITUDE";
				auto itMode = modeMap.find(tmpModeStr);
				viewerParam.imageType = (itMode != modeMap.end()) ? itMode->second : 3; // defeault DISTANCE_AMPLITUDE
				
				viewerParam.lidarAngleV = config["LidarAngleV"] ? config["LidarAngleV"].as<double>() : 0;
				viewerParam.lidarAngleH = config["LidarAngleH"] ? config["LidarAngleH"].as<double>() : 0;

				RCLCPP_INFO(this->get_logger(),"Loaded params: devName=%s, frame_id=%s, max = %d, edge = %d, imgType = %d, Angle = %.2f/%.2f\n", viewerParam.devName.c_str(), viewerParam.frame_id.c_str(), viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, viewerParam.imageType, viewerParam.lidarAngleV, viewerParam.lidarAngleH);
			}
			else{
				RCLCPP_INFO(this->get_logger(),"Not found params: devName=%s, frame_id=%s, max = %d, edge = %d, imgType = %d, Angle = %.2f/%.2f\n", viewerParam.devName.c_str(), viewerParam.frame_id.c_str(), viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, viewerParam.imageType, viewerParam.lidarAngleV, viewerParam.lidarAngleH);
			}
		}

	    // save yaml
	    void save_params()
	    {
		    const std::unordered_map<int, std::string> modeMap = {
		        {1, "DISTANCE"},
				{2, "DISTANCE_AMPLITUDE"},
		    };

			int imgType = this->get_parameter("C. imageType").as_int();
			if( imgType < 1 || imgType > 2 ) imgType = 2; // default DISTANCE_AMPLITUDE
			
	        std::ofstream fout(yaml_path_);
	        fout << "devName: " << this->get_parameter("0. devName").as_string() << "\n";
	        fout << "FrameID: " << this->get_parameter("Q. frameID").as_string() << "\n";
	        fout << "MaxDistance: " << this->get_parameter("Z. MaxDistance").as_int() << "\n";
	        fout << "PointColud EDGE: " << this->get_parameter("Y. PointColud EDGE").as_int() << "\n";
			fout << "ImageType: " << modeMap.at(imgType) << "\n";
	        fout << "LidarAngleV: " << this->get_parameter("P. transformAngleV").as_double() << "\n";
	        fout << "LidarAngleH: " << this->get_parameter("P. transformAngleH").as_double() << "\n";

	        fout.close();
	        RCLCPP_INFO(this->get_logger(), "Params saved to %s", yaml_path_.c_str());
	    }
		
		void initNslLibrary();
		void setMatrixColor(cv::Mat image, int x, int y, NslOption::NslVec3b color);
		void timeDelay(int milli);
		void renewParameter();
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, NslPCD *frame);
		void setWinName();
		void paramDump(const std::string & filename);
		void paramLoad();

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
		int mouseXpos, mouseYpos;
		bool reconfigure;
		char winName[100];
	};


} //end namespace nanosys

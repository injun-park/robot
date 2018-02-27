/*
Sample code to show how we can subscribe to the Kinect /camera/rgb/points topic containing PCL 2.0 data (with RGB)
and use the PCL library to:
1) display the data (pcl_visualization) - with RGB and just depth
2) limit data to a predefined 'z' (depth)
3) publish the filtered point cloud

Author: Ching L. Teo
Date: 05/30/2011
*/
// standard includes
#include<stdexcept>
// ROS core
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
// PCL includes
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl_visualization/pcl_visualizer.h>
//openCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
// cv_bridge
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace pcl;
using namespace pcl_visualization;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class ProcessKinectRange
{	

public:
	// constructor
	ProcessKinectRange(int argc, char** argv) : it(node_handle)
	{
		angular_resolution = 0.3f; // Resolution of the range image
		noise_level = 0.0f;         // Distance in which the z-buffer averages instead of taking the minimum
		min_range  = 0.0f;          // Minimum range for the range image creation
		border_size = 0;            // Additional border around the range image
		viz_type = 1;		    // 0 - depth+pcl, 1 - rgb+pcl, 2 - depth image only
		coordinate_frame = RangeImage::CAMERA_FRAME;
		viewer = new PCLVisualizer("Live viewer - point cloud");
		range_image = new RangeImage;
		sensor_pose = Eigen::Affine3f::Identity();

		// check command line inputs
		ProcessCommandLine(argc, argv);


		//check input topic
		if (node_handle.resolveName("input")=="/input")
		{	
    			std::cerr << "Did you forget input:=<your Kinect PCL topic>?\n";
			exit(0);
		}
		if (node_handle.resolveName("input2")=="/input2" && viz_type == 2)
		{	
    			std::cerr << "Did you forget input2:=<your Kinect Depth image topic>?\n";
			exit(0);
		}

		// set-up attributes + subscriber
		angular_resolution = deg2rad (angular_resolution);	
		subscriber = node_handle.subscribe ("input", 1, &ProcessKinectRange::cloud_msg_cb, this);
		subscriber_depth = it.subscribe ("input2", 1, &ProcessKinectRange::depth_msg_cb, this); // receive depth images directly from Kinect
		//set-up publishers -- a templated point cloud publisher (will convert from pcl::PointCloud to sensor_msgs::PointCloud2 automatically)
		publisher = node_handle.advertise <pcl::PointCloud<pcl::PointXYZRGB> >("/umd_image_proc/PointCloud2_umd_processKinectRange2", 1);
		publisher_img = it.advertise ("/umd_image_proc/image_umd_processKinectRange2", 1);
	}

	// destructor
	~ProcessKinectRange()
	{
	}
	
	// call backs for subscriber
	void cloud_msg_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
	{
	  m.lock ();
	  cloud_msg = msg;
	  //std::cout << "Received cloud msg: " << "header=" << msg->header << "width="<<msg->width <<", height="<<msg->height<<".\n";
	  m.unlock ();
	 
	// if point clouds are to be processed, else skip
	 if(viz_type!=2)
	  	ProcessCloudMessage();

	}
	void depth_msg_cb (const sensor_msgs::ImageConstPtr& msg)
	{
	  m.lock ();
	  kinect_depthimg_msg = msg;
	  m.unlock ();

	  if(viz_type == 2)
	  	ProcessDepthMessage();

	}

private:
	// ros attributes
	ros::NodeHandle node_handle;
  	ros::Subscriber subscriber;
	ros::Publisher	publisher;
	image_transport::ImageTransport it; // should always use ImageTransport to subscribe and publish images
	image_transport::Subscriber subscriber_depth;
  	image_transport::Publisher publisher_img;
	PCLVisualizer* viewer;
	RangeImage* range_image;

	// other attributes
	float angular_resolution; 	// Resolution of the range image
	float	noise_level;         	// Distance in which the z-buffer averages instead of taking the minimum
	float	min_range;          	// Minimum range for the range image creation
	int	border_size;            // Additional border around the range image
	int 	viz_type;		// the type of points expected: 0 - no rgb, 1 - otherwise
	RangeImage::CoordinateFrame coordinate_frame;
	sensor_msgs::PointCloud2ConstPtr cloud_msg, old_cloud_msg;
	sensor_msgs::ImageConstPtr kinect_depthimg_msg, old_kinect_depthimg_msg;
	boost::mutex m;
	Eigen::Affine3f sensor_pose;

	// Main internal function to process the point cloud message
	void ProcessCloudMessage()
	{

		usleep (10000);
		viewer->spinOnce (10);
		// if no new messages appear, just stop processing
		if (!cloud_msg || cloud_msg == old_cloud_msg)
			return;
		old_cloud_msg = cloud_msg;
		
		// get message from ROS and convert to point cloud
		PointCloud<PointXYZRGB> point_cloud;
		fromROSMsg(*cloud_msg, point_cloud);

		// if the sensor point cloud provides "far range data", use it to compute the sensor_pose	
		PointCloud<PointWithViewpoint> far_ranges;
		RangeImage::extractFarRanges(*cloud_msg, far_ranges);
		if (pcl::getFieldIndex(*cloud_msg, "vp_x")>=0)
		{
			PointCloud<PointWithViewpoint> tmp_pc;
			fromROSMsg(*cloud_msg, tmp_pc);
			Eigen::Vector3f average_viewpoint = RangeImage::getAverageViewPoint(tmp_pc);
			sensor_pose = Eigen::Translation3f(average_viewpoint) * sensor_pose;
		}
		
		//ROS_INFO("(h,w)=%d,%d.\n", point_cloud.height, point_cloud.width);     
		
		// For efficieny, all functions in PCL work with PointCloud<PointT>::Ptr, so we extract them from the real point cloud
		PointCloud<PointXYZRGB>::Ptr point_cloud_ptr (new PointCloud<PointXYZRGB>(point_cloud));
		PointCloud<PointXYZRGB>::Ptr point_cloudfilt_ptr (new PointCloud<PointXYZRGB>); // filtered pc

		// Filter clouds in Z dimension (min, max)
		FilterPointCloudZ(point_cloud_ptr, point_cloudfilt_ptr, 0.0f, 10.0f);

		//ROS_INFO("(pco_h, pco_w)=%d,%d.\n", (*point_cloud_ptr).height, (*point_cloud_ptr).width);     
		//ROS_INFO("(h,w)=%d,%d.\n", (*point_cloudfilt_ptr).height, (*point_cloudfilt_ptr).width);     
				
		// visualize
		VisualizePointCloud(point_cloudfilt_ptr); 
		//VisualizePointCloud(point_cloud_ptr); 

		//publish filtered point cloud
		publisher.publish((*point_cloudfilt_ptr));	
/*//OLD CODE
		// publish range_image
		if(viz_type==0)
		{

			//ROS_INFO("(rimg_h, rimg_w)=%d,%d.\n", range_image->width , range_image->height);  
			double rval_tmp;   			
			IplImage *tmp = cvCreateImage(cvSize(range_image->width , range_image->height), IPL_DEPTH_32F, 1); 
			IplImage *t2 = cvCreateImage(cvSize(range_image->width , range_image->height), IPL_DEPTH_8U, 1); // for visualization only

			// Get iplImages from RangeImage
			// NOTE (06/08/11): Kinect Produces depth images directly...see active_seg_2D code
			cvSetZero(tmp); 
			cvSetZero(t2);

			for(int p_x = 0; p_x < tmp->width; p_x++)
			  for(int p_y = 0; p_y < tmp->height; p_y++)
			  {
			    //tmp->imageData[p_y*tmp->widthStep + p_x] = (range_image->getPoint(p_x, p_y).range > 0.0 ? 255 : 0);
			    rval_tmp = range_image->getPoint(p_x, p_y).range;
			    //ROS_INFO("rval: %g", rval_tmp);
			    if(rval_tmp > 0.0f){
				// ROS_INFO("here");
			    	cvSet2D(tmp, p_y, p_x, cvScalar(rval_tmp));
				if(rval_tmp > 3.0f)
					t2->imageData[p_y*t2->widthStep + p_x] = 255;
				else if(rval_tmp > 2.0f)
					t2->imageData[p_y*t2->widthStep + p_x] = 200;
				else if(rval_tmp > 1.0f)
					t2->imageData[p_y*t2->widthStep + p_x] = 150;
				else if(rval_tmp > 0.5f)
					t2->imageData[p_y*t2->widthStep + p_x] = 100;
				else
					t2->imageData[p_y*t2->widthStep + p_x] = 50;
			    }
			    else{
				cvSet2D(tmp, p_y, p_x, cvScalar(0.0)); 
				t2->imageData[p_y*t2->widthStep + p_x] = 0;
			    	//ROS_INFO("r_val=%g\n",rval_tmp);
			    }
			  }

			//DEBUG
			double minval, maxval;
			cvMinMaxLoc(tmp, &minval, &maxval);
			ROS_INFO("min: %g, max: %g", minval, maxval);

			// visualize
			cv::namedWindow("range image");
			cv::imshow("range image", t2);
			cv::waitKey(3);
			
			// convert to cv::Mat
			cv::Mat rangeMat(tmp);		

			// copy and publish
			cv_bridge::CvImage cvres_out;
			cvres_out.header = cloud_msg->header;
			//cout<< cvres_out.header;
			cvres_out.encoding = enc::TYPE_32FC1;
			cvres_out.image = rangeMat;
			publisher_img.publish(cvres_out.toImageMsg());

			// release memory
			cvReleaseImage(&tmp);
			cvReleaseImage(&t2);
		}
*/
	 		
	}

	// Main function to process kinect depth messages
	void ProcessDepthMessage()
	{
		if (!kinect_depthimg_msg || kinect_depthimg_msg == old_kinect_depthimg_msg)
			return;
		old_kinect_depthimg_msg = kinect_depthimg_msg;

		// convert message from ROS to openCV
		cv_bridge::CvImagePtr cv_depthptr;
		try
		{
			cv_depthptr = cv_bridge::toCvCopy(kinect_depthimg_msg, enc::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// convert to IplImage
		IplImage ipl_depth = cv_depthptr->image;
	/*	
                //clean up the depth image - no negatives or NAN
		IplImage* ipl_depth_clean = cvCreateImage(cvGetSize(&ipl_depth), IPL_DEPTH_32F,1);
		double rval_tmp;
		for(int p_x = 0; p_x < ipl_depth.width; p_x++)
		  for(int p_y = 0; p_y < ipl_depth.height; p_y++)
		  {
		    rval_tmp = cvGet2D(&ipl_depth, p_y,p_x).val[0];
		    if(rval_tmp > 0.0f){
		    	cvSet2D(ipl_depth_clean, p_y, p_x, cvScalar(rval_tmp));
		    }
		    else{
			cvSet2D(ipl_depth_clean, p_y, p_x, cvScalar(0.0)); 
		    }
		  }
	*/


		 // filter depth image - no negatives or NAN
		cv::Mat depth_clean(cv_depthptr->image.rows, cv_depthptr->image.cols, CV_32FC1);
		cv::Mat img(cv_depthptr->image.rows, cv_depthptr->image.cols, CV_8UC1);
		for(int i = 0; i < cv_depthptr->image.rows; i++)
		{
			float* Di = cv_depthptr->image.ptr<float>(i);
			float* Ii = depth_clean.ptr<float>(i);
			char* Ivi = img.ptr<char>(i);
				for(int j = 0; j < cv_depthptr->image.cols; j++)
				{   
				   if(Di[j] > 0.0f){				    	
					Ii[j] = Di[j];
					Ivi[j] = (char) (255*((Di[j])/(5.5))); // some suitable values..
				   }
				    else{
					Ii[j] = 0.0f;
					Ivi[j] = 0;
				    }
				}   
		}

		// copy and publish depth image
		cv_bridge::CvImage cvres_out;
		cvres_out.header = cv_depthptr->header;
		//cout<< cvres_out.header;
		cvres_out.encoding = enc::TYPE_32FC1;
		cvres_out.image = depth_clean;
		publisher_img.publish(cvres_out.toImageMsg());

		// display
		cv::imshow("range image", img);
		cv::waitKey(3);

		
	}

	// Filter Point Cloud
	void FilterPointCloudZ(PointCloud<PointXYZRGB>::Ptr& pc, PointCloud<PointXYZRGB>::Ptr& pc_f, float minZ, float maxZ)
	{
		PassThrough<PointXYZRGB> filt;
		filt.setInputCloud(pc);
		filt.setFilterFieldName("z"); // filter z dimension
		filt.setFilterLimits(minZ, maxZ);
		filt.setKeepOrganized(true); // Important: to keep the "image-structure" after filtering, if not it becomes 1D (and sparse)
		filt.filter(*pc_f);		
	}

	// Visualize Point Clouds
	void VisualizePointCloud(PointCloud<PointXYZRGB>::Ptr& pc)
	{
		viewer->removePointCloud ("range image cloud");
		
		if(viz_type == 0)
		{
			
			range_image->createFromPointCloud(*pc, angular_resolution, deg2rad(360.0f), deg2rad(180.0f),
		                                sensor_pose, coordinate_frame, noise_level, min_range, border_size);
				
			// display DEPTH point cloud
		    	pcl_visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> color_handler_cloud(*range_image,200, 200, 200);
			viewer->addPointCloud (*range_image, color_handler_cloud, "range image cloud");

		}
		else
		{

			// display RGB point cloud
			pcl_visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler_cloud(*pc); 
		    	viewer->addPointCloud (*pc, color_handler_cloud, "range image cloud");

			// get a sample data point from the point cloud
			PointXYZRGB tpoint;			
			tpoint = pc->at(200,200); // sample image point
			std::vector<uint8_t> rgb_t(4); // hold 4 elements 
			extract_pcl_rgb(tpoint, rgb_t); 
			// x, y, z in meters. x right, y down, z forward from camera center, r, g, b:(1 - 255)
			ROS_INFO("(x,y,z)=%g,%g,%g, (r,g,b)=%d,%d,%d", tpoint.x, tpoint.y, tpoint.z, rgb_t[0],rgb_t[1],rgb_t[2]); 
			

		}	
	}
	
	// function to extract rgb stored in pcl PointXYZRGB
	void extract_pcl_rgb(pcl::PointXYZRGB& point_t, std::vector<uint8_t>& rgb_v)
	{
		// extract color values
		uint32_t rgb_val_;
		memcpy(&rgb_val_, &(point_t.rgb), sizeof(float));

		uint8_t garbage_ = (uint8_t)((rgb_val_ >> 24) & 0x000000ff);
		uint8_t r_ = (uint8_t)((rgb_val_ >> 16) & 0x000000ff);
		uint8_t g_ = (uint8_t)((rgb_val_ >> 8) & 0x000000ff);
		uint8_t b_ = (uint8_t)((rgb_val_) & 0x000000ff);

		// save to rgb vector
		rgb_v[0]=r_; 
		rgb_v[1]=g_; 
		rgb_v[2]=b_; 
		rgb_v[3]= garbage_;

	}

	void ProcessCommandLine(int argc, char** argv)
	{
		 for (char c; (c = getopt (argc, argv, "t:hc:r:")) != -1; ) {
		    switch (c) {
		      case 'c':
			coordinate_frame = RangeImage::CoordinateFrame (strtol (optarg, NULL, 0));
			break;
		      case 'r':
		      {
			angular_resolution = strtod (optarg, NULL);
			cout << PVARN(angular_resolution);
			break;
		      }
		      case 't':
		      {
			viz_type = strtol (optarg, NULL, 0);
			if (viz_type < 0  ||  viz_type > 2)
			{
			  cout << "VizType "<<viz_type<<" is unknown.\n";
			  exit (0);
			}
			if (viz_type != 2)
				cout << "Receiving "<<(viz_type==0 ? "depth point clouds" : "depth+RGB point clouds")<<".\n";
                        else
			 	cout << "Processing Depth image only.\n";
			break;
		      }
		      case 'h':
		      default:
			printUsage (argv[0]);
			exit (0);
		    }
		  }
	}	

	void printUsage (const char* progName)
	{
	  cout << "\n\nUsage: "<<progName<<" [options] input:=<yourInput>\n\n"
	       << "Options:\n"
	       << "-------------------------------------------\n"
	       << "-c <int>     source coordinate frame (default "<<coordinate_frame<<")\n"
	       << "-t           0 - 3D points, 1 - 3D+RGB points (default), 2 - depth image only.\n"
	       << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
	       << "-h           this help\n"
	       << "\n\n";
	}
};

int main(int argc, char** argv)
{

  ros::init (argc, argv, "umd_processKinectRange2");
  ProcessKinectRange kc(argc, argv);
  ros::spin();
  return 0;
  
}

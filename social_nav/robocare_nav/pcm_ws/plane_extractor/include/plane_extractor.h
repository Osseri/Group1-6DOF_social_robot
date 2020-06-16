
#include "nui_tracker.h"
#include "tool_class.h"

//Ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <pcm_msgs/PlaneMultiArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

//PCL
#include <pcl_tools.h>

class plane_extractor : public  Listener
{
public :
	plane_extractor();
	virtual ~plane_extractor();

private :
	pcl_tools pcl;
	eun_u::tool_class mat_cal;

	//Point
	string m_frame_id;
	eun_u::Bounds3D m_zone;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;

	//Ros
	ros::Publisher m_pub_display;
	ros::Publisher m_pub_plane;
	ros::Publisher m_pub_laser;

	//Callback
	nui_tracker m_nui_tracker;
	void get_depth_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
	void get_skeleton(vector<UserData> user_data);

	//pub_plane
	void pub_plane(vector<eun_u::Plane3D> input);

	//Astra Laser
	void conversion_to_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double* output);
	void pub_laser(double* input);

	//Display
	void point_display(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
	void object_display(vector<eun_u::Object3D> input);
	void plane_display(vector<eun_u::Plane3D> input);
	void vector_display(vector<eun_u::Plane3D> input);
};



#include <plane_extractor.h>

plane_extractor::plane_extractor()
{
	//Point
	ros::NodeHandle nodeHandle;
	nodeHandle.param<std::string>("plane_extractor/frame_id", m_frame_id, "camera");
	nodeHandle.param("plane_extractor/min_x", m_zone.min_x, 0.5);
	nodeHandle.param("plane_extractor/max_x", m_zone.max_x, 1.0);
	nodeHandle.param("plane_extractor/min_y", m_zone.min_y, -1.0);
	nodeHandle.param("plane_extractor/max_y", m_zone.max_y, 1.0);
	nodeHandle.param("plane_extractor/min_z", m_zone.min_z, -0.5);
	nodeHandle.param("plane_extractor/max_z", m_zone.max_z, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	this->m_cloud = cloud;
	this->m_cloud->points.resize(640 * 480);
	this->m_cloud->width = (640 * 480);
	this->m_cloud->height = 1;

	//ros
	this->m_pub_display = nodeHandle.advertise<visualization_msgs::Marker>("/plane_extractor/plane_display", 10);
	this->m_pub_plane = nodeHandle.advertise<pcm_msgs::PlaneMultiArray>("/plane_extractor/plane", 10);
	this->m_pub_laser = nodeHandle.advertise<sensor_msgs::LaserScan>("/astra/scan", 10);

	//CallBack
	this->m_nui_tracker.set_nui_Listener((Listener*) this);
}

plane_extractor::~plane_extractor()
{

}

//CallBack
void plane_extractor::get_depth_xyz(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
	//Point Arrange
	int point_count = 0;
	m_cloud->resize(points->size());
	m_cloud->width = points->size();
	m_cloud->height = 1;

	for(int i = 0; i < (int) points->size(); i++)
	{
		if(m_zone.min_x < points->points[i].x && m_zone.max_x > points->points[i].x &&
				m_zone.min_y < points->points[i].y && m_zone.max_y > points->points[i].y &&
				m_zone.min_z < points->points[i].z && m_zone.max_z > points->points[i].z)
		{
			m_cloud->points[point_count].x = points->points[i].x;
			m_cloud->points[point_count].y = points->points[i].y;
			m_cloud->points[point_count].z = points->points[i].z;
			point_count++;
		}
	}

	if(point_count > 10)
	{
		m_cloud->points.resize(point_count);
		m_cloud->width = point_count;
		m_cloud->height = 1;

		//Point Filtering
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
		cloud_filtered = pcl.point_filtering(m_cloud);
		//for(int i = 0; i < (int) cloud_filtered->points.size(); i++) printf("Point[%d] X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", i, cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z);

		//Astra Laser
		double distance[270];
		conversion_to_laser(cloud_filtered, distance);
		pub_laser(distance);

		//Object Segmentation
		vector<eun_u::Object3D> object;
		//object = pcl.plane_extract(cloud_filtered, 2);
		//object = pcl.euclidean_segmentation(cloud_filtered);
		object = pcl.rg_segmentation(cloud_filtered);
		//printf("Num Object : %d\n", (int) object.size());
		//for(int i = 0; i < (int) object.size(); i++) printf("Object[%d] : %d\n", i, (int) object[i].points.size());

		//Define Object
		if((int) object.size() > 0)
		{
			vector<eun_u::Plane3D> plane;
			for(int i = 0; i < (int) object.size(); i++)
			{
				eun_u::Plane3D temp_plane;
				temp_plane = mat_cal.cal_plane_data(object[i]);
				if(temp_plane.width * temp_plane.height > 0.05) plane.push_back(temp_plane);
			}

			if(m_pub_plane.getNumSubscribers() > 0) pub_plane(plane);
			if(m_pub_display.getNumSubscribers() > 0)	point_display(cloud_filtered);
			if(m_pub_display.getNumSubscribers() > 0)	object_display(object);
			if(m_pub_display.getNumSubscribers() > 0)	plane_display(plane);
			if(m_pub_display.getNumSubscribers() > 0)	vector_display(plane);

			if((int) plane.size() > 0) plane.clear();
		}
		if((int) object.size() > 0) object.clear();
	}

	printf("\n");
}

void plane_extractor::get_skeleton(vector<UserData> user_data)
{

}

//pub_plane
void plane_extractor::pub_plane(vector<eun_u::Plane3D> input)
{
	if((int) input.size() > 0)
	{
		pcm_msgs::Point point;
		pcm_msgs::PlaneMultiArray msg;
		for(int i = 0; i < (int) input.size(); i++)
		{
			pcm_msgs::Plane plane;
			plane.frame_id = m_frame_id;
			plane.width = input[i].width;
			plane.height = input[i].height;

			plane.angle_x = input[i].angle_x;
			plane.angle_y = input[i].angle_y;
			plane.angle_z = input[i].angle_z;

			plane.equ_a = input[i].equ_a;
			plane.equ_b = input[i].equ_b;
			plane.equ_c = input[i].equ_c;
			plane.equ_d = input[i].equ_d;

			plane.mean_center.x = input[i].mean_center.x;
			plane.mean_center.y = input[i].mean_center.y;
			plane.mean_center.z = input[i].mean_center.z;

			plane.median_center.x = input[i].median_center.x;
			plane.median_center.y = input[i].median_center.y;
			plane.median_center.z = input[i].median_center.z;

			plane.size_center.x = input[i].size_center.x;
			plane.size_center.y = input[i].size_center.y;
			plane.size_center.z = input[i].size_center.z;

			plane.vec_x.X = input[i].vec_x.X;
			plane.vec_x.Y = input[i].vec_x.Y;
			plane.vec_x.Z = input[i].vec_x.Z;

			plane.vec_y.X = input[i].vec_y.X;
			plane.vec_y.Y = input[i].vec_y.Y;
			plane.vec_y.Z = input[i].vec_y.Z;

			plane.vec_z.X = input[i].vec_z.X;
			plane.vec_z.Y = input[i].vec_z.Y;
			plane.vec_z.Z = input[i].vec_z.Z;


			for(int j = 0; j < 8; j++)
			{
				point.x = input[i].octagon_poins[j].x;
				point.y = input[i].octagon_poins[j].y;
				point.z = input[i].octagon_poins[j].z;
				plane.octagon_points.push_back(point);
				//printf("[%d] X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", i, point.x, point.y, point.z);
			}

			msg.plane.push_back(plane);
		}

		m_pub_plane.publish(msg);
	}
}

//Astra Laser
void plane_extractor::conversion_to_laser(pcl::PointCloud<pcl::PointXYZ>::Ptr input, double* output)
{
	int angle;
	double distance;
	eun_u::Vector3D vec;
	eun_u::Point2D point;
	memset(output, 0, sizeof(double) * 270);

	for(int i = 0; i < (int) input->points.size(); i++)
	{
		if(m_zone.min_x < input->points[i].x && m_zone.max_x > input->points[i].x &&
				m_zone.min_y < input->points[i].y && m_zone.max_y > input->points[i].y &&
				m_zone.min_z < input->points[i].z && m_zone.max_z > input->points[i].z)
		{
			point.x = vec.X = input->points[i].x;
			point.y = vec.Y = input->points[i].y;
			vec.Z = 0;
			angle = (mat_cal.angle_BetweenAxisXandAvector(vec) * 3);

			if(vec.Y < 0) angle = (int) (-fabs(angle) + 135);
			else angle = (int) (fabs(angle) + 135);
			distance = mat_cal.distance_BetweenZeroPointAndPoint2D(point);

			if(angle > 0 && angle < 270 && distance != 0)
			{
				if(distance < output[angle] || output[angle] == 0)	output[angle] = distance;
			}
		}
	}
}

void plane_extractor::pub_laser(double* input)
{
	sensor_msgs::LaserScan out_put;
	ros::Time time = ros::Time::now();

	out_put.header.stamp = time;
	out_put.header.frame_id = m_frame_id;
	out_put.angle_min = -45 * DEGREE_TO_RADIAN;
	out_put.angle_max = 45 * DEGREE_TO_RADIAN;
	out_put.angle_increment = DEGREE_TO_RADIAN * 0.3333333333333;
	out_put.time_increment = 0.00005;
	out_put.scan_time = ros::Time::now().toSec();
	out_put.range_min = 0.01;
	out_put.range_max = 10.0;

	for(int i = 0; i < 270; i++)
	{
		out_put.ranges.push_back(input[i]);
	}

	m_pub_laser.publish(out_put);
}

//Display
void plane_extractor::point_display(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	visualization_msgs::Marker points;
	points.header.frame_id = m_frame_id;
	points.header.stamp = ros::Time::now();
	points.ns = "Input Points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.type = visualization_msgs::Marker::POINTS;

	points.scale.x = 0.01;
	points.scale.y = 0.01;
	points.scale.z = 0.01;

	points.id = 0;

	points.color.r = 217.0/255.0f;
	points.color.g = 65.0/255.0f;
	points.color.b = 197.0/255.0f;
	points.color.a = 1.0;

	if((int) input->points.size() > 0)
	{
		geometry_msgs::Point p;
		for(int i = 0; i < (int) input->points.size(); i++)
		{
			if(input->points[i].x != 0 && input->points[i].y != 0 && input->points[i].z != 0)
			{
				p.x = (float) input->points[i].x;
				p.y = (float) input->points[i].y;
				p.z = (float) input->points[i].z;
				points.points.push_back(p);
			}
		}

		m_pub_display.publish(points);
	}
}

void plane_extractor::object_display(vector<eun_u::Object3D> input)
{
	visualization_msgs::Marker points0, points1, points2;
	points0.header.frame_id = m_frame_id;
	points0.header.stamp = ros::Time::now();
	points0.ns = "Object";
	points0.action = visualization_msgs::Marker::ADD;
	points0.pose.orientation.w = 1.0;
	points0.type = visualization_msgs::Marker::POINTS;

	points0.scale.x = 0.01;
	points0.scale.y = 0.01;
	points0.scale.z = 0.01;

	points1 = points2 = points0;

	points0.id = 1;
	points1.id = 2;
	points2.id = 3;

	points0.color.r = 1.0f;
	points0.color.g = 0.0f;
	points0.color.b = 0.0f;

	points1.color.r = 0.0f;
	points1.color.g = 1.0f;
	points1.color.b = 0.0f;

	points2.color.r = 0.0f;
	points2.color.g = 0.0f;
	points2.color.b = 1.0f;

	points0.color.a = points1.color.a = points2.color.a = 1.0;

	if((int) input.size() > 0)
	{
		geometry_msgs::Point p;
		for(int i = 0; i < (int) input.size(); i++)
		{
			// POINTS markers use x and y scale for width/height respectively
			for(int j = 0; j < (int) input[i].points.size(); j++)
			{
				p.x = (float) input[i].points[j].x;
				p.y = (float) input[i].points[j].y;
				p.z = (float) input[i].points[j].z;

				if(i%3 == 0) points0.points.push_back(p);
				else if(i%3 == 1) points1.points.push_back(p);
				else points2.points.push_back(p);
			}
		}

		m_pub_display.publish(points0);
		m_pub_display.publish(points1);
		m_pub_display.publish(points2);
	}
}

void plane_extractor::plane_display(vector<eun_u::Plane3D> input)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = m_frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "plane";
	marker.id = 100;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 217.0/255.0f;
	marker.color.g = 65.0/255.0f;
	marker.color.b = 197.0/255.0f;

	geometry_msgs::Point p[8];
	geometry_msgs::Point center;

	for(int i = 0; i < (int) input.size(); i++)
	{
		center.x = input[i].size_center.x;
		center.y = input[i].size_center.y;
		center.z = input[i].size_center.z;

		for(int j = 0; j < 8; j++)
		{
			p[j].x = input[i].octagon_poins[j].x;
			p[j].y = input[i].octagon_poins[j].y;
			p[j].z = input[i].octagon_poins[j].z;
		}

		for(int j = 0; j < 7; j++)
		{
			if(p[j].x != 0.0 && p[j].y != 0.0 && p[j].z != 0.0 && p[j+1].x != 0.0 && p[j+1].y != 0.0 && p[j+1].z != 0.0)
			{
				marker.points.push_back(p[j]);
				marker.points.push_back(p[j+1]);
				marker.points.push_back(center);
			}
		}

		if(p[7].x != 0.0 && p[7].y != 0.0 && p[7].z != 0.0 && p[0].x != 0.0 && p[0].y != 0.0 && p[0].z != 0.0)
		{
			marker.points.push_back(p[7]);
			marker.points.push_back(p[0]);
			marker.points.push_back(center);
		}
	}

	m_pub_display.publish(marker);
}

void plane_extractor::vector_display(vector<eun_u::Plane3D> input)
{
	visualization_msgs::Marker vec_x, vec_y, vec_z;
	vec_x.header.frame_id = vec_y.header.frame_id = vec_z.header.frame_id = m_frame_id;
	vec_x.header.stamp = vec_y.header.stamp = vec_z.header.stamp =ros::Time();
	vec_x.ns = vec_y.ns = vec_z.ns ="vector";
	vec_x.type = vec_y.type = vec_z.type = visualization_msgs::Marker::LINE_LIST;
	vec_x.action = vec_y.action = vec_z.action = visualization_msgs::Marker::ADD;

	vec_x.id = 50;
	vec_x.color.a = 1.0; // Don't forget to set the alpha!
	vec_x.color.r = 1.0;
	vec_x.color.g = 0.0;
	vec_x.color.b = 0.0;
	vec_x.scale.x = 0.02;

	vec_y.id = 51;
	vec_y.color.a = 1.0; // Don't forget to set the alpha!
	vec_y.color.r = 0.0;
	vec_y.color.g = 1.0;
	vec_y.color.b = 0.0;
	vec_y.scale.x = 0.02;

	vec_z.id = 52;
	vec_z.color.a = 1.0; // Don't forget to set the alpha!
	vec_z.color.r = 0.0;
	vec_z.color.g = 0.0;
	vec_z.color.b = 1.0;
	vec_z.scale.x = 0.02;

	for(int i = 0; i < (int) input.size(); i++)
	{
		geometry_msgs::Point mean, end;
		mean.x = input[i].mean_center.x;
		mean.y = input[i].mean_center.y;
		mean.z = input[i].mean_center.z;
		end.x = input[i].mean_center.x + input[i].vec_x.X * 0.3;
		end.y = input[i].mean_center.y + input[i].vec_x.Y * 0.3;
		end.z = input[i].mean_center.z + input[i].vec_x.Z * 0.3;
		vec_x.points.push_back(mean);
		vec_x.points.push_back(end);


		end.x = input[i].mean_center.x + input[i].vec_y.X * 0.3;
		end.y = input[i].mean_center.y + input[i].vec_y.Y * 0.3;
		end.z = input[i].mean_center.z + input[i].vec_y.Z * 0.3;
		vec_y.points.push_back(mean);
		vec_y.points.push_back(end);

		end.x = input[i].mean_center.x + input[i].vec_z.X * 0.3;
		end.y = input[i].mean_center.y + input[i].vec_z.Y * 0.3;
		end.z = input[i].mean_center.z + input[i].vec_z.Z * 0.3;
		vec_z.points.push_back(mean);
		vec_z.points.push_back(end);
	}

	m_pub_display.publish(vec_x);
	ros::Duration(0.01).sleep();
	m_pub_display.publish(vec_y);
	ros::Duration(0.01).sleep();
	m_pub_display.publish(vec_z);

}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "plane_extractor");

	plane_extractor GGG;

	ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	spinner.spin();

	return (0);
}


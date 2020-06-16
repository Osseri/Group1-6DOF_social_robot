//EUCLIDEAN_SEGMENTATION V1.05(Plane_Detection ADD)

#include <pcl_tools.h>
pcl_tools::pcl_tools()
{
	//KdTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	m_Kdtree = tree;

	//Normals
	m_normal_estimator.setSearchMethod (m_Kdtree);

	//Point Filtering
	pcl::VoxelGrid<pcl::PointXYZ> m_vg;

	//Segmentation Plane
	m_seg_plane.setOptimizeCoefficients (true);
	m_seg_plane.setModelType (pcl::SACMODEL_PLANE);
	m_seg_plane.setMethodType (pcl::SAC_RANSAC);
	m_seg_plane.setMaxIterations (100);
	m_seg_plane.setDistanceThreshold (0.02);

	//Extract Object by EuclideanCluster
	m_ec.setClusterTolerance (0.3);
	m_ec.setMinClusterSize (10);
	m_ec.setMaxClusterSize (50000);
	m_ec.setSearchMethod (m_Kdtree);

	//Normals Estimator
	m_normal_estimator.setKSearch (50);	//default 50

	//Extract Object by RegionGrowing
	m_reg.setMinClusterSize (5); //default 50
	m_reg.setMaxClusterSize (1000000);
	m_reg.setSearchMethod (m_Kdtree);
	m_reg.setNumberOfNeighbours (30);	//default 30
	m_reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);	//default (3.0 / 180.0 * M_PI)
	m_reg.setCurvatureThreshold (1.0);	//defalut (1.0)
}

pcl_tools::~pcl_tools()
{

}

//Point Filtering
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tools::point_filtering(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	m_vg.setInputCloud (input);
	m_vg.setLeafSize (POINT_FILTER_LEAF_SIZE, POINT_FILTER_LEAF_SIZE, POINT_FILTER_LEAF_SIZE);
	m_vg.filter (*return_cloud);

	return return_cloud;
}

//Point sorting YZ plane, y : width, z : height
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tools::point_sortingYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr input, eun_u::Bounds3D range)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	return_cloud->width = (int) (range.max_y - range.min_y)/SORTING_SIZE;
	return_cloud->height = (int) (range.max_z - range.min_z)/SORTING_SIZE;
	return_cloud->points.resize(return_cloud->width * return_cloud->height);

	int w_index;
	int h_index;
	int p_index;
	int *point_count = new int[return_cloud->width * return_cloud->height];
	memset(point_count, 0, sizeof(int) * return_cloud->width * return_cloud->height);
	for(int i = 0; i < input->points.size(); i++)
	{
		w_index = (int) ((input->points[i].y - range.min_y) / SORTING_SIZE);
		h_index = (int) ((input->points[i].z - range.min_z) / SORTING_SIZE);
		//printf("[%d] W : %d\tH : %d\tY : %0.3lf\tZ : %0.3lf\n", i, w_index, h_index, input->points[i].y, input->points[i].z);
		if(w_index > 0 && w_index < return_cloud->width && h_index > 0 && h_index < return_cloud->height)
		{
			p_index = h_index * return_cloud->width + w_index;
			return_cloud->points[p_index].x += input->points[i].x;
			return_cloud->points[p_index].y += input->points[i].y;
			return_cloud->points[p_index].z += input->points[i].z;
			point_count[p_index]++;
		}
	}

	for(int i = 0; i < return_cloud->points.size(); i++)
	{
		if(point_count[i] != 0)
		{
			return_cloud->points[i].x = return_cloud->points[i].x / point_count[i];
			return_cloud->points[i].y = return_cloud->points[i].y / point_count[i];
			return_cloud->points[i].z = return_cloud->points[i].z / point_count[i];
		}
	}
	//for(int i = 0; i < (int) return_cloud->points.size(); i++) printf("Point[%d] X : %0.3lf\tY : %0.3lf\tZ : %0.3lf\n", i, return_cloud->points[i].x, return_cloud->points[i].y, return_cloud->points[i].z);
	delete[] point_count;
	return return_cloud;
}

//Plane Segmentation
vector<eun_u::Object3D> pcl_tools::plane_extract(pcl::PointCloud<pcl::PointXYZ>::Ptr& input, int num_plane)
{
	vector<eun_u::Object3D> return_object;
	if((int) return_object.size() > 0) return_object.clear();

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	if((int) input->points.size() > 100)
	{
		for(int i = 0; i < num_plane; i++)
		{
			printf("plane[%d] input Point num : %d\n", i, (int) input->points.size());

			m_seg_plane.setInputCloud(input);
			m_seg_plane.segment(*inliers, *coefficients);
			if (inliers->indices.size () == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			m_extract.setInputCloud(input);
			m_extract.setIndices(inliers);
			m_extract.setNegative(false);
			m_extract.filter(*cloud_plane);

			// Remove the planar inliers, extract the rest
			m_extract.setNegative (true);
			m_extract.filter(*filter_cloud);
			*input = *filter_cloud;
			//printf("plane[%d] Plane num : %d\n", i, (int) cloud_plane->points.size());
			//printf("plane[%d] Remain num : %d\n", i, (int) input->points.size());

			eun_u::Point3D temp_point;
			eun_u::Object3D temp_object;
			for(int i = 0; i < (int) cloud_plane->points.size(); i++)
			{
				temp_point.x = cloud_plane->points[i].x;
				temp_point.y = cloud_plane->points[i].y;
				temp_point.z = cloud_plane->points[i].z;
				temp_object.points.push_back(temp_point);
			}

			temp_object.center_position = mat_cal.cal_medianPoint3D(temp_object.points);
			return_object.push_back(temp_object);

			if((int) input->points.size() < 100)	break;
		}
	}

	return return_object;
}

vector<eun_u::Object3D> pcl_tools::euclidean_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
	vector<eun_u::Object3D> return_object;
	if((int) return_object.size() > 0 ) return_object.clear();

	m_Kdtree->setInputCloud(input);
	std::vector<pcl::PointIndices> cluster_indices;
	m_ec.setInputCloud (input);
	m_ec.extract (cluster_indices);

	if((int) cluster_indices.size() > 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			eun_u::Object3D temp_object;

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				eun_u::Point3D temp_point;
				temp_point.x = input->points[*pit].x;
				temp_point.y = input->points[*pit].y;
				temp_point.z = input->points[*pit].z;
				temp_object.points.push_back(temp_point);
			}

			temp_object.center_position = mat_cal.cal_medianPoint3D(temp_object.points);
			return_object.push_back(temp_object);
		}
	}

	return return_object;
}

vector<eun_u::Object3D> pcl_tools::rg_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
	vector<eun_u::Object3D> return_object;
	if((int) return_object.size() > 0 ) return_object.clear();

	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	m_normal_estimator.setInputCloud(input);
	m_normal_estimator.compute (*normals);

	m_reg.setInputNormals (normals);
	m_reg.setInputCloud (input);
	std::vector <pcl::PointIndices> clusters;
	m_reg.extract (clusters);

	if((int) clusters.size() > 0)
	{
		for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
		{
			eun_u::Object3D temp_object;

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				eun_u::Point3D temp_point;
				temp_point.x = input->points[*pit].x;
				temp_point.y = input->points[*pit].y;
				temp_point.z = input->points[*pit].z;
				temp_object.points.push_back(temp_point);
			}

			temp_object.center_position = mat_cal.cal_medianPoint3D(temp_object.points);
			return_object.push_back(temp_object);
		}
	}

	return return_object;
}


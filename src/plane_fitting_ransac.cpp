#include "plane_fitting_ransac.h"

PlaneFittingRansac::PlaneFittingRansac(double distance_threshold_,double cluster_tolerance_, int min_cluster_size_,int max_cluster_size_, bool do_refine_, double  table_z_filter_min_, double table_z_filter_max_,double z_filter_min_, double z_filter_max_, double plane_detection_voxel_size_, double clustering_voxel_size_, int inlier_threshold_) :
	PlaneFitting(table_z_filter_min_,table_z_filter_max_,distance_threshold_, cluster_tolerance_, min_cluster_size_, max_cluster_size_, do_refine_),
	z_filter_min(z_filter_min_),
	z_filter_max(z_filter_max_),
	plane_detection_voxel_size(plane_detection_voxel_size_),
	clustering_voxel_size(clustering_voxel_size_),
	inlier_threshold(inlier_threshold_),
	cloud_filtered_ptr(new PointCloudT()),
	cloud_downsampled_ptr(new PointCloudT()),
	cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>()),
	table_inliers_ptr(new pcl::PointIndices()),
	table_coefficients_ptr(new pcl::ModelCoefficients)
{
  	// Filtering parameters
	grid_.setLeafSize (plane_detection_voxel_size, plane_detection_voxel_size, plane_detection_voxel_size);
	grid_.setFilterFieldName ("z");
	grid_.setFilterLimits (z_filter_min, z_filter_max);
	grid_.setDownsampleAllData (false);


	grid_objects_.setLeafSize (clustering_voxel_size, clustering_voxel_size, clustering_voxel_size);
	grid_objects_.setDownsampleAllData (false);


	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (z_filter_min, z_filter_max);

	normals_tree_ = boost::make_shared<KdTree> ();
	clusters_tree_ = boost::make_shared<KdTree> ();

	// Normal estimation parameters
	n3d_.setKSearch (10);  
	n3d_.setSearchMethod (normals_tree_);

	// Table model fitting parameters
	seg_.setDistanceThreshold (0.05); 
	seg_.setMaxIterations (10000);
	seg_.setNormalDistanceWeight (0.1);
	seg_.setOptimizeCoefficients (do_refine_);
	seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setProbability (0.99);

}

FittingData PlaneFittingRansac::fit(const PointCloudT::ConstPtr & point_cloud_in_)
{

	// Step 1 : Filter, remove NaNs and downsample
	pass_.setInputCloud (point_cloud_in_);
	pass_.filter (*cloud_filtered_ptr);

	grid_.setInputCloud (cloud_filtered_ptr);
	grid_.filter (*cloud_downsampled_ptr);

	if (cloud_filtered_ptr->points.size() < (unsigned int)min_cluster_size)
	{
		PCL_ERROR ("Downsampled cloud only has points %d", (int)cloud_filtered_ptr->points.size());
		PCL_THROW_EXCEPTION (FittingException, "Downsampled cloud only has only " << (int)cloud_downsampled_ptr->points.size());
	}

	// Step 2 : Estimate normals
	n3d_.setInputCloud (cloud_downsampled_ptr);
	n3d_.compute (*cloud_normals_ptr);


	// Step 3 : Perform planar segmentation
	seg_.setInputCloud (cloud_downsampled_ptr);
	seg_.setInputNormals (cloud_normals_ptr);
	seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);

	//pcl::SACSegmentation<PointT> seg;
	//seg.setOptimizeCoefficients (do_refine);
	//seg.setModelType (pcl::SACMODEL_PLANE);
	//seg.setMethodType (pcl::SAC_RANSAC);
	//seg.setDistanceThreshold (distance_threshold);
	//seg.setInputCloud (cloud_filtered_ptr);
	//seg.segment (table_inliers, *table_coefficients);*/


	if (table_inliers_ptr->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		PCL_THROW_EXCEPTION (FittingException, "points belonging to flat surface not found ");
	}
	if ( table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold)
	{
		PCL_ERROR("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(), inlier_threshold);
		PCL_THROW_EXCEPTION (FittingException, "Failed to detect point cloud.");
	}
	if (table_coefficients_ptr->values.size () <=3)
	{
		PCL_ERROR ("Failed to detect point cloud.");
		PCL_THROW_EXCEPTION (FittingException, "Failed to detect point cloud.");
	}


	proj_.setInputCloud (cloud_downsampled_ptr);
	proj_.setIndices (table_inliers_ptr);
	proj_.setModelCoefficients (table_coefficients_ptr);
	proj_.filter (*table_cloud);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud_downsampled_ptr);
	extract.setIndices (table_inliers_ptr);
	extract.setNegative (true);
	extract.filter (*cloud_all_minus_table);

	Eigen::Vector4f coefficients_eigen(table_coefficients_ptr->values.data());
	double confidence=1.0;
	return FittingData(coefficients_eigen,confidence,FittingData::PLANE,table_cloud,cloud_all_minus_table);
}





#include "plane_fitting_ransac.h"

PlaneFittingRansac::PlaneFittingRansac(double distance_threshold_,double cluster_tolerance_, int min_cluster_size_,int max_cluster_size_, bool do_refine_, double  table_z_filter_min_, double table_z_filter_max_,double z_filter_min_, double z_filter_max_, double plane_detection_voxel_size_, double clustering_voxel_size_, int inlier_threshold_) :
	PlaneFitting(distance_threshold_, cluster_tolerance_, min_cluster_size_, max_cluster_size_, do_refine_),
	table_z_filter_min(table_z_filter_min_),
	table_z_filter_max(table_z_filter_max_),
	z_filter_min(z_filter_min_),
	z_filter_max(z_filter_max_),
	plane_detection_voxel_size(plane_detection_voxel_size_),
	clustering_voxel_size(clustering_voxel_size_),
	inlier_threshold(inlier_threshold_)
{

}

FittingData PlaneFittingRansac::fit(const PointCloudT::ConstPtr & point_cloud_in_)
{
	pcl::VoxelGrid<PointT> grid_, grid_objects_;
	pcl::PassThrough<PointT> pass_;
 	KdTreePtr normals_tree_, clusters_tree_;
	pcl::NormalEstimation<PointT, pcl::Normal> n3d_;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;
	pcl::ProjectInliers<PointT> proj_;
	pcl::ConvexHull<PointT> hull_;

	pcl::ExtractPolygonalPrismData<PointT> prism_;
	pcl::EuclideanClusterExtraction<PointT> pcl_cluster_;
	PointCloudT::Ptr table_cloud_hull(new PointCloudT);
  	// Filtering parameters
	grid_.setLeafSize (plane_detection_voxel_size, plane_detection_voxel_size, plane_detection_voxel_size);
	grid_objects_.setLeafSize (clustering_voxel_size, clustering_voxel_size, clustering_voxel_size);
	grid_.setFilterFieldName ("z");
	pass_.setFilterFieldName ("z");

	pass_.setFilterLimits (z_filter_min, z_filter_max);
	grid_.setFilterLimits (z_filter_min, z_filter_max);
	grid_.setDownsampleAllData (false);
	grid_objects_.setDownsampleAllData (false);


	normals_tree_ = boost::make_shared<KdTree> ();
	clusters_tree_ = boost::make_shared<KdTree> ();

	// Normal estimation parameters
	n3d_.setKSearch (10);  
	n3d_.setSearchMethod (normals_tree_);
	// Table model fitting parameters
	seg_.setDistanceThreshold (0.05); 
	seg_.setMaxIterations (10000);
	seg_.setNormalDistanceWeight (0.1);
	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);
	seg_.setProbability (0.99);

	// Step 1 : Filter, remove NaNs and downsample
	PointCloudT cloud_filtered;
	pass_.setInputCloud (point_cloud_in_);
	pass_.filter (cloud_filtered);
	PointCloudT::ConstPtr cloud_filtered_ptr = boost::make_shared<const PointCloudT > (cloud_filtered);

	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients (do_refine);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distance_threshold);
	seg.setInputCloud (point_cloud_in_);
	seg.segment (*plane_indices, *coefficients);

	if (plane_indices->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		PCL_THROW_EXCEPTION (FittingException, "points belonging to flat surface not found ");
	}

	PointCloudT cloud_downsampled;
	grid_.setInputCloud (cloud_filtered_ptr);
	grid_.filter (cloud_downsampled);
	PointCloudT::ConstPtr cloud_downsampled_ptr = boost::make_shared<const PointCloudT > (cloud_downsampled);
	if (cloud_downsampled.points.size() < (unsigned int)min_cluster_size)
	{
		PCL_ERROR ("Downsampled cloud only has points %d", (int)cloud_downsampled.points.size());
		PCL_THROW_EXCEPTION (FittingException, "Downsampled cloud only has only " << (int)cloud_downsampled.points.size());
	}


	// Step 2 : Estimate normals
	pcl::PointCloud<pcl::Normal> cloud_normals;
	n3d_.setInputCloud (cloud_downsampled_ptr);
	n3d_.compute (cloud_normals);
	pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);


	// Step 3 : Perform planar segmentation
	pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
	seg_.setInputCloud (cloud_downsampled_ptr);
	seg_.setInputNormals (cloud_normals_ptr);
	seg_.segment (table_inliers, table_coefficients);
	pcl::PointIndices::ConstPtr table_inliers_ptr = boost::make_shared<const pcl::PointIndices> (table_inliers);
	pcl::ModelCoefficients::ConstPtr table_coefficients_ptr = boost::make_shared<const pcl::ModelCoefficients> (table_coefficients);

	if (table_coefficients.values.size () <=3)
	{
		PCL_ERROR ("Failed to detect point cloud.");
		PCL_THROW_EXCEPTION (FittingException, "Failed to detect point cloud.");
	}

	if ( table_inliers.indices.size() < (unsigned int)inlier_threshold)
	{
		PCL_ERROR("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers.indices.size(), inlier_threshold);
		PCL_THROW_EXCEPTION (FittingException, "Failed to detect point cloud.");
	}


	PointCloudT table_projected;
	proj_.setInputCloud (cloud_downsampled_ptr);
	proj_.setIndices (table_inliers_ptr);
	proj_.setModelCoefficients (table_coefficients_ptr);
	proj_.filter (*table_cloud);

	// Create a Convex Hull representation of the projected inliers
	pcl::ConvexHull<PointT> chull;
	chull.setInputCloud (table_cloud);
	chull.reconstruct (*table_cloud_hull);

	// segment those points that are in the polygonal prism
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> ex;
	ex.setInputCloud (table_cloud);
	ex.setInputPlanarHull (table_cloud_hull);
	ex.segment (*convex_hull_indices);

	// ---[ Get the objects on top of the table
	pcl::PointIndices cloud_object_indices;
	//prism_.setInputCloud (cloud_all_minus_table_ptr);
	ex.setInputCloud (point_cloud_in_);
	ex.setInputPlanarHull (table_cloud_hull);
	ex.setHeightLimits (table_z_filter_min, table_z_filter_max);  
	ex.segment (cloud_object_indices);

	PointCloudT cloud_objects;
	pcl::ExtractIndices<PointT> extract_object_indices;
	extract_object_indices.setInputCloud (point_cloud_in_);
	extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
	extract_object_indices.filter (cloud_objects);
  	PointCloudT::ConstPtr cloud_objects_ptr = boost::make_shared<const PointCloudT > (cloud_objects);


	if (cloud_objects.points.empty ()) 
	{
		PCL_ERROR ("No objects on plane.");
		PCL_THROW_EXCEPTION (FittingException, "No objects on plane.");
	}

	// ---[ Downsample the points
	PointCloudT cloud_objects_downsampled;
	grid_objects_.setInputCloud (cloud_objects_ptr);
	grid_objects_.filter (cloud_objects_downsampled);
	PointCloudT::ConstPtr cloud_objects_downsampled_ptr = boost::make_shared <const PointCloudT > (cloud_objects_downsampled);

	// ---[ Split the objects into Euclidean clusters
	std::vector<pcl::PointIndices> clusters2;
	pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
	pcl_cluster_.extract (clusters2);

 	std::vector<PointCloudT> clusters;
	getClustersFromPointCloud (cloud_objects, clusters2, clusters);

	Eigen::Vector4f coefficients_eigen(table_coefficients_ptr->values.data());

	return FittingData(coefficients_eigen,1.0,FittingData::PLANE,table_cloud);
}





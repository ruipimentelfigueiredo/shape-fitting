#include "plane_fitting_ransac.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
PlaneFittingRansac::PlaneFittingRansac(double distance_threshold_,double cluster_tolerance_, int min_cluster_size_,int max_cluster_size_, bool do_refine_, double  table_z_filter_min_, double table_z_filter_max_,double z_filter_min_, double z_filter_max_, double plane_detection_voxel_size_, double clustering_voxel_size_, int inlier_threshold_) :
	PlaneFitting(table_z_filter_min_,table_z_filter_max_,distance_threshold_, cluster_tolerance_, min_cluster_size_, max_cluster_size_, do_refine_),
	z_filter_min(z_filter_min_),
	z_filter_max(z_filter_max_),
	plane_detection_voxel_size(plane_detection_voxel_size_),
	clustering_voxel_size(clustering_voxel_size_),
	inlier_threshold(inlier_threshold_),
	cloud_downsampled_ptr(new PointCloudT()),
	cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>()),
	table_coefficients_ptr(new pcl::ModelCoefficients)
{
  	// Filtering parameters
	grid_.setLeafSize (plane_detection_voxel_size, plane_detection_voxel_size, plane_detection_voxel_size);
	grid_.setFilterFieldName ("z");
	grid_.setFilterLimits (z_filter_min, z_filter_max);
	grid_.setDownsampleAllData (false);

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

	// Set up Organized Multi Plane Segmentation
	mps.setMinInliers (5000);
	mps.setAngularThreshold (pcl::deg2rad (5.0)); 
	mps.setDistanceThreshold (0.1);
}

FittingData PlaneFittingRansac::fit(const PointCloudT::ConstPtr & point_cloud_in_)
{
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	// Specify method for normal estimation
	norm_est.setNormalEstimationMethod (norm_est.AVERAGE_3D_GRADIENT);
	// Specify max depth change factor
	norm_est.setMaxDepthChangeFactor(0.02f);
	// Specify smoothing area size
	norm_est.setNormalSmoothingSize(20.0f);
	// Set the input points
	norm_est.setInputCloud(point_cloud_in_);
	// Estimate the surface normals and store the result in "normals_out"
	norm_est.compute(*cloud_normals_ptr);

	// visualize normals
	/*pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor (0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(point_cloud_in_, cloud_normals_ptr);

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}*/

	// Segment Planes
	std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;  
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	mps.setInputNormals (cloud_normals_ptr);
	mps.setInputCloud (point_cloud_in_);
	mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

 	pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());

	//Segment Objects
	pcl::PointCloud<PointT>::CloudVectorType clusters;

	if (regions.size () > 0)
	{
		std::cout << regions.size() << std::endl;
		*table_coefficients_ptr=model_coefficients[0];

		table_inliers_ptr->indices=inlier_indices[0].indices;
		std::vector< bool > plane_labels; 
		plane_labels.resize(label_indices.size());
		for (size_t i = 0; i < label_indices.size (); ++i)
			if (label_indices[i].indices.size () > 300)
				plane_labels[i]=true;
			else
				plane_labels[i]=false;	

		euclidean_cluster_comparator_->setInputCloud (point_cloud_in_);
		euclidean_cluster_comparator_->setLabels (labels);
		euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
		euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

		pcl::PointCloud<pcl::Label> euclidean_labels;
		std::vector<pcl::PointIndices> euclidean_label_indices;
		pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation(euclidean_cluster_comparator_);
		euclidean_segmentation.setInputCloud (point_cloud_in_);
		euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

		for (size_t i = 0; i < euclidean_label_indices.size (); i++)
		{
			if (euclidean_label_indices[i].indices.size () > 1000)
			{
				pcl::PointCloud<PointT> cluster;
				pcl::copyPointCloud (*point_cloud_in_,euclidean_label_indices[i].indices,cluster);
				clusters.push_back (cluster);
			}    
		}

		PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
	}
	else
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		PCL_THROW_EXCEPTION (FittingException, "points belonging to flat surface not found");
	}

	// Step 3 : Perform planar segmentation
	/*seg_.setInputCloud (cloud_downsampled_ptr);
	seg_.setInputNormals (cloud_normals_ptr);
	seg_.segment (*table_inliers_ptr, *table_coefficients_ptr);
	std::vector<int>::iterator iter = table_inliers_ptr->indices.begin();*/

	std::vector<int>::iterator iter = table_inliers_ptr->indices.begin();
	while (iter != table_inliers_ptr->indices.end())
	{
		if (cloud_normals_ptr->points[*iter].curvature > 0.001)
		{
			// erase returns the new iterator
			iter = table_inliers_ptr->indices.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	/*for(int i=0; i< table_inliers_ptr->indices.size(); ++i)
	{
		//std::cout << cloud_normals_ptr->points[table_inliers_ptr->indices[i]].curvature << std::endl;
		if (cloud_normals_ptr->points[table_inliers_ptr->indices[i]].curvature > 0.04)
		{
			std::cout << "curvature:" << cloud_normals_ptr->points[table_inliers_ptr->indices[i]].curvature << std::endl;
		}
	}*/

	if (table_inliers_ptr->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		PCL_THROW_EXCEPTION (FittingException, "points belonging to flat surface not found ");
	}

	if (table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold)
	{
		PCL_ERROR("Plane detection has %d inliers, below min threshold of %d\n", (int)table_inliers_ptr->indices.size(), inlier_threshold);
		PCL_THROW_EXCEPTION (FittingException, "Failed to detect point cloud.");
	}

	if (table_coefficients_ptr->values.size () <=3)
	{
		PCL_ERROR ("Failed to detect point cloud.\n");
		PCL_THROW_EXCEPTION (FittingException, "Failed to detect point cloud.");
	}

	proj_.setInputCloud (point_cloud_in_);
	proj_.setIndices (table_inliers_ptr);
	proj_.setModelCoefficients (table_coefficients_ptr);
	proj_.filter (*table_cloud);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (point_cloud_in_);
	extract.setIndices (table_inliers_ptr);
	extract.setNegative (true);
	extract.filter (*table_outliers_cloud);

	Eigen::Vector4f coefficients_eigen(table_coefficients_ptr->values.data());
	double confidence=1.0;
	return FittingData(coefficients_eigen,confidence,FittingData::PLANE,table_cloud,table_outliers_cloud);
}





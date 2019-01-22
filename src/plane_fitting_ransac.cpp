#include "plane_fitting_ransac.h"

PlaneFittingRansac::PlaneFittingRansac(double distance_threshold_, double cluster_tolerance_, int min_cluster_size_, int max_cluster_size_, bool do_refine_, double table_z_filter_min_, double table_z_filter_max_, double z_filter_min_, double z_filter_max_, double plane_detection_voxel_size_, double clustering_voxel_size_, int inlier_threshold_, double angle_threshold_) : 
							PlaneFitting(table_z_filter_min_, table_z_filter_max_, cluster_tolerance_, min_cluster_size_, max_cluster_size_, inlier_threshold_, do_refine_),
								z_filter_min(z_filter_min_),
								z_filter_max(z_filter_max_),
								plane_detection_voxel_size(plane_detection_voxel_size_),
								clustering_voxel_size(clustering_voxel_size_),
								angle_threshold(pcl::deg2rad(angle_threshold_)),	
								cloud_filtered_ptr(new PointCloudT()),
								cloud_downsampled_ptr(new PointCloudT()),
								cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>()),
								table_inliers_ptr(new pcl::PointIndices()),																																																																																							 
								table_coefficients_ptr(new pcl::ModelCoefficients)
{
	// Filtering parameters
	grid_.setLeafSize(plane_detection_voxel_size, plane_detection_voxel_size, plane_detection_voxel_size);
	grid_.setFilterFieldName("z");
	grid_.setFilterLimits(z_filter_min, z_filter_max);
	grid_.setDownsampleAllData(false);

	grid_objects_.setLeafSize(clustering_voxel_size, clustering_voxel_size, clustering_voxel_size);
	grid_objects_.setDownsampleAllData(false);

	normals_tree_ = boost::make_shared<KdTree>();
	clusters_tree_ = boost::make_shared<KdTree>();

	// Normal estimation parameters TODO: TIRAR
	n3d_.setKSearch(10);
	n3d_.setSearchMethod(normals_tree_);

	// Table model fitting parameters
	seg_.setDistanceThreshold(0.05);
	seg_.setMaxIterations(10000);
	seg_.setNormalDistanceWeight(0.1);
	seg_.setOptimizeCoefficients(do_refine_);
	seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg_.setMethodType(pcl::SAC_RANSAC);
	seg_.setProbability(0.99);

	table_coefficients_ptr->values.resize(4);

	mps.setMinInliers(inlier_threshold);
	mps.setAngularThreshold(angle_threshold); 
	mps.setDistanceThreshold(table_z_filter_min); 
	//mps.setMaximumCurvature();
}

FittingData PlaneFittingRansac::fit(const PointCloudT::ConstPtr &point_cloud_in_,const pcl::PointCloud<pcl::Normal>::ConstPtr & point_cloud_normal_in_, float* distance_map)
{
	// Segment Planes
	if(point_cloud_in_->isOrganized())
	{
		std::vector<pcl::PointIndices> boundary_indices;
		std::vector<pcl::ModelCoefficients> model_coefficients;

		label_indices.clear();
		labels->clear();
		inlier_indices.clear();

		// Set up Organized Multi Plane Segmentation
		mps.setInputNormals(point_cloud_normal_in_);
		mps.setInputCloud(point_cloud_in_);
		//mps.segment (model_coefficients, inlier_indices);
		mps.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		if (model_coefficients.size() > 0)
		{
			boundary_cloud->points = regions[0].getContour ();
		
			// Extract table
			*table_coefficients_ptr = model_coefficients[0];
			*table_inliers_ptr = inlier_indices[0];

			proj_.setInputCloud(point_cloud_in_);
			proj_.setIndices(table_inliers_ptr);
			proj_.setModelCoefficients(table_coefficients_ptr);
			proj_.filter(*table_cloud);

			pcl::ExtractIndices<PointT> extract;
			extract.setInputCloud(point_cloud_in_);
			extract.setIndices(table_inliers_ptr);
			extract.setNegative(true);
			extract.filter(*cloud_all_minus_table);

			Eigen::Vector4f coefficients_eigen(table_coefficients_ptr->values.data());
			double confidence = 1.0;
			return FittingData(coefficients_eigen, confidence, FittingData::PLANE, table_cloud, cloud_all_minus_table,boundary_cloud);
		}
		else
		{
			PCL_ERROR("Failed to detect planes.\n");
			PCL_THROW_EXCEPTION(FittingException, "Failed to detect planes.\n");
		}
	}
	else
	{
		grid_.setInputCloud(cloud_filtered_ptr);
		grid_.filter(*cloud_downsampled_ptr);

		if (cloud_filtered_ptr->points.size() < (unsigned int)inlier_threshold)
		{
			PCL_ERROR("Downsampled cloud only has points %d", (int)cloud_filtered_ptr->points.size());
			PCL_THROW_EXCEPTION(FittingException, "Downsampled cloud only has only " << (int)cloud_downsampled_ptr->points.size());
		}

		// Step 2 : Estimate normals (TODO MOVE ALL PRE-PROCESSING OUTSIDE)
		n3d_.setInputCloud(cloud_downsampled_ptr);
		n3d_.compute(*cloud_normals_ptr);

		// Step 3 : Perform planar segmentation
		seg_.setInputCloud(cloud_downsampled_ptr);
		seg_.setInputNormals(cloud_normals_ptr);
		seg_.segment(*table_inliers_ptr, *table_coefficients_ptr);

		if (table_inliers_ptr->indices.size() == 0)
		{
			PCL_ERROR("Could not estimate a planar model for the given dataset.");
			PCL_THROW_EXCEPTION(FittingException, "points belonging to flat surface not found ");
		}
		if (table_inliers_ptr->indices.size() < (unsigned int)inlier_threshold)
		{
			PCL_ERROR("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers_ptr->indices.size(), inlier_threshold);
			PCL_THROW_EXCEPTION(FittingException, "Failed to detect point cloud.");
		}
		if (table_coefficients_ptr->values.size() <= 3)
		{
			PCL_ERROR("Failed to detect point cloud.");
			PCL_THROW_EXCEPTION(FittingException, "Failed to detect point cloud.");
		}

		proj_.setInputCloud(cloud_downsampled_ptr);
		proj_.setIndices(table_inliers_ptr);
		proj_.setModelCoefficients(table_coefficients_ptr);
		proj_.filter(*table_cloud);

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_downsampled_ptr);
		extract.setIndices(table_inliers_ptr);
		extract.setNegative(true);
		extract.filter(*cloud_all_minus_table);

		Eigen::Vector4f coefficients_eigen(table_coefficients_ptr->values.data());
		double confidence = 1.0;

		pcl::ConvexHull<PointT> chull;
		chull.setInputCloud(table_cloud);
		chull.reconstruct(*table_cloud_hull);

		// segment those points that are in the polygonal prism
		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> ex;

		// Segment points in plane
		ex.setInputCloud(point_cloud_in_);
		ex.setInputPlanarHull(table_cloud_hull);
		ex.setHeightLimits(-table_z_filter_min, table_z_filter_min);
		ex.segment(*convex_hull_indices);

		extract.setInputCloud(point_cloud_in_);
		extract.setIndices(convex_hull_indices);
		extract.setNegative(false);
		extract.filter(*table_cloud);

		// Segment points above plane
		/*ex.setInputCloud(point_cloud_in_);
		ex.setInputPlanarHull(table_cloud_hull);
		ex.setHeightLimits(table_z_filter_min,table_z_filter_max);
		ex.segment(*convex_hull_indices);

		extract.setInputCloud(point_cloud_in_);
		extract.setIndices(convex_hull_indices);
		extract.setNegative(false);
		extract.filter(*cloud_all_minus_table);*/
		return FittingData(coefficients_eigen, confidence, FittingData::PLANE, table_cloud, cloud_all_minus_table, boundary_cloud);
	}
}

#include "plane_fitting.h"


PlaneFitting::PlaneFitting(double table_z_filter_min_, double table_z_filter_max_,double distance_threshold_,double cluster_tolerance_, int min_cluster_size_,int max_cluster_size_, bool do_refine_) : 
	table_z_filter_min(table_z_filter_min_),
	table_z_filter_max(table_z_filter_max_),
	distance_threshold(distance_threshold_),
	cluster_tolerance(cluster_tolerance_),
	min_cluster_size(min_cluster_size_),
	max_cluster_size(max_cluster_size_),
	do_refine(do_refine_),
	cloud_filtered(new pcl::PointCloud<PointT>),
	table_cloud(new pcl::PointCloud<PointT> ()),
	cloud_all_minus_table(new pcl::PointCloud<PointT> ()),
	table_cloud_hull(new PointCloudT()),
	coefficients (new pcl::ModelCoefficients),
	plane_indices (new pcl::PointIndices),
	convex_hull_indices (new pcl::PointIndices),
	tree(new pcl::search::KdTree<PointT>)
{

}

/*void PlaneFitting::ExtractTableTopClusters(PointCloudT::ConstPtr input_cloud, std::vector<pcl::PointIndices> & clusters_indices)
{
  	tree->setInputCloud (cloud_filtered);
	// Create EuclideanClusterExtraction and set parameters
	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setSearchMethod (tree);
	ec.setClusterTolerance (cluster_tolerance);
	ec.setMinClusterSize (min_cluster_size);
	ec.setMaxClusterSize (max_cluster_size);
	// set input cloud and let it run
	ec.setInputCloud (input_cloud);
	ec.extract (clusters_indices);

	int j = 0;
	std::vector<PointCloudT::Ptr> clusters;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin (); it != clusters_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
		//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		//std::stringstream ss;
		//ss << "cloud_cluster_" << j << ".pcd";
		//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		j++;
	}


}*/

pcl::PointIndices::Ptr PlaneFitting::getConvexHull()
{
	return this->convex_hull_indices;
}

PointCloudT::Ptr PlaneFitting::getTableCloud()
{
	return this->table_cloud;
}


void PlaneFitting::getClustersFromPointCloud (const PointCloudT &cloud_objects, 			    
			    const std::vector<pcl::PointIndices> &clusters2, 
			    std::vector<PointCloudT> &clusters)
{
	clusters.resize (clusters2.size ());
	for (size_t i = 0; i < clusters2.size (); ++i)
	{
		pcl::PointCloud<PointT> cloud_cluster;
		pcl::copyPointCloud(cloud_objects, clusters2[i], clusters[i]);    
	}
}

void PlaneFitting::extractTabletopClusters(PointCloudT::ConstPtr input_cloud, std::vector<PointCloudT::Ptr> & clusters_point_clouds)
{
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
	ex.setInputCloud (input_cloud);
	ex.setInputPlanarHull (table_cloud_hull);
	ex.setHeightLimits (table_z_filter_min, table_z_filter_max);  
	ex.segment (cloud_object_indices);

	PointCloudT cloud_objects;
	pcl::ExtractIndices<PointT> extract_object_indices;
	extract_object_indices.setInputCloud (input_cloud);
	extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
	extract_object_indices.filter (cloud_objects);
  	PointCloudT::ConstPtr cloud_objects_ptr = boost::make_shared<const PointCloudT > (cloud_objects);

	if (cloud_objects_ptr->points.empty ()) 
	{
		PCL_ERROR ("No objects on plane.");
		return;
		PCL_THROW_EXCEPTION (FittingException, "No objects on plane.");
	}

	std::vector<pcl::PointIndices> clusters_indices;
  	tree->setInputCloud (cloud_objects_ptr);

	// Create EuclideanClusterExtraction and set parameters
	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setSearchMethod (tree);
	ec.setClusterTolerance (cluster_tolerance);
	ec.setMinClusterSize (min_cluster_size);
	ec.setMaxClusterSize (max_cluster_size);
	// set input cloud and let it run
	ec.setInputCloud (cloud_objects_ptr);
	ec.extract (clusters_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin (); it != clusters_indices.end (); ++it)
	{
		// Extract the inliers
		PointCloudT::Ptr cloud_cluster (new PointCloudT);

		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (cloud_objects_ptr->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters_point_clouds.push_back(cloud_cluster);
	}
}


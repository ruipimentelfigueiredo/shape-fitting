#include "plane_fitting.h"


PlaneFitting::PlaneFitting(double distance_threshold_,double cluster_tolerance_, int min_cluster_size_,int max_cluster_size_, bool do_refine_) : 
	distance_threshold(distance_threshold_),
	cluster_tolerance(cluster_tolerance_),
	min_cluster_size(min_cluster_size_),
	max_cluster_size(max_cluster_size_),
	do_refine(do_refine_),
	cloud_filtered(new pcl::PointCloud<PointT>),
	table_cloud(new pcl::PointCloud<PointT> ()),
	coefficients (new pcl::ModelCoefficients),
	plane_indices (new pcl::PointIndices),
	convex_hull_indices (new pcl::PointIndices),
	tree(new pcl::search::KdTree<PointT>)
{

}

void PlaneFitting::ExtractTableTopClusters(PointCloudT::ConstPtr input_cloud, std::vector<pcl::PointIndices> & clusters_indices)
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


}

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


#include "plane_fitting.h"

PlaneFitting::PlaneFitting(double table_z_filter_min_, double table_z_filter_max_,double cluster_tolerance_, int min_cluster_size_,int max_cluster_size_, int inlier_threshold_, bool do_refine_) : 
	table_z_filter_min(table_z_filter_min_),
	table_z_filter_max(table_z_filter_max_),
	cluster_tolerance(cluster_tolerance_),
	min_cluster_size(min_cluster_size_),
	max_cluster_size(max_cluster_size_),
	do_refine(do_refine_),
	cloud_filtered(new pcl::PointCloud<PointT>),
	table_cloud(new pcl::PointCloud<PointT> ()),
	cloud_all_minus_table(new pcl::PointCloud<PointT> ()),
	table_cloud_hull(new PointCloudT()),
	boundary_cloud(new PointCloudT()),
	coefficients (new pcl::ModelCoefficients),
	plane_indices (new pcl::PointIndices),
	convex_hull_indices (new pcl::PointIndices),
	tree(new pcl::search::KdTree<PointT>),
	labels(new pcl::PointCloud<pcl::Label>()),
	inlier_threshold(inlier_threshold_)
{
	euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<pcl::PointXYZ,pcl::Normal,pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<pcl::PointXYZ,pcl::Normal,pcl::Label> ());
	edge_aware_comparator_.reset (new pcl::EdgeAwarePlaneComparator<PointT, pcl::Normal> ());
	euclidean_cluster_comparator_->setDistanceThreshold (cluster_tolerance, false);
}

pcl::PointIndices::Ptr PlaneFitting::getConvexHull()
{
	return this->convex_hull_indices;
}

PointCloudT::Ptr PlaneFitting::getTableCloud()
{
	return this->table_cloud;
}


void PlaneFitting::getClustersFromPointCloud (
				const PointCloudT &cloud_objects, 			    
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

void PlaneFitting::extractTabletopClusters(PointCloudT::ConstPtr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr input_normals, std::vector<PointCloudT::Ptr> & clusters_point_clouds, std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> & clusters_)
{
	if(input_cloud->isOrganized())
	{
		//Segment Objects
		if (regions.size () > 0)
		{
			// The function: void setExcludeLabels (const std::vector<bool>& exclude_labels)
			// has been deprecated in PCL 1.10! 
			// Change to new function.
			
			// std::vector<bool> plane_labels;
			// for (size_t i = 0; i < label_indices.size (); ++i)
			// {
			// 	if (label_indices[i].indices.size () >= (unsigned int) inlier_threshold) // its a plane
			// 	{
			// 		plane_labels.push_back (true);
			// 	}
			// 	else
			// 	{
			// 		plane_labels.push_back (false);
			// 	}
			// }

		    pcl::EuclideanClusterComparator<PointT, pcl::Label>::ExcludeLabelSetPtr plane_labels(new pcl::EuclideanClusterComparator<PointT, pcl::Label>::ExcludeLabelSet);
			for (size_t i = 0; i < label_indices.size (); ++i)
			{
				if (label_indices[i].indices.size () >= (unsigned int) inlier_threshold) // its a plane
				{
					plane_labels->insert(i);
				}
			}

			euclidean_cluster_comparator_->setInputCloud (input_cloud);
			euclidean_cluster_comparator_->setInputNormals (input_normals);
			euclidean_cluster_comparator_->setLabels (labels);
			euclidean_cluster_comparator_->setExcludeLabels (plane_labels);

			pcl::PointCloud<pcl::Label> euclidean_labels;
			std::vector<pcl::PointIndices> euclidean_label_indices;
			pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZ,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
			euclidean_segmentation.setInputCloud (input_cloud);
			euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
			
			for (size_t i = 0; i < euclidean_label_indices.size (); i++)
			{
				if (euclidean_label_indices[i].indices.size () > min_cluster_size && euclidean_label_indices[i].indices.size () < max_cluster_size)
				{
					pcl::PointCloud<pcl::PointNormal>::Ptr cloud_final (new pcl::PointCloud<pcl::PointNormal>());

					pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
					pcl::PointCloud<pcl::Normal>::Ptr cluster_normal(new pcl::PointCloud<pcl::Normal>());

					pcl::copyPointCloud(*input_cloud,euclidean_label_indices[i].indices,*cluster);
					pcl::copyPointCloud(*input_normals,euclidean_label_indices[i].indices,*cluster_normal);

					clusters_point_clouds.push_back (cluster);

					pcl::concatenateFields (*cluster, *cluster_normal, *cloud_final);
					clusters_.push_back(cloud_final);
				}    
			}			
		}
	}
	else
	{
		// segment those points that are in the polygonal prism
		// ---[ Get the objects on top of the table
		pcl::PointIndices cloud_object_indices;
		pcl::ExtractPolygonalPrismData<pcl::PointXYZ> ex;
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
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_final (new pcl::PointCloud<pcl::PointNormal>());
			PointCloudT::Ptr cloud_cluster (new PointCloudT);
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>());

			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				cloud_cluster->points.push_back (cloud_objects_ptr->points[*pit]); 
				cloud_normal->points.push_back(input_normals->points[*pit]);
			}

			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			cloud_normal->width = cloud_cluster->points.size();
			cloud_normal->height = 1;
			cloud_normal->is_dense = true;

			clusters_point_clouds.push_back(cloud_cluster);

			pcl::concatenateFields (*cloud_cluster, *cloud_normal, *cloud_final);

			clusters_.push_back(cloud_final);
		}
	}
}


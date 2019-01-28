/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef PLANEFITTING_H
#define PLANEFITTING_H
#include <random>
#include <sstream>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/common/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

#include "fitting_data.h"

class PlaneFitting
{
	protected:
	// params
	double table_z_filter_min; 
	double table_z_filter_max;
	double cluster_tolerance;
	double min_cluster_size;
	double max_cluster_size;

	// refine estimation
	bool do_refine;

	//aux structures
	PointCloudT::Ptr cloud_filtered;
	PointCloudT::Ptr table_cloud;
	PointCloudT::Ptr cloud_all_minus_table;
	PointCloudT::Ptr table_cloud_hull;
	PointCloudT::Ptr boundary_cloud;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PassThrough<PointT> pass;
	pcl::PointIndices::Ptr plane_indices;
	pcl::PointIndices::Ptr convex_hull_indices;
	pcl::search::KdTree<PointT>::Ptr tree;

	pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
	pcl::EdgeAwarePlaneComparator<pcl::PointXYZ, pcl::Normal>::Ptr edge_aware_comparator_;
	pcl::PointCloud<pcl::Label>::Ptr labels;
	std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT>>> regions;
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> inlier_indices;

	int inlier_threshold;
	pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;
	public:
		PlaneFitting(double table_z_filter_min_, double table_z_filter_max_, double cluster_tolerance_=0.02, int min_cluster_size_=0.02,int max_cluster_size_=0.5, int inlier_threshold_=500, bool do_refine_=false);

	void extractTabletopClusters(PointCloudT::ConstPtr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr input_normals, std::vector<PointCloudT::Ptr> & clusters_point_clouds, std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> & clusters_);

	Eigen::Matrix4f refine(const PointCloudT::ConstPtr & point_cloud_source_, const PointCloudT::ConstPtr & point_cloud_target_);
	virtual FittingData fit(const PointCloudT::ConstPtr &point_cloud_in_, const pcl::PointCloud<pcl::Normal>::ConstPtr & point_cloud_normal_in_, float* distance_map=NULL) = 0;

	pcl::PointIndices::Ptr getConvexHull();
	PointCloudT::Ptr getTableCloud();

	void getClustersFromPointCloud (const PointCloudT &cloud_objects, const std::vector<pcl::PointIndices> &clusters2, std::vector<PointCloudT> &clusters);
};


#endif // PLANEFITTING_H

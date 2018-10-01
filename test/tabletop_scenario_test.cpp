/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include <ctime>
#include <chrono>
#include "cylinder_fitting_hough.h"
#include "cylinder_fitting_ransac.h"
#include "plane_fitting_ransac.h"
#include "helpers.h"

int main (int argc, char** argv)
{
	/* TODO - Process options */
	if (argc < 6)
	{
		std::cout << "invalid number of arguments: dataset_dir heights radii noise clutter"<< std::endl;
		exit(-1);
	}

	std::string dataset_dir = std::string(argv[1]);
	std::cout << "dataset_dir: " << dataset_dir << std::endl;

	// CYLINDER FITTING HOUGH PARAMETERS
	unsigned int angle_bins=atoi(argv[2]);
	std::cout << "angle_bins: " << angle_bins<< std::endl;

	unsigned int radius_bins=atoi(argv[3]);
	std::cout << "radius_bins: " << radius_bins<< std::endl;

	unsigned int position_bins=atoi(argv[4]);
	std::cout << "position_bins: " << position_bins<< std::endl;

	// OUR HOUGH IMPLEMENTATION PARAMETERS
	unsigned int orientation_accumulators_num=atoi(argv[5]);
	std::cout << "orientation_accumulators_num: " << orientation_accumulators_num<< std::endl;

	unsigned int gaussian_sphere_points_num=atoi(argv[6]);
	std::cout << "gaussian_sphere_points_num: " << gaussian_sphere_points_num<< std::endl;
	
	double accumulator_peak_threshold=(double)atof(argv[7]);
	std::cout << "accumulator_peak_threshold: " << accumulator_peak_threshold<< std::endl;
	
	double min_radius=(double)atof(argv[8]);
	std::cout << "min_radius: " << min_radius<< std::endl;
		
	double max_radius=(double)atof(argv[9]);
	std::cout << "max_radius: " << max_radius<< std::endl;

	double distance_threshold=(double)atof(argv[10]);
	std::cout << "distance_threshold: " << distance_threshold<< std::endl;

	double cluster_tolerance=(double)atof(argv[11]);
	std::cout << "cluster_tolerance: " << cluster_tolerance<< std::endl;

	int min_cluster_size=atoi(argv[12]);
	std::cout << "min_cluster_size: " << min_cluster_size<< std::endl;

	int max_cluster_size=(double)atoi(argv[13]);
	std::cout << "max_cluster_size: " << max_cluster_size<< std::endl;

	bool do_refine=(double)atof(argv[14]);
	std::cout << "do_refine: " << do_refine<< std::endl;

	double table_z_filter_min=(double)atof(argv[15]);
	std::cout << "table_z_filter_min: " << table_z_filter_min<< std::endl;

	double table_z_filter_max=(double)atof(argv[16]);
	std::cout << "table_z_filter_max: " << table_z_filter_max<< std::endl;

	double z_filter_min=(double)atof(argv[17]);
	std::cout << "z_filter_min: " << z_filter_min<< std::endl;

	double z_filter_max=(double)atof(argv[18]);
	std::cout << "z_filter_max: " << z_filter_max<< std::endl;

	double plane_detection_voxel_size=(double)atof(argv[19]);
	std::cout << "plane_detection_voxel_size: " << plane_detection_voxel_size<< std::endl;

	double clustering_voxel_size=(double)atof(argv[20]);
	std::cout << "clustering_voxel_size: " << clustering_voxel_size<< std::endl;

	int inlier_threshold=atoi(argv[21]);
	std::cout << "inlier_threshold: " << inlier_threshold<< std::endl;

	std::vector<boost::shared_ptr<CylinderFitting> > cylinder_segmentators;

	// Gaussian Sphere Uniform
	std::vector<double> weights;
	std::vector<Eigen::Matrix<double, 3 ,1> > means;
	std::vector<Eigen::Matrix<double, 3 ,1> > std_devs;
	weights.push_back(1.0);
	Eigen::Matrix<double, 3 ,1> mean_eigen(0,0,0);
	means.push_back(mean_eigen);
	Eigen::Matrix<double, 3 ,1> std_dev_eigen(0.5,0.5,0.5);
	std_devs.push_back(std_dev_eigen);

	GaussianMixtureModel gmm(weights, means, std_devs);
	GaussianSphere gaussian_sphere(gmm,gaussian_sphere_points_num,orientation_accumulators_num);

	// CYLINDER HOUGH
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderFittingHough> (new CylinderFittingHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins, (double)min_radius, (double)max_radius,(double)accumulator_peak_threshold,CylinderFittingHough::NORMAL, false,true)));

	// Get point clouds
	PointClouds point_clouds(dataset_dir);

	//PLANE RANSAC
	PlaneFittingRansac plane_fitting_ransac(distance_threshold,cluster_tolerance,min_cluster_size,max_cluster_size,do_refine,table_z_filter_min,table_z_filter_max,z_filter_min, z_filter_max,plane_detection_voxel_size, clustering_voxel_size,inlier_threshold);

	while(1)
	{
		for(unsigned int i=0;i<point_clouds.file_names.size();++i)
		{
			pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>());
			point_cloud=point_clouds.loadPointCloud(dataset_dir+point_clouds.file_names[i]);

			/* PLANE FITTING */
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
			FittingData model_params=plane_fitting_ransac.fit(point_cloud);
			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
			/* END PLANE FITTING */

			auto plane_fitting_duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();

			/* VISUALIZE */
			model_params.visualize(point_cloud);
			/* END VISUALIZE */
			std::cout << "iteration " << (i+1) << " of " << point_clouds.file_names.size() << " table fitting time: " << plane_fitting_duration << " ms"<<  std::endl;
		}
	}

	return 0;
}

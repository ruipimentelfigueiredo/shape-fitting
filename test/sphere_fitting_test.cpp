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
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "sphere_fitting_hough.h"
#include "helpers.h"
#include <chrono>
using namespace std;
using namespace std::chrono;

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
	std::string ground_truth_dir=dataset_dir+"ground_truth/";
	std::string point_clouds_dir=dataset_dir+"point_clouds/";
	std::string output_dir=dataset_dir+"results/";

	// HOUGH PARAMETERS
	unsigned int radius_bins=atoi(argv[2]);
	std::cout << "radius_bins: " << radius_bins<< std::endl;

	unsigned int position_bins=atoi(argv[3]);
	std::cout << "position_bins: " << position_bins<< std::endl;

	// OUR HOUGH IMPLEMENTATION PARAMETERS
	unsigned int orientation_accumulators_num=atoi(argv[4]);
	std::cout << "orientation_accumulators_num: " << orientation_accumulators_num<< std::endl;

	unsigned int gaussian_sphere_points_num=atoi(argv[5]);
	std::cout << "gaussian_sphere_points_num: " << gaussian_sphere_points_num<< std::endl;
	
	float accumulator_peak_threshold=atof(argv[6]);
	std::cout << "accumulator_peak_threshold: " << accumulator_peak_threshold<< std::endl;
	
	float min_radius=atof(argv[7]);
	std::cout << "min_radius: " << min_radius<< std::endl;
		
	float max_radius=atof(argv[8]);
	std::cout << "max_radius: " << max_radius<< std::endl;
	
	// RANSAC PARAMETERS
	float normal_distance_weight=atof(argv[9]);
	std::cout << "normal_distance_weight: " << normal_distance_weight<< std::endl;

	unsigned int max_iterations=atoi(argv[10]);
	std::cout << "max_iterations: " << max_iterations << std::endl;

	float distance_threshold=atof(argv[11]);
	std::cout << "distance_threshold: " << distance_threshold << std::endl;

	bool do_refine=atoi(argv[12]);
	std::cout << "do_refine: " << do_refine << std::endl;


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

	// Gaussian Sphere Top Biased
	std::vector<double> weights_biased;
	std::vector<Eigen::Matrix<double, 3 ,1> > means_biased;
	std::vector<Eigen::Matrix<double, 3 ,1> > std_devs_biased;
	weights_biased.push_back(1.0);
	Eigen::Matrix<double, 3 ,1> mean_eigen_biased(0,0,1.0);
	means_biased.push_back(mean_eigen_biased);
	Eigen::Matrix<double, 3 ,1> std_dev_eigen_biased(0.1,0.1,0.1);
	std_devs_biased.push_back(std_dev_eigen_biased);

	GaussianMixtureModel gmm_biased(weights_biased, means_biased, std_devs_biased);
	GaussianSphere gaussian_sphere_biased(gmm_biased,gaussian_sphere_points_num,orientation_accumulators_num);

	std::vector<boost::shared_ptr<SphereFittingHough> > sphere_segmentators;

	// HOUGH NORMAL
	sphere_segmentators.push_back(boost::shared_ptr<SphereFittingHough> (new SphereFittingHough(gaussian_sphere,(unsigned int)position_bins,(unsigned int)radius_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold, false, false)));

	// HOUGH NORMAL (Soft-Voting)
	sphere_segmentators.push_back(boost::shared_ptr<SphereFittingHough> (new SphereFittingHough(gaussian_sphere,(unsigned int)position_bins,(unsigned int)radius_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold, false, true)));

	// HOUGH HYBRID BIASED
	//sphere_segmentators.push_back(boost::shared_ptr<SphereFittingHough> (new SphereFittingHough(gaussian_sphere_biased,(unsigned int)position_bins,(unsigned int)radius_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,false, false)));	

	// HOUGH HYBRID BIASED (Soft-Voting)
	//sphere_segmentators.push_back(boost::shared_ptr<SphereFittingHough> (new SphereFittingHough(gaussian_sphere_biased,(unsigned int)position_bins,(unsigned int)radius_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,false, true)));

	// Get ground truths
	GroundTruth ground_truths(ground_truth_dir);
	std::cout << "Number of ground truths:" << ground_truths.ground_truth.size() << std::endl;
	
	// Get point clouds
	PointClouds point_clouds(point_clouds_dir);
	std::cout << "Number of point_clouds:" << point_clouds.point_clouds.size() << std::endl;

	std::string detections_frame_id="world";
	std::string marker_detections_namespace_="detections";
	// Segment cylinder and store results
	std::vector<Eigen::VectorXf > detections;
	std::vector<float> position_errors;

	createDirectory(output_dir);

	for (unsigned int d=0;d < sphere_segmentators.size();++d)
	{
		std::fstream fs_radius;
		std::fstream fs_position;
		std::fstream fs_time;

		fs_radius.open (output_dir+"radius_noise_"           + std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app);
		fs_position.open (output_dir+"position_noise_"       + std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app);
		fs_time.open (output_dir+"time_noise_"               + std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app);

		for(unsigned int i=0;i<point_clouds.point_clouds.size();++i)
		{	
			// Ground truth iterations
			unsigned int iteration=i % ground_truths.ground_truth.size();

			Cylinder ground_truth=ground_truths.ground_truth[iteration];

			pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>());
			*point_cloud=*point_clouds.point_clouds[i];

			/* FIT */
    			high_resolution_clock::time_point t1 = high_resolution_clock::now();
			FittingData model_params=sphere_segmentators[d]->fit(point_clouds.point_clouds[i]);
    			high_resolution_clock::time_point t2 = high_resolution_clock::now();
			/* END FIT */

			/* STORE RESULTS */
			float radius_error=fabs(model_params.parameters[3]-ground_truth.radius);
			fs_radius << radius_error << " " << "\n";

			float position_error=(Eigen::Vector3f(model_params.parameters[0],model_params.parameters[1],model_params.parameters[2])-Eigen::Vector3f(ground_truth.position[0], ground_truth.position[1],ground_truth.position[2])).norm();
			fs_position << position_error << " " << "\n";

    			auto duration = duration_cast<milliseconds>( t2 - t1 ).count();
			fs_time << duration << " " << "\n";
			/* END STORE RESULTS */

			/* VISUALIZE */
			model_params.visualize(point_cloud);
			/* END VISUALIZE */

			std::cout << "iteration " << i << " of " << point_clouds.point_clouds.size() << " total time: " << duration << " ms"<<  std::endl;
		}

		fs_radius.close();
		fs_position.close();
		fs_time.close();
	}

	return (0);
}

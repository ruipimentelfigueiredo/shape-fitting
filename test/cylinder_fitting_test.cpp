#include "cylinder_segmentation_hough.h"
#include <ctime>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
#include "helpers.h"
#include <chrono>
using namespace std;
using namespace std::chrono;
#include <pcl/visualization/cloud_viewer.h>

 ////////////////////////////////////////////////////////////////////////////////////////////

/*namespace pcl
{

  namespace visualization
  {
    class PCL_EXPORTS PCLVisualizerCylinder : PCLVisualizer
    {
	public: 
	bool updateCylinder(const pcl::ModelCoefficients & coefficients, const std::string & id = "cylinder", int viewport = 0)	
	{
		// Check to see if this ID entry already exists (has it been already added to the visualizer?)
		ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
		if (am_it != shape_actor_map_->end ())
		{
			pcl::console::print_warn (stderr, "[addCylinder] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
			return (false);
		}

		if (coefficients.values.size () != 7)
		{
			PCL_WARN ("[addCylinder] Coefficients size does not match expected size (expected 7).\n");
			return (false);
		}

		vtkSmartPointer<vtkDataSet> data = createCylinder (coefficients);


		//////////////////////////////////////////////////////////////////////////
		// Get the actor pointer
		vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
		vtkAlgorithm *algo = actor->GetMapper ()->GetInput ()->GetProducerPort ()->GetProducer ();
		vtkCylinderSource *src = vtkCylinderSource::SafeDownCast (algo);

		src->SetHeight (coefficients[6]);
		src->SetRadius (coefficients[5]);
		src->Update ();
		//actor->GetProperty ()->SetColor (r, g, b);
		actor->Modified ();

		return (true);
	}
};*/


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
	
	float accumulator_peak_threshold=atof(argv[7]);
	std::cout << "accumulator_peak_threshold: " << accumulator_peak_threshold<< std::endl;
	
	float min_radius=atof(argv[8]);
	std::cout << "min_radius: " << min_radius<< std::endl;
		
	float max_radius=atof(argv[9]);
	std::cout << "max_radius: " << max_radius<< std::endl;
	
	// RANSAC PARAMETERS
	float normal_distance_weight=atof(argv[10]);
	std::cout << "normal_distance_weight: " << normal_distance_weight<< std::endl;

	unsigned int max_iterations=atoi(argv[11]);
	std::cout << "max_iterations: " << max_iterations << std::endl;

	float distance_threshold=atof(argv[12]);
	std::cout << "distance_threshold: " << distance_threshold << std::endl;

	bool do_refine=atoi(argv[13]);
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
	Eigen::Matrix<double, 3 ,1> std_dev_eigen_biased(0.5,0.5,0.5);
	std_devs_biased.push_back(std_dev_eigen_biased);

	GaussianMixtureModel gmm_biased(weights_biased, means_biased, std_devs_biased);
	GaussianSphere gaussian_sphere_biased(gmm_biased,gaussian_sphere_points_num,orientation_accumulators_num);

	// Gaussian Sphere Super Top Biased
	std::vector<double> weights_super_biased;
	std::vector<Eigen::Matrix<double, 3 ,1> > means_super_biased;
	std::vector<Eigen::Matrix<double, 3 ,1> > std_devs_super_biased;
	weights_super_biased.push_back(1.0);
	Eigen::Matrix<double, 3 ,1> mean_eigen_super_biased(0,0,1.0);
	means_super_biased.push_back(mean_eigen_super_biased);
	Eigen::Matrix<double, 3 ,1> std_dev_eigen_super_biased(0.05,0.05,0.05);
	std_devs_super_biased.push_back(std_dev_eigen_super_biased);

	GaussianMixtureModel gmm_super_biased(weights_super_biased, means_super_biased, std_devs_super_biased);
	GaussianSphere gaussian_sphere_super_biased(gmm_super_biased,gaussian_sphere_points_num,orientation_accumulators_num);

	// Gaussian Sphere Mega Top Biased
	/*std::vector<double> weights_mega_biased;
	std::vector<Eigen::Matrix<double, 3 ,1> > means_mega_biased;
	std::vector<Eigen::Matrix<double, 3 ,1> > std_devs_mega_biased;
	weights_mega_biased.push_back(1.0);
	Eigen::Matrix<double, 3 ,1> mean_eigen_mega_biased(0,0,1.0);
	means_mega_biased.push_back(mean_eigen_mega_biased);
	Eigen::Matrix<double, 3 ,1> std_dev_eigen_mega_biased(0.05,0.05,0.05);
	std_devs_mega_biased.push_back(std_dev_eigen_mega_biased);

	GaussianMixtureModel gmm_mega_biased(weights_mega_biased, means_mega_biased, std_devs_mega_biased);
	GaussianSphere gaussian_sphere_mega_biased(gmm_mega_biased,gaussian_sphere_points_num,orientation_accumulators_num);*/


	std::vector<boost::shared_ptr<CylinderSegmentation> > cylinder_segmentators;

	// HOUGH NORMAL
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::NORMAL, false, false)));

	// HOUGH NORMAL (soft-voting)
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::NORMAL, false, true)));


	// HOUGH HYBRID
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID,false, false)));

	// HOUGH HYBRID (soft-voting)
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID,false, true)));


	// HOUGH HYBRID BIASED
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere_biased,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID,false, false)));	

	// HOUGH HYBRID BIASED (soft-voting)
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere_biased,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID,false, true)));	


	// HOUGH HYBRID SUPER BIASED
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere_super_biased,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID,false, false)));

	// HOUGH HYBRID SUPER BIASED (soft-voting)
	cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere_super_biased,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID,false, true)));


	// HOUGH HYBRID MEGA BIASED
	//cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere_mega_biased,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID)));	

	//cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationRansac> (new CylinderSegmentationRansac(normal_distance_weight,(unsigned int)max_iterations,(unsigned int)distance_threshold,(unsigned int)min_radius,(float)max_radius, do_refine)));

	// HOUGH + RANSAC
	//cylinder_segmentators.push_back(boost::shared_ptr<CylinderFittingHoughRANSAC> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::NORMAL)));
	
	// HOUGH + PROSAC
	//cylinder_segmentators.push_back(boost::shared_ptr<CylinderFittingHoughProsacRANSAC> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::NORMAL)));

	
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

	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();*/
	for (unsigned int d=0;d < 1;++d)
	{
		//if(d==1||d==3||d==5||d==7) continue;
		std::fstream fs_orientation;
		std::fstream fs_radius;
		std::fstream fs_position;
		std::fstream fs_time;

		fs_orientation.open (output_dir+"orientation_noise_" + std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app);
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
			CylinderFitting model_params=cylinder_segmentators[d]->segment(point_clouds.point_clouds[i]);
    			high_resolution_clock::time_point t2 = high_resolution_clock::now();
    			auto duration = duration_cast<milliseconds>( t2 - t1 ).count();
			fs_time << duration << " " << "\n";
			std::cout << "iteration " << i << " of " << point_clouds.point_clouds.size() << " total time: " << duration << " ms"<<  std::endl;
			/* END FIT */

			/* STORE RESULTS */
			float orientation_error=acos(model_params.parameters.segment(3,3).dot(Eigen::Vector3f(ground_truth.direction[0],ground_truth.direction[1],ground_truth.direction[2])));
			if(orientation_error>M_PI/2.0)
				orientation_error=M_PI-orientation_error;
			fs_orientation << orientation_error << " " << "\n";

			float radius_error=fabs(model_params.parameters[6]-ground_truth.radius);
			fs_radius << radius_error << " " << "\n";

			float position_error=(model_params.parameters.head(2)-Eigen::Vector2f(ground_truth.position[0], ground_truth.position[1])).norm();
			fs_position << position_error << " " << "\n";
			/* END STORE RESULTS */

			/* VISUALIZE */
			/*pcl::ModelCoefficients model_coefficients;
			model_coefficients.values.resize (7);
			model_coefficients.values[0] = model_params.parameters[0];
			model_coefficients.values[1] = model_params.parameters[1];
			model_coefficients.values[2] = model_params.parameters[2];
			model_coefficients.values[3] = model_params.parameters[3];
			model_coefficients.values[4] = model_params.parameters[4];
			model_coefficients.values[5] = model_params.parameters[5];
			model_coefficients.values[6] = model_params.parameters[6];
			viewer->removeAllShapes();
			if(i==0)
			{
				viewer->addPointCloud<pcl::PointXYZ> (point_cloud, "sample cloud");
				viewer->addCylinder(model_coefficients,"ground truth");
			}
			else
			{
  				viewer->updatePointCloud<pcl::PointXYZ> (point_cloud, "sample cloud");
				viewer->addCylinder(model_coefficients,"ground truth");
			}
    			viewer->spinOnce(100);*/
			/* END VISUALIZE */
		}

		fs_orientation.close();
		fs_radius.close();
		fs_position.close();
		fs_time.close();
	}

	return (0);
}

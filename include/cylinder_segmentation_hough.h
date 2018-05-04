#ifndef CYLINDERSEGMENTATIONHOUGH_H
#define CYLINDERSEGMENTATIONHOUGH_H
#include "cylinder_segmentation.h"

class GaussianMixtureModel
{
	public:
		GaussianMixtureModel(const std::vector<double> & weights_, const std::vector<Eigen::Matrix<double, 3 ,1> > & means_, const std::vector<Eigen::Matrix<double, 3 ,1> > & std_devs_) : weights(weights_),means(means_),std_devs(std_devs_)
		{}
	std::vector<double> weights;
	std::vector<Eigen::Matrix<double, 3 ,1> > means;
	std::vector<Eigen::Matrix<double, 3 ,1> > std_devs;
};

class GaussianSphere
{

	public:
		GaussianSphere(const GaussianMixtureModel & gmm_,const unsigned int & gaussian_sphere_points_num_=900, const unsigned int & orientation_accumulators_num_=10) :
			gmm(gmm_),
 			gaussian_sphere_points_num(gaussian_sphere_points_num_),
			orientation_accumulators_num(orientation_accumulators_num_)
		{
			std::random_device rd{};
			std::mt19937 gen{rd()};

			iteration=0;
			// Create randomized structure
			// By randomly sampling a unitary sphere from a 3D, zero mean Gaussian distribution
			std::cout << "Creating gaussian spheres" << std::endl;
			for(unsigned int o=0;o<orientation_accumulators_num;++o)
			{
				std::cout << "  Creating gaussian sphere number " << o << std::endl;
				std::vector<Eigen::Vector3d> gaussian_sphere_points_;
				for(unsigned int g=0;g<gmm.weights.size();++g)
				{

					for(unsigned int i=0;i<gaussian_sphere_points_num*gmm.weights[g];++i)
					{
		                		std::normal_distribution<> d1{gmm.means[g][0],gmm.std_devs[g][0]};
		                		std::normal_distribution<> d2{gmm.means[g][1],gmm.std_devs[g][1]};
		                		std::normal_distribution<> d3{gmm.means[g][2],gmm.std_devs[g][2]};
						// Generate random patch on the sphere surface
						Eigen::Vector3d random_point(d1(gen),d2(gen),d3(gen));
						random_point.normalize();
						gaussian_sphere_points_.push_back(random_point);
					}
				}

				gaussian_sphere_points.push_back(gaussian_sphere_points_);
				std::cout << "  Done" << std::endl;
			}
			std::cout << "Done" << std::endl;
		};

	
	const std::vector<Eigen::Vector3d> & getGaussianSphere()
	{
		++iteration;
		//std::cout << iteration%orientation_accumulators_num<< std::endl;
		return gaussian_sphere_points[iteration%orientation_accumulators_num];
	}

	static int iteration;
	GaussianMixtureModel gmm;
	unsigned int gaussian_sphere_points_num;
	unsigned int orientation_accumulators_num;
	std::vector<std::vector<Eigen::Vector3d> > gaussian_sphere_points;
};

class CylinderSegmentationHough : public CylinderSegmentation
{
        public:
	static const unsigned int NORMAL=0;
	static const unsigned int CURVATURE=1;
	static const unsigned int HYBRID=2;
	// private attributes

	// Direction HT
	GaussianSphere gaussian_sphere;
	std::vector<double> cyl_direction_accum;


	// Circle GHT
	unsigned int angle_bins;

	double angle_step;
	unsigned int radius_bins;
	unsigned int position_bins;
	double r_step;
	double accumulator_peak_threshold;

	std::vector<std::vector<std::vector<unsigned int> > > cyl_circ_accum;

	unsigned int mode;


	// private methods
	Eigen::Vector3d findCylinderDirection(const NormalCloudT::ConstPtr & cloud_normals, const PointCloudT::ConstPtr & point_cloud_in_);
	Eigen::Matrix<double,5,1> findCylinderPositionRadius(const PointCloudT::ConstPtr & point_cloud_in_);

	public:
		CylinderSegmentationHough(const GaussianSphere & gaussian_sphere_, unsigned int angle_bins_=30,unsigned int radius_bins_=10,unsigned int position_bins_=10,double min_radius_=0.01,double max_radius_=0.1, double accumulator_peak_threshold_=0.2, unsigned int mode_=0, bool do_refine_=false);

		CylinderFitting segment(const PointCloudT::ConstPtr & point_cloud_in_);


};

#endif // CYLINDERSEGMENTATIONHOUGH_H

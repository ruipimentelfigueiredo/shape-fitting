#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include "helpers.h"


int main (int argc, char** argv)
{

	/* Program arguments check */
	if (argc < 6)
	{
	std::cout << "invalid number of arguments: dataset_dir iterations heights radii noise outliers height_sampling_rate angle_sampling_rate"<< std::endl;
	exit(-1);
	}

	std::random_device rd{};
	std::mt19937 gen{rd()};
	std::normal_distribution<> dis{0,1.0};

	std::string dataset_dir = std::string(argv[1]);
	std::cout << "dataset_dir: " << dataset_dir << std::endl;

	std::string ground_truth_dir = dataset_dir+"/ground_truth";
	std::string point_clouds_dir = dataset_dir+"/point_clouds";
	createDirectory(ground_truth_dir);
	createDirectory(point_clouds_dir);

	unsigned int iterations=atoi(argv[2]);
	std::cout << "iterations: " << iterations << std::endl;

	static std::string heights_ = std::string(argv[3]);
	std::cout << "heights: " << heights_ << std::endl;

	static std::string radii_ = std::string(argv[4]); 
	std::cout << "radii: " << radii_ << std::endl;

	static std::string noise_levels_ = std::string(argv[5]);
	std::cout << "noise: " << noise_levels_ << std::endl;

	static std::string outlier_levels_ = std::string(argv[6]);
	std::cout << "outliers: " << outlier_levels_ << std::endl;

	static unsigned int height_sampling_density = atoi(argv[7]);
	std::cout << "height_sampling_density: " << height_sampling_density<< std::endl;

	static unsigned int angle_sampling_density = atoi(argv[8]);
	std::cout << "angle_sampling_density: " << angle_sampling_density<< std::endl;

	std::vector<double> heights;
	std::vector<double> radii;
	std::vector<double> noise_levels; // percentage of object radius (std_dev)
	std::vector<double> outlier_levels; // percentage of object size (std_dev)

	double j;
	std::stringstream ss_(heights_);
	while (ss_ >> j)
	{
	heights.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	ss_=std::stringstream(radii_);
	while (ss_ >> j)
	{
	radii.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	ss_=std::stringstream(noise_levels_);
	while (ss_ >> j)
	{
	noise_levels.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	ss_=std::stringstream(outlier_levels_);
	while (ss_ >> j)
	{
	outlier_levels.push_back(j);

	if (ss_.peek() == ',')
		ss_.ignore();
	}

	std::vector<pcl::PointCloud<pcl::PointXYZ> > point_clouds;
	std::vector<Eigen::Matrix4d> transfs;


	// First, generate 200 point clouds with different radius, heights at random poses
		
	for(unsigned int i=0; i<iterations; ++i)
	{
		for(unsigned int o=0; o<outlier_levels.size();++o)
		{
		    	for(unsigned int h_=0; h_<heights.size();++h_)
		    	{
				for(unsigned int r_=0; r_<radii.size();++r_)
				{

					double height=heights[h_];
					double radius=radii[r_];
					// Generate cylinder according to parameters
					pcl::PointCloud<pcl::PointXYZ> cloud;
					cloud.header.frame_id="world";

					double angle_step=2.0*M_PI/angle_sampling_density;
					double height_step=fabs(height/height_sampling_density);

					for(unsigned int a=0; a < angle_sampling_density; ++a)
					{
						double x,y,z;
						x=cos(angle_step*a)*fabs(radius);
						y=sin(angle_step*a)*fabs(radius);
						for(unsigned int h=0; h < height_sampling_density; ++h)
						{
						    z=(double)height_step*h;
						    pcl::PointXYZ point(x,y,z);
						    cloud.push_back(point);
						}
					}


					// top / down surfaces
					unsigned int plane_samples=outlier_levels[o]*sqrt(height_sampling_density*angle_sampling_density);

					double plane_size=2*radius;
					double plane_min=0.0-radius;
					double plane_step=(double)plane_size/plane_samples;
					double z=0.0;
					for(unsigned int x_=0; x_<plane_samples;++x_)
					{
						double x=plane_min+x_*plane_step;
						for(unsigned int y_=0; y_<plane_samples;++y_)
						{

						    double y=plane_min+y_*plane_step;
						    if(sqrt(x*x+y*y)>radius) continue;

						    pcl::PointXYZ point(x,y,z);
						    cloud.push_back(pcl::PointXYZ(x,y,height));
						}
					}

					point_clouds.push_back(cloud);

					// Generate random orientation
					Eigen::Vector3d rot_dir(dis(gen),dis(gen),dis(gen));
					rot_dir.normalize();

					double angle;
					angle=dis(gen);

					rot_dir.normalize();
					rot_dir=Eigen::Vector3d::UnitZ();
					Eigen::Matrix3d rot;
					rot=Eigen::AngleAxisd(M_PI*angle,rot_dir);

					Eigen::Vector3d cyl_dir=Eigen::Vector3d::UnitZ();
					Eigen::Vector3d cyl_pos(0,0,0);
					Eigen::Matrix4d transf;
					transf.block(0,0,3,3)=rot;
					//transf.block(0,4,3,1)=cyl_pos;
					transfs.push_back(transf);


					unsigned int index=r_+h_*radii.size()+o*radii.size()*heights.size()+i*radii.size()*heights.size()*outlier_levels.size();

					std::stringstream ss;
					ss << dataset_dir << "/ground_truth/ground_truth_" << std::setfill('0') << std::setw(6) << index << ".txt";
					
					std::string ground_truth_file=ss.str();
					Cylinder ground_truth(radius,height,cyl_dir,cyl_pos, ground_truth_file);
					Cylinder test(ground_truth_file);
				}
			}
		}
	}

	for(unsigned int i=0; i<iterations; ++i)
	{
		for(unsigned int o=0; o<outlier_levels.size();++o)
		{
		    	for(unsigned int h_=0; h_<heights.size();++h_)
		    	{
				for(unsigned int r_=0; r_<radii.size();++r_)
				{
					// Then corrupt with diferent levels of noise
					for(unsigned int n=0; n<noise_levels.size(); ++n)
					{

						unsigned int index=r_+h_*radii.size()+o*radii.size()*heights.size()+i*radii.size()*heights.size()*outlier_levels.size();
						//std::cout << index << std::endl;
						pcl::PointCloud<pcl::PointXYZ> noisy_cloud;
						noisy_cloud=point_clouds[index];
						std::normal_distribution<> d{0,noise_levels[n]*radii[r_]};
						for(unsigned int p=0; p<noisy_cloud.size();++p)
						{
						    noisy_cloud.points[p].x+=d(gen);

						    noisy_cloud.points[p].y+=d(gen);

						    noisy_cloud.points[p].z+=d(gen);
						}

						//Transform point cloud
						pcl::PointCloud<pcl::PointXYZ> cloud_transf;
						// transfs[index]
						Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
						pcl::transformPointCloud (noisy_cloud, cloud_transf,transf);
						std::stringstream ss;
						ss << dataset_dir << "/point_clouds/cloud_" << std::setfill('0') << std::setw(6) << std::to_string(index) << "_noise_" << std::setfill('0') << std::setw(6) << n << ".pcd";
						pcl::io::savePCDFile(ss.str(), cloud_transf); // Binary format
					}
				}
			}
		}
	}
    
	return (0);
}


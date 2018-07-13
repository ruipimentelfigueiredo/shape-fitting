/*
 *  Copyright (C) 2018 Rui Pimentel de Figueiredo
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *      
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/


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

	std::string ground_truth_dir = dataset_dir+"/ground_truth/sphere/";
	std::string point_clouds_dir = dataset_dir+"/point_clouds/sphere/";
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


	static std::string occlusion_levels_ = std::string(argv[7]);
	std::cout << "occlusion_levels: " << occlusion_levels_ << std::endl;

	static unsigned int height_sampling_density = atoi(argv[8]);
	std::cout << "height_sampling_density: " << height_sampling_density<< std::endl;

	static unsigned int angle_sampling_density = atoi(argv[9]);
	std::cout << "angle_sampling_density: " << angle_sampling_density<< std::endl;

	std::vector<double> heights;
	std::vector<double> radii;
	std::vector<double> noise_levels; // percentage of object radius (std_dev)
	std::vector<double> outlier_levels; // percentage of object size (std_dev)
	std::vector<double> occlusion_levels; // percentage of object size (std_dev)

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


	ss_=std::stringstream(occlusion_levels_);
	while (ss_ >> j)
	{
	occlusion_levels.push_back(j);

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
			for(unsigned int occ=0; occ<occlusion_levels.size();++occ)
			{
				for(unsigned int r_=0; r_<radii.size();++r_)
				{
					double radius=radii[r_];
					// Generate sphere according to parameters
					pcl::PointCloud<pcl::PointXYZ> cloud;
					cloud.header.frame_id="world";

					double angle_step=2.0*M_PI/angle_sampling_density;

					unsigned int angle_sampling_density_final=round((1.0-occlusion_levels[occ])*angle_sampling_density);

					for(unsigned int a=0; a < angle_sampling_density_final; ++a)
					{
						double x,y,z;
						x=cos(angle_step*a)*fabs(radius);
						y=sin(angle_step*a)*fabs(radius);
						z=(double)10;
						pcl::PointXYZ point(x,y,z);
						cloud.push_back(point);
					}
					
					point_clouds.push_back(cloud);

					Eigen::Matrix3d rot;
					rot=Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitZ());

					Eigen::Vector3d sphere_pos(0,0,0);
					Eigen::Matrix4d transf;
					transf.block(0,0,3,3)=rot;
					//transf.block(0,4,3,1)=sphere_pos;
					transfs.push_back(transf);

					unsigned int index=r_+occ*radii.size()+o*radii.size()*occlusion_levels.size()+i*radii.size()*occlusion_levels.size()*outlier_levels.size();
					std::cout << index << std::endl;
					std::stringstream ss;
					ss << dataset_dir << ground_truth_dir << "ground_truth_" << std::setfill('0') << std::setw(6) << index << ".txt";
				
					std::string ground_truth_file=ss.str();
					Sphere ground_truth(radius, sphere_pos, ground_truth_file);
					Sphere test(ground_truth_file);

				}
			}
		}
	}

	for(unsigned int i=0; i<iterations; ++i)
	{
		for(unsigned int o=0; o<outlier_levels.size();++o)
		{
			for(unsigned int occ=0; occ<occlusion_levels.size();++occ)
			{
			    	for(unsigned int h_=0; h_<heights.size();++h_)
			    	{
					for(unsigned int r_=0; r_<radii.size();++r_)
					{
						// Then corrupt with diferent levels of noise
						for(unsigned int n=0; n<noise_levels.size(); ++n)
						{

							unsigned int index=r_+h_*radii.size()+occ*radii.size()*heights.size()+o*radii.size()*heights.size()*occlusion_levels.size()+i*radii.size()*heights.size()*occlusion_levels.size()*outlier_levels.size();

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
	}
    
	return (0);
}


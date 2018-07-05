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

#include <boost/filesystem.hpp>
#include <ctime>
bool createDirectory(std::string & path)
{
    boost::filesystem::path dir(path);
    try
    {
        if (boost::filesystem::create_directories(dir) == 0) {
            std::cout << "Created directory " << path << std::endl;
        } else {
            std::cout << "Did not create directory (probably exists) " << path << std::endl;
        }
    }
    catch (boost::filesystem::filesystem_error &e)
    {
        std::cerr << "Could not create directory " << path << std::endl;
        return false;
    }

    return true;
}

struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};
 
void readDirectory(const std::string& name, std::vector<std::string>& v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
}

class Cylinder
{

	public:
		Cylinder(const std::string & ground_truth_file)
		{
			std::fstream myfile(ground_truth_file, std::ios_base::in);
			myfile >> std::setprecision(5) 
			       >> radius 
			       >> height 
			       >> direction[0] 
			       >> direction[1]
			       >> direction[2] 
			       >> position[0] 
			       >> position[1] 
			       >> position[2];

			direction.normalize();
		};

		Cylinder(const double & radius_, const double & height_, const Eigen::Vector3d & direction_, const Eigen::Vector3d & position_, const std::string & ground_truth_file) :
			radius(radius_), 
			height(height_),
			direction(direction_), 
			position(position_)
		{
			direction.normalize();
                        std::ofstream myfile;
                        myfile.open(ground_truth_file);
                        myfile << std::setprecision(5)
                               << radius << " "
                               << height << " " 
                               << direction[0] << " "
                               << direction[1] << " "
                               << direction[2] << " "
                               << position[0] << " "
                               << position[1] << " "
                               << position[2];
		};
       		friend std::ostream &operator<<(std::ostream &os, const Cylinder& obj);


		double radius;
		double height;
		Eigen::Vector3d direction;
		Eigen::Vector3d position;
};

std::ostream &operator<<(std::ostream &os, const Cylinder& obj) { 
	return os << obj.radius << " " << obj.height << " " << obj.direction.transpose() << " " << obj.position.transpose() << std::endl;
};


class GroundTruth 
{
	public:
		GroundTruth(const std::string & dataset_path_)
		{
			std::vector<std::string> file_names;
			boost::filesystem::path p(dataset_path_);
			boost::filesystem::directory_iterator start(p);
			boost::filesystem::directory_iterator end;
			std::transform(start, end, std::back_inserter(file_names), path_leaf_string());
			std::sort(file_names.begin(), file_names.end());

			for(size_t f=0;f<file_names.size();++f)
			{
				ground_truth.push_back(dataset_path_+file_names[f]);
				//std::cout << dataset_path_+file_names[f] << std::endl;
			}		
		};

		std::vector<Cylinder> ground_truth;
};

class PointClouds 
{
	public:
		PointClouds(const std::string & dataset_path_)
		{
			std::vector<std::string> file_names;
			boost::filesystem::path p(dataset_path_);
			boost::filesystem::directory_iterator start(p);
			boost::filesystem::directory_iterator end;
			std::transform(start, end, std::back_inserter(file_names), path_leaf_string());
			std::sort(file_names.begin(), file_names.end());

			for(size_t f=0;f<file_names.size();++f)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

				if (pcl::io::loadPCDFile<pcl::PointXYZ> (dataset_path_+file_names[f], *cloud) == -1) //* load the file
				{
					exit(-1);
				}
				point_clouds.push_back(cloud);
				//std::cout << dataset_path_+file_names[f] << std::endl;
			}			
		};

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > point_clouds;
};





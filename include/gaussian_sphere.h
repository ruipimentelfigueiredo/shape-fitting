/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef GAUSSIANSPHERE_H
#define GAUSSIANSPHERE_H

#include "gaussian_mixture_model.h"

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

#endif // GAUSSIANSPHERE_H


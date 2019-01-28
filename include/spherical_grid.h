/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef SPHERICALGRID_H
#define SPHERICALGRID_H
#include "orientation_accumulator_space.h"
class SphericalGrid : public OrientationAccumulatorSpace
{
	public:
		SphericalGrid(const unsigned int & gaussian_sphere_points_num_=900, const unsigned int & orientation_accumulators_num_=10):
			OrientationAccumulatorSpace(gaussian_sphere_points_num_,orientation_accumulators_num_)
		{
			double theta_bins=sqrt(gaussian_sphere_points_num_);
			double phi_bins=sqrt(gaussian_sphere_points_num_);
			std::cout << "Creating gaussian spheres" << std::endl;
			for(unsigned int o=0;o<orientation_accumulators_num;++o)
			{
				std::cout << "  Creating gaussian sphere number " << o << std::endl;
				std::vector<Eigen::Vector3d> gaussian_sphere_points_;
				for(unsigned int theta_index=0; theta_index<theta_bins; ++theta_index)
				{
					double theta_step=(double)theta_index/theta_bins;
					double theta = 2 * M_PI * theta_step;
					for(unsigned int phi_index=0; phi_index<phi_bins; ++phi_index)
					{		
						double phi_step=(double)phi_index/phi_bins;

						double phi = M_PI * phi_step;
						double x = sin(phi) * cos(theta);
						double y = sin(phi) * sin(theta);
						double z = cos(phi);

						Eigen::Vector3d point(x,y,fabs(z));
						point.normalize();
						gaussian_sphere_points_.push_back(point);
					}
				}

				accumulator_space.push_back(gaussian_sphere_points_);
				//visualizeGaussianSphere(accumulator_space.size()-1);
				std::cout << "  Done" << std::endl;
			}
			std::cout << "Done" << std::endl;
		};

		const std::vector<Eigen::Vector3d> & getOrientationAccumulatorSpace()
		{
			return accumulator_space[0];
		}
};
#endif // SPHERICALGRID_H

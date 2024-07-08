#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <vector>
#include <limits>
#include <boost/container_hash/is_contiguous_range.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <utility>

template <typename PointT>
class NormalChangeRateEstimation : public pcl::Feature<PointT, pcl::Normal>
{
public:
	using Feature<PointT, pcl::Normal>::k_;
	using typename pcl::Feature<PointT, pcl::Normal>::PointCloudOut;
	using typename pcl::Feature<PointT, pcl::Normal>::PointCloud;
	using typename pcl::Feature<PointT, pcl::Normal>::PointCloudConstPtr;

	NormalChangeRateEstimation()
	{
		k_ = 10; // Default number of nearest neighbors
	}

	void setKSearch(int k)
	{
		k_ = k;
	}

protected:
	void computeFeature(PointCloudOut& output) override
	{
		for (size_t i = 0; i < this->indices_->size(); ++i)
		{
			std::vector<int> pointIdxNKNSearch(k_);
			std::vector<float> pointNKNSquaredDistance(k_);

			if (this->searchForNeighbors(i, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				double normalChangeRate = computeNormalChangeRate(pointIdxNKNSearch);
				output.points[i].curvature = normalChangeRate;
			}
			else
			{
				output.points[i].curvature = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}

private:
	double computeNormalChangeRate(const std::vector<int>& indices)
	{
		if (indices.size() < 4)
		{
			return (indices.size() == 3) ? 0.0 : std::numeric_limits<double>::quiet_NaN();
		}

		pcl::PointCloud<PointT>::Ptr neighborhood(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud(*this->input_, indices, *neighborhood);

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*neighborhood, centroid);

		Eigen::Matrix3f covariance;
		pcl::computeCovarianceMatrixNormalized(*neighborhood, centroid, covariance);

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_solver(covariance);
		if (eig_solver.info() != Eigen::Success)
		{
			return std::numeric_limits<double>::quiet_NaN();
		}
		Eigen::Vector3f eigenvalues = eig_solver.eigenvalues();

		double sum = eigenvalues[0] + eigenvalues[1] + eigenvalues[2];
		if (sum < std::numeric_limits<double>::epsilon())
		{
			return std::numeric_limits<double>::quiet_NaN();
		}
		double eMin = std::min(eigenvalues[0], std::min(eigenvalues[1], eigenvalues[2]));
		return eMin / sum;
	}
};

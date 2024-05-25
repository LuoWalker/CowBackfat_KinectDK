#include "GetFeature.h"
#include <vector>

float vectorAngle(const Eigen::Vector3f& x, const Eigen::Vector3f& y) {
	float Lx = x.norm();
	float Ly = y.norm();
	float cosAngle = x.dot(y) / (Lx * Ly);
	float angle = std::acos(cosAngle);
	return angle * 180 / M_PI;
}
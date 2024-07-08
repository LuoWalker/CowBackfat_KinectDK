#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>
#include <unsupported/Eigen/Polynomials>

//曲率计算结构体
typedef struct CURVATURE {
	int index;
	float curvature;
}CURVATURE;


// 计算法矢量夹角A
double calculateAngleA(const Eigen::Vector3f& normal, const std::vector<Eigen::Vector3f>& neighbor_normals);

// 计算曲率均方差σ_D
double calculateCurvatureStdDev(const std::vector<double>& curvatures, double mean_curvature);

// 二次曲线拟合函数
void fitQuadraticCurve(const std::vector<Eigen::Vector2d>& points, double& a, double& b, double& c);

// 二次曲线求极值点及其导数
void findExtrema(double a, double b, double c, double& extremum_x, double& extremum_y);

// 多项式拟合函数
void fitPolynomial(const std::vector<Eigen::Vector2d>& points, int degree, Eigen::VectorXd& coefficients);

// 计算多项式的导数
Eigen::VectorXd polynomialDerivative(const Eigen::VectorXd& coefficients);

// 使用牛顿法求解导数为零的点（极值点），这里用作根的个数判断
int countExtrema(const Eigen::VectorXd& derivative, double tol = 1e-6, int max_iter = 1000);

// 使用牛顿法求解导数为零的点（极值点），并返回极值点的坐标
std::vector<double> findExtrema(const Eigen::VectorXd& derivative, double start, double end, int direction, double tol = 1e-6, int max_iter = 1000);

// 计算多项式在某点的二阶导数
double evaluateSecondDerivative(const Eigen::VectorXd& coefficients, double x);
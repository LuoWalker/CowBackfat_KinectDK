#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>
#include <unsupported/Eigen/Polynomials>

//���ʼ���ṹ��
typedef struct CURVATURE {
	int index;
	float curvature;
}CURVATURE;


// ���㷨ʸ���н�A
double calculateAngleA(const Eigen::Vector3f& normal, const std::vector<Eigen::Vector3f>& neighbor_normals);

// �������ʾ������_D
double calculateCurvatureStdDev(const std::vector<double>& curvatures, double mean_curvature);

// ����������Ϻ���
void fitQuadraticCurve(const std::vector<Eigen::Vector2d>& points, double& a, double& b, double& c);

// ����������ֵ�㼰�䵼��
void findExtrema(double a, double b, double c, double& extremum_x, double& extremum_y);

// ����ʽ��Ϻ���
void fitPolynomial(const std::vector<Eigen::Vector2d>& points, int degree, Eigen::VectorXd& coefficients);

// �������ʽ�ĵ���
Eigen::VectorXd polynomialDerivative(const Eigen::VectorXd& coefficients);

// ʹ��ţ�ٷ���⵼��Ϊ��ĵ㣨��ֵ�㣩�������������ĸ����ж�
int countExtrema(const Eigen::VectorXd& derivative, double tol = 1e-6, int max_iter = 1000);

// ʹ��ţ�ٷ���⵼��Ϊ��ĵ㣨��ֵ�㣩�������ؼ�ֵ�������
std::vector<double> findExtrema(const Eigen::VectorXd& derivative, double start, double end, int direction, double tol = 1e-6, int max_iter = 1000);

// �������ʽ��ĳ��Ķ��׵���
double evaluateSecondDerivative(const Eigen::VectorXd& coefficients, double x);
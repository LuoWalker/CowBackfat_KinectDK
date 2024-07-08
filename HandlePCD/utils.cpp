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
double calculateAngleA(const Eigen::Vector3f& normal, const std::vector<Eigen::Vector3f>& neighbor_normals) {
	double sum_dot_products = 0.0;
	for (const auto& neighbor_normal : neighbor_normals) {
		sum_dot_products += std::abs(normal.dot(neighbor_normal));
	}
	return sum_dot_products / neighbor_normals.size();
}

// 计算曲率均方差σ_D
double calculateCurvatureStdDev(const std::vector<double>& curvatures, double mean_curvature) {
	double sum = 0.0;
	for (const auto& curvature : curvatures) {
		sum += (curvature - mean_curvature) * (curvature - mean_curvature);
	}
	return std::sqrt(sum / curvatures.size());
}

// 二次曲线拟合函数
void fitQuadraticCurve(const std::vector<Eigen::Vector2d>& points, double& a, double& b, double& c) {
	int n = points.size();
	Eigen::MatrixXd A(n, 3);
	Eigen::VectorXd B(n);

	for (int i = 0; i < n; ++i) {
		double x = points[i].x();
		double y = points[i].y();
		A(i, 0) = x * x;
		A(i, 1) = x;
		A(i, 2) = 1;
		B(i) = y;
	}

	Eigen::VectorXd result = A.colPivHouseholderQr().solve(B);
	a = result(0);
	b = result(1);
	c = result(2);
}

// 二次曲线求极值点及其导数
void findExtrema(double a, double b, double c, double& extremum_x, double& extremum_y) {
	extremum_x = -b / (2 * a);
	extremum_y = a * extremum_x * extremum_x + b * extremum_x + c;
}

// 多项式拟合函数
void fitPolynomial(const std::vector<Eigen::Vector2d>& points, int degree, Eigen::VectorXd& coefficients) {
	int n = points.size();
	Eigen::MatrixXd A(n, degree + 1);
	Eigen::VectorXd B(n);

	for (int i = 0; i < n; ++i) {
		double y = points[i].x();
		double z = points[i].y();
		for (int j = 0; j <= degree; ++j) {
			A(i, j) = std::pow(y, j);
		}
		B(i) = z;
	}

	coefficients = A.colPivHouseholderQr().solve(B);
}

// 计算多项式的导数
Eigen::VectorXd polynomialDerivative(const Eigen::VectorXd& coefficients) {
	int degree = coefficients.size() - 1;
	Eigen::VectorXd derivative(degree);
	for (int i = 1; i <= degree; ++i) {
		derivative[i - 1] = coefficients[i] * i;
	}
	return derivative;
}

// 计算多项式在某点的二阶导数
double evaluateSecondDerivative(const Eigen::VectorXd& coefficients, double x) {
	int degree = coefficients.size() - 1;
	double second_derivative = 0.0;
	for (int i = 2; i <= degree; ++i) {
		second_derivative += coefficients[i] * i * (i - 1) * std::pow(x, i - 2);
	}
	return second_derivative;
}

// 使用牛顿法求解导数为零的点（极值点），并返回极值点的坐标
std::vector<double> findExtrema(const Eigen::VectorXd& derivative, double start, double end, int direction, double tol = 1e-6, int max_iter = 1000) {
	int degree = derivative.size();
	std::vector<double> extrema;

	// 生成初始猜测值（可以根据具体情况调整）
	for (double initial_guess = start; initial_guess <= end; initial_guess += 1.0) {
		double x = initial_guess;
		for (int i = 0; i < max_iter; ++i) {
			double fx = 0.0;
			double dfx = 0.0;
			for (int j = 0; j < degree; ++j) {
				fx += derivative[j] * std::pow(x, j);
				if (j > 0) {
					dfx += derivative[j] * j * std::pow(x, j - 1);
				}
			}
			if (std::abs(dfx) < tol) break; // 避免除以0
			double x_new = x - fx / dfx;
			if (std::abs(x_new - x) < tol) {
				extrema.push_back(x_new);
				break;
			}
			x = x_new;
		}
	}

	// 去重，合并非常接近的极值点
	if (direction == 0) {
		std::sort(extrema.begin(), extrema.end());
	}
	else if (direction == 1) {
		std::sort(extrema.begin(), extrema.end(), [](double a, double b) {
			return a > b;
			});
	}
	auto last = std::unique(extrema.begin(), extrema.end(), [tol](double a, double b) {
		return std::abs(a - b) < tol;
		});
	extrema.erase(last, extrema.end());

	// 去除边界外的极值点
	last = std::remove_if(extrema.begin(), extrema.end(), [start, end](double x) {
		return x < start || x > end;
		});
	extrema.erase(last, extrema.end());

	return extrema;
}

// 使用牛顿法求解导数为零的点（极值点），这里用作根的个数判断
int countExtrema(const Eigen::VectorXd& derivative, double tol = 1e-6, int max_iter = 1000) {
	int degree = derivative.size() - 1;
	std::vector<double> roots;
	Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
	solver.compute(derivative);
	solver.realRoots(roots);

	return roots.size();
}
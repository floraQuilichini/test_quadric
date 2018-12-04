#pragma once
#ifndef DEF_Quadric
#define DEF_Quadric

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <utility>
#include "point.h"


class Quadric
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Quadric(Eigen::Matrix3d& q1, Eigen::Matrix3d& q2);
	Quadric(Eigen::Matrix3d& q1);
	Quadric();
	void compute_eigen_vectors_and_values();
	Point compute_opt_pos();
	std::tuple<double, double, double> compute_ellipse_parameters();
	std::pair<Eigen::Vector2d, double> get_first_vector_and_value();
	std::pair<Eigen::Vector2d, double> get_second_vector_and_value();
	Eigen::Matrix3d get_quadric_value();
	void set_quadric_value(Eigen::Matrix3d& q);
	Eigen::Matrix3d split_quadric_value(double alpha, double beta, Quadric& Q_neighboor);

private:
	Eigen::Matrix3d _quadric;
	Eigen::Vector2d _eigen_v1;
	Eigen::Vector2d _eigen_v2;
	double _eigen_value1;
	double _eigen_value2;

};

#endif



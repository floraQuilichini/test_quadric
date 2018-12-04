#pragma once
#ifndef DEF_Point
#define DEF_Point

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 

class Point
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Point(Eigen::Vector2d& p);
	Point(double x, double y);
	Point();

	Eigen::Vector2d get_value();
	double get_x();
	double get_y();

	void set_value(Eigen::Vector2d& p);

private:

	Eigen::Vector2d _point;
};

#endif

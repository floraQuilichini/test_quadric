#pragma once
#ifndef PRED_ERR
#define PRED_ERR

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <vector>
#include "point.h"

class PredErr
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		PredErr(Point& true_pos, Point& rec_pos);
	Eigen::Vector2d get_err();
	double get_err2();
private:
	Eigen::Vector2d _err;
	double _err2;
	
};

#endif


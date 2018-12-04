#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 

#include"prediction_error.h"
#include "point.h"

PredErr::PredErr(Point& true_pos, Point& rec_pos): _err(Eigen::Vector2d(true_pos.get_x() - rec_pos.get_x(), true_pos.get_y() - rec_pos.get_y())), _err2(sqrt(_err.transpose()*_err)) {}
Eigen::Vector2d PredErr::get_err() { return _err; }
double PredErr::get_err2() { return _err2; }

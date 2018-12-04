#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 

#include "point.h"

Point::Point(Eigen::Vector2d& p) : _point(p) {}
Point::Point(double x, double y) : _point(Eigen::Vector2d(x, y)) {}
Point::Point(): _point(Eigen::Vector2d()) {}
Eigen::Vector2d Point::get_value() { return _point; }
double Point::get_x() { return _point[0]; }
double Point::get_y() { return _point[1]; }

void Point::set_value(Eigen::Vector2d& p) {	_point = p;}

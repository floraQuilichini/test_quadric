#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 

#include "halfedge.h"
#include "vertex.h"
#include "point.h"

Halfedge::Halfedge(Vertex* v1, Vertex* v2): _v1(v1), _v2(v2){}
double Halfedge::compute_length()
{
	return sqrt(std::pow(_v1->get_point().get_x() - _v2->get_point().get_x(), 2.0) + std::pow(_v1->get_point().get_x() - _v2->get_point().get_x(), 2.0));
}
void Halfedge::compute_QEM_cost()
{
	Quadric newQ(_v1->get_quadric().get_quadric_value(), _v2->get_quadric().get_quadric_value());
	Point opt_pos = newQ.compute_opt_pos();

	Eigen::Vector3d v_opt(opt_pos.get_x(), opt_pos.get_y(), 1.0 );
	_cost = v_opt.transpose() * newQ.get_quadric_value() * v_opt;
	
}
void Halfedge::compute_angle_cost()
{
	Vertex* ptr_v = _v1;
	_cost = 0;
	for (int i = 0; i < 2; i++, ptr_v = _v2)
	{
		double p_x = ptr_v->get_point().get_x();
		double p_y = ptr_v->get_point().get_y();
		double r = sqrt(ptr_v->get_point().get_value().transpose() * ptr_v->get_point().get_value());
		double cos_theta = p_x / r;
		double sin_theta = p_y / r;

		double theta = std::acos(cos_theta); // le acos renvoie des valeurs entre [0, pi]

		if (sin(theta)*sin_theta < 0.0)
			_cost += 2.0*3.1416 - theta;
		else
			_cost += theta;
	}
	
	_cost /= 2.0;
	
}
void Halfedge::set_v1(Vertex* v1) { _v1 = v1; }
void Halfedge::set_v2(Vertex* v2) { _v2 = v2; }
Vertex* Halfedge::get_v1() { return _v1; }
Vertex* Halfedge::get_v2() { return _v2; }
double Halfedge::get_cost() { return _cost; }
#pragma once
#ifndef DEF_Halfedge
#define DEF_Halfedge

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 

class Vertex;

class Halfedge
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Halfedge(Vertex* v1, Vertex* v2);
	double compute_length();
	void compute_QEM_cost();
	void compute_angle_cost();
	void set_v1(Vertex* v1);
	void set_v2(Vertex* v2);
	Vertex* get_v1();
	Vertex* get_v2();
	double get_cost();

private:
	Vertex* _v1;
	Vertex* _v2;
	double _cost;
};

#endif


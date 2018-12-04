#pragma once
#ifndef DEF_Vertex
#define DEF_Vertex

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <vector>
#include "point.h"
#include "halfedge.h"
#include "quadric.h"
#include "node.h"

class Vertex
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vertex(Point& point);
	Eigen::Matrix3d get_quadric_value();
	Point get_point();
	void set_quadric(Quadric& quadric);
	void set_point(Point& p);
	void set_prev_hedge(Halfedge* p_he);
	void set_next_hedge(Halfedge* n_he);
	void set_vertex_node(Node* node);
	Halfedge* get_prev_hedge();
	Halfedge* get_next_hedge();
	Quadric get_quadric();
	Node* get_vertex_node();
	~Vertex();

private:

	Point _point_value;
	Quadric _quadric;
	Halfedge* _p_hedge = NULL;
	Halfedge* _n_hedge = NULL;
	Node* _node = NULL;
};

#endif

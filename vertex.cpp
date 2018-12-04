#include "point.h"
#include "vertex.h"
#include "halfedge.h"
#include "quadric.h"
#include "node.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 


Vertex::Vertex(Point& pp): _point_value(pp) {}
Eigen::Matrix3d Vertex::get_quadric_value() { return _quadric.get_quadric_value(); }
Point Vertex::get_point() { return _point_value; }
void Vertex::set_prev_hedge(Halfedge* p_he) { _p_hedge = p_he; }
void Vertex::set_next_hedge(Halfedge* n_he) { _n_hedge = n_he; }
void Vertex::set_quadric(Quadric& q) { _quadric = q; }
void Vertex::set_point(Point& p) { _point_value = p; }
void Vertex::set_vertex_node(Node* node) { _node = node; }
Halfedge* Vertex::get_prev_hedge() { return _p_hedge; }
Halfedge* Vertex::get_next_hedge() { return _n_hedge; }
Quadric Vertex::get_quadric() { return _quadric; }
Node* Vertex::get_vertex_node() { return _node; }
Vertex::~Vertex() 
{ 
	delete _n_hedge;
	delete _p_hedge;
	std::cout << "delete vertex " << std::endl;
}

#pragma once
#ifndef DEF_Mesh
#define DEF_Mesh

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <list>
#include "halfedge.h"
#include "vertex.h"
#include "point.h"
#include "collapseMemory.h"

class Mesh
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	void add_vertex(Point& point);
	Point collapse_edge(Halfedge* hedge);
	std::pair<Vertex*, Vertex*> split_vertex(std::list<Vertex*>::iterator current_it);
	void Mesh::set_hedges();
	void Mesh::init_quadrics();
	Eigen::Vector3d get_line_eq(Vertex* v1, Vertex* v2);
	Eigen::Matrix3d compute_K(Halfedge* he);
	void collapse_batch(CollapseMemory& memory);
	void set_hedges_cost();
	const std::type_info& get_attribute_type();
	int get_size();
	std::list<Vertex*>::iterator get_input_iterator_begin();
	std::list<Vertex*>::iterator get_input_iterator_end();
	void set_alpha_and_beta(double alpha, double beta);
	void split_batch(CollapseMemory& memory);
	void collapse_simulation();
	void split_simulation();
	std::vector<Point>::iterator get_opt_pos_sim_begin();
	std::vector<Point>::iterator get_opt_pos_sim_end();
	std::vector<std::tuple<double, double, double>>::iterator get_ellipse_params_begin();
	std::vector<std::tuple<double, double, double>>::iterator get_ellipse_params_end();
	int get_vec_err2_size();
	double print_err2(int indice);
	~Mesh();

private:
	std::list<Vertex*> _mesh;
	std::vector<Point, Eigen::aligned_allocator<Point>> _opt_pos_sim;
	std::vector<std::tuple<double, double, double>> _ellipse_params;
	std::vector<double> _err2_vec;
	double _alpha;
	double _beta;
};

#endif



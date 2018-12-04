#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <iostream>
#include <iterator>

#include "halfedge.h"
#include "vertex.h"
#include "point.h"
#include "quadric.h"
#include "mesh.h"
#include "queue.h"
#include "collapseMemory.h"
#include "node.h"
#include "prediction_error.h"

void Mesh::add_vertex(Point& point)
{
	Vertex* vertex = new Vertex(point);
	_mesh.push_back(vertex);
	
}
Point Mesh::collapse_edge(Halfedge* hedge)
{
	// vertex to remove = v1
	Vertex* v1 = hedge->get_v1();
	Vertex* v2 = hedge->get_v2();
	auto search = std::find(_mesh.begin(), _mesh.end(), v1);
	if (search == _mesh.end())
		std::cout << "vertex is not in mesh" << std::endl;
	else
	{
		// get the next vertex after the vertex erased to connect it to the previous vertex of the vertex erased
		Vertex* prev_v2 = v1->get_prev_hedge()->get_v2();
		v2->get_prev_hedge()->set_v2(prev_v2);
		prev_v2->get_next_hedge()->set_v2(v2);
		// set new quadric associated to that vertex
		Quadric newQ = Quadric(v1->get_quadric().get_quadric_value(), v2->get_quadric().get_quadric_value());
		v2->set_quadric(newQ);

		// put v2 in optimal position
		v2->set_point(v2->get_quadric().compute_opt_pos());

		//set new edge cost
		v2->get_prev_hedge()->compute_QEM_cost();
		prev_v2->get_next_hedge()->compute_QEM_cost();

		// delete vertex
		delete v1;
		v1 = NULL;
		_mesh.erase(search);

		return v2->get_point();

	}
}

std::pair<Vertex*, Vertex*> Mesh::split_vertex(std::list<Vertex*>::iterator current_it)
{
	Vertex* vertex = *current_it;

	// split quadric
	Eigen::Matrix3d Qnn = vertex->get_quadric().split_quadric_value(_alpha, _beta, vertex->get_next_hedge()->get_v2()->get_quadric());
	Eigen::Matrix3d Qnp = vertex->get_quadric().split_quadric_value(_alpha, _beta, vertex->get_prev_hedge()->get_v2()->get_quadric());

	// create new Quadric and set the older one
	Quadric Qn_p = Quadric(Qnp);
	vertex->get_quadric().set_quadric_value(Qnn);

	// create a new point and move the older one
	Point Pn_p = Qn_p.compute_opt_pos();
	vertex->set_point(vertex->get_quadric().compute_opt_pos());

	// set new vertex
	Vertex* vn_p = new Vertex(Pn_p);
	vn_p->set_quadric(Qn_p);

	// create new hedges
	Halfedge* hp = new Halfedge(vn_p, vertex->get_prev_hedge()->get_v2());
	Halfedge* hn = new Halfedge(vn_p, vertex);

	// connect new hedges to old ones
	vertex->get_prev_hedge()->set_v2(vn_p);
	vn_p->set_next_hedge(hn);
	vn_p->set_prev_hedge(hp);
	hp->get_v2()->get_next_hedge()->set_v2(vn_p);

	//push vertex in mesh
	if (current_it != _mesh.begin())
		_mesh.insert(current_it, vn_p);
	else
		_mesh.push_front(vn_p);

	return std::make_pair(vn_p, vertex);

}

void Mesh::set_hedges()
{
	if (_mesh.empty())
		std::cout << "mesh is empty " << std::endl;
	else if (_mesh.size() == 1)
		std::cout << "mesh contains one vertex " << std::endl;
	else if (_mesh.size() == 2)
	{
		Halfedge* next_he = new Halfedge(*_mesh.begin(), *std::next(_mesh.begin(), 1));
		(*_mesh.begin())->set_next_hedge(next_he);
		Halfedge* prev_he = new Halfedge(*std::next(_mesh.begin(), 1), *_mesh.begin());
		(*std::next(_mesh.begin(), 1))->set_prev_hedge(prev_he);
	}
	else
	{
		auto it = std::next(_mesh.begin(), 1);
		while (*it != _mesh.back())
		{
			Halfedge* next_he = new Halfedge(*it, *std::next(it, 1));
			Halfedge* prev_he = new Halfedge(*it, *std::prev(it, 1));
			(*it)->set_next_hedge(next_he);
			(*it)->set_prev_hedge(prev_he);


			++it;
		}
		// cas particuliers debut et fin
		Halfedge* next_he = new Halfedge(*it, _mesh.front());
		Halfedge* prev_he = new Halfedge(*it, *std::prev(it, 1));
		(*it)->set_next_hedge(next_he);
		(*it)->set_prev_hedge(prev_he);

		next_he = new Halfedge(_mesh.front(), *std::next(_mesh.begin(), 1));
		prev_he = new Halfedge(_mesh.front(), *it);
		_mesh.front()->set_next_hedge(next_he);
		_mesh.front()->set_prev_hedge(prev_he);
	}
}

void Mesh::init_quadrics()
{
	if (_mesh.empty())
		std::cout << "mesh is empty" << std::endl;
	else
	{
		for (auto it = _mesh.begin(); it != _mesh.end(); it++)
		{
			Quadric q_init = Quadric(compute_K((*it)->get_next_hedge()), compute_K((*it)->get_prev_hedge()));
			(*it)->set_quadric(q_init);
		}
	}

}

Eigen::Vector3d Mesh::get_line_eq(Vertex* v1, Vertex* v2)
{
	Eigen::Vector2d n_v1v2;

	n_v1v2 << v2->get_point().get_y() - v1->get_point().get_y(), v1->get_point().get_x() - v2->get_point().get_x();
	n_v1v2 /= sqrt(n_v1v2.transpose() * n_v1v2);

	double d = -n_v1v2[1] * v2->get_point().get_y() - v2->get_point().get_x() * n_v1v2[0];

	return Eigen::Vector3d(n_v1v2[0], n_v1v2[1], d);
}

Eigen::Matrix3d Mesh::compute_K(Halfedge* e)
{
	Eigen::Vector3d line_coeff = get_line_eq(e->get_v1(), e->get_v2());

	Eigen::Matrix3d K = line_coeff*line_coeff.transpose();
	return K / e->compute_length();
}
void Mesh::set_hedges_cost()
{
	auto it = _mesh.begin();
	for (; it != _mesh.end(); it++)
	{
		(*it)->get_next_hedge()->compute_QEM_cost();
		(*it)->get_prev_hedge()->compute_QEM_cost();
	}

}

void Mesh::collapse_batch(CollapseMemory& memory)
{
	std::list<Halfedge*> forbidden_hedges;

	// fill queue
	Queue queue;
	queue.fill_queue(_mesh.begin(), _mesh.end());

	memory.fill_next_row(_mesh.begin(), _mesh.end());

	while (! queue.is_empty() && _mesh.size() > 3)
	{
		// on sort la première arête
		Halfedge* he = queue.top_hedge();
		queue.pop_element();

		if (std::find(forbidden_hedges.begin(), forbidden_hedges.end(), he) == forbidden_hedges.end()) // if edge is not forbidden
		{
			//fill vector forbidden edges
			forbidden_hedges.push_back(he->get_v2()->get_next_hedge());
			forbidden_hedges.push_back(he->get_v1()->get_prev_hedge()->get_v2()->get_next_hedge());

			// keep memory 
			Node* new_node = memory.set_new_node(he->get_v2(), he->get_v1());

			// collapse edge
			Point opt_pos = collapse_edge(he);

			// update memory position
			new_node->set_pos(opt_pos);

		}

	}
	
	memory.increase_ind_row();
}

void Mesh::collapse_simulation()
{
	_opt_pos_sim.clear();
	_ellipse_params.clear();
	for (auto it = _mesh.begin(); it != _mesh.end(); it++)
	{
		Quadric new_quadric((*it)->get_quadric().get_quadric_value(), (*it)->get_next_hedge()->get_v2()->get_quadric().get_quadric_value());
		Point opt_pos = new_quadric.compute_opt_pos();
		_opt_pos_sim.push_back(opt_pos);

		std::tuple<double, double, double> ellipse_param = new_quadric.compute_ellipse_parameters();
		_ellipse_params.push_back(ellipse_param);
	}
}

const std::type_info& Mesh::get_attribute_type()
{
	return typeid(_mesh);
}

int Mesh::get_size() { return _mesh.size(); }

std::list<Vertex*>::iterator Mesh::get_input_iterator_begin() { return _mesh.begin(); }
std::list<Vertex*>::iterator Mesh::get_input_iterator_end() { return _mesh.end(); }

void Mesh::set_alpha_and_beta(double alpha, double beta)
{
	_alpha = alpha;
	_beta = beta;
}

void Mesh::split_batch(CollapseMemory& memory)
{

	for (auto it = _mesh.begin(); it != _mesh.end(); it++)
	{
		//Node* node = memory.search_parent_node(*it);
		Node* node = (*it)->get_vertex_node();
		if (node->has_two_parents())
		{
			std::pair<Vertex*, Vertex*> new_vertices = split_vertex(it);
			node->get_father()->set_vertex(new_vertices.first);
			node->get_mother()->set_vertex(node->get_vertex());

			(new_vertices.first)->set_vertex_node(node->get_father());
			(*it)->set_vertex_node(node->get_mother());

			//set_err_pred
			PredErr pred_err_p(node->get_father()->get_pos(), (new_vertices.first)->get_point());
			_err2_vec.push_back(pred_err_p.get_err2());
			PredErr pred_err_n(node->get_mother()->get_pos(), (new_vertices.second)->get_point());
			_err2_vec.push_back(pred_err_n.get_err2());
		}

	}

	memory.decrease_ind_row();

}

void Mesh::split_simulation()
{
	_opt_pos_sim.clear();
	_ellipse_params.clear();
	for (auto it = _mesh.begin(); it != _mesh.end(); it++)
	{
		Eigen::Matrix3d old_q = _beta*((*it)->get_quadric().get_quadric_value());
		Eigen::Matrix3d old_p_q = _alpha*((*it)->get_prev_hedge()->get_v2()->get_quadric().get_quadric_value());
		Eigen::Matrix3d old_n_q = _alpha*((*it)->get_next_hedge()->get_v2()->get_quadric().get_quadric_value());

		Quadric new_q1(old_q, old_p_q);
		Quadric new_q2(old_q, old_n_q);
		Point opt_pos1 = new_q1.compute_opt_pos();
		Point opt_pos2 = new_q2.compute_opt_pos();
		_opt_pos_sim.push_back(opt_pos1);
		_opt_pos_sim.push_back(opt_pos2);

		std::tuple<double, double, double> ellipse_param1 = new_q1.compute_ellipse_parameters();
		std::tuple<double, double, double> ellipse_param2 = new_q2.compute_ellipse_parameters();
		_ellipse_params.push_back(ellipse_param1);
		_ellipse_params.push_back(ellipse_param2);
	}

}

std::vector<Point>::iterator Mesh::get_opt_pos_sim_begin() { return _opt_pos_sim.begin(); }
std::vector<Point>::iterator Mesh::get_opt_pos_sim_end() { return _opt_pos_sim.end(); }
std::vector<std::tuple<double, double, double>>::iterator Mesh::get_ellipse_params_begin() { return _ellipse_params.begin(); }
std::vector<std::tuple<double, double, double>>::iterator Mesh::get_ellipse_params_end() { return _ellipse_params.end(); }

int Mesh::get_vec_err2_size() { return _err2_vec.size(); }
double Mesh::print_err2(int indice) { return _err2_vec[indice]; }

Mesh::~Mesh()
{
	for(auto it = _mesh.begin(); it != _mesh.end(); it++)
	{
		delete *it;
	}

}

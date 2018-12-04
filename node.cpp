#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <iostream>
#include <utility>
#include "point.h"
#include "node.h"
#include "vertex.h"


Node::Node(Vertex* vertex, Node* mother, Node* father, Node* child)
{
	_vertex = vertex;
	_mother = mother;
	_father = father;
	_child = child;
	_pos = vertex->get_point();
}

//-------------------------------------------------------------------------------------
bool Node::has_two_parents()
{
	return (_mother != NULL && _father != NULL);
}

//------------------------------------------------------------------------------------------
void Node::set_vertex(Vertex * vertex) { _vertex = vertex; }
void Node::set_mother(Node* mother) { _mother = mother; }
void Node::set_father(Node* father) { _father = father; }
void Node::set_child(Node* child) { _child = child; }
void Node::set_pos(Point& pos) { _pos = pos; }
//------------------------------------------------------------------------------------

Point Node::get_pos() { return _pos; }
Vertex* Node::get_vertex() { return _vertex; }
Node* Node::get_mother() { return _mother; }
Node* Node::get_father() { return _father; }
Node* Node::get_child() { return _child; }

//-----------------------------------------------------------------------------

Node::~Node()
{
	std::cout << "delete node " << std::endl;
	if (this->has_two_parents()) // si le noeud est issu de la fusion de deux noeuds précédents
	{
		this->get_father()->set_child(NULL);
		this->get_mother()->set_child(NULL);

	}
	else if (this->get_mother() != NULL) // si le noeud a un parent
	{
		this->get_mother()->set_child(NULL);
	}
	else // on est arrivé aux racines initiales
	{
		std::cout << "racine atteinte " << std::endl;
	}

}


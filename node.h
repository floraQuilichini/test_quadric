#pragma once
#ifndef DEF_Node
#define DEF_Node

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <utility>
#include "point.h"

class Vertex;

class Node
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Node(Vertex* vertex, Node* mother = NULL, Node* father = NULL, Node* child = NULL);
	bool has_two_parents();
	void set_vertex(Vertex * vertex);
	void set_mother(Node* mother);
	void set_father(Node* father);
	void set_child(Node* child);
	void set_pos(Point& pos);
	Point get_pos();
	Vertex* get_vertex();
	Node* get_mother();
	Node* get_father();
	Node* get_child();
	~Node();
	

private:

	Point _pos;
	Vertex* _vertex;
	Node* _mother;
	Node* _father;
	Node* _child;
};
#endif
#pragma once
#ifndef DEF_CollapseMemory
#define DEF_CollapseMemory

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <utility>
#include <set>
#include <list>
#include <vector>
#include <iterator>
#include "point.h"
#include "vertex.h"
#include "node.h"


class CollapseMemory
{
public:
	template<typename InputIterator1>
	CollapseMemory(InputIterator1 begin, InputIterator1 end)
	{
		for (auto it = begin; it != end; it++)
		{
			Node* new_node = new Node(*it, NULL, NULL, NULL);
			(*it)->set_vertex_node(new_node);
			_roots.push_back(new_node);
		}
	}
//	Node* search_parent_node(Vertex* parent_vertex);
	Node* set_new_node(Vertex* vertex_mother, Vertex* vertex_father);
	void increase_ind_row();
	void decrease_ind_row();
	void delete_root(Node* node);
	template<typename InputIterator1>
	void fill_next_row(InputIterator1 begin, InputIterator1 end)
	{
		for (auto it = begin; it != end; it++)
		{
			//Node* mother = search_parent_node(*it);
			Node* mother = (*it)->get_vertex_node();
			if (mother)
			{
				Node* child = new Node(mother->get_vertex(), mother, NULL, NULL);
				mother->set_child(child);
			}
		}

	}

	~CollapseMemory();

private:
	std::vector<Node*> _roots;
	int _ind_current_row = 0;


};

#endif




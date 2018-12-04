#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <utility>
#include <iostream>
#include <iterator>
#include <list>
#include <vector>
#include <tuple>
#include "point.h"
#include "collapseMemory.h"
#include "node.h"
#include "vertex.h"

//------------------------------------------------------------------------------------
/*Node* CollapseMemory::search_parent_node(Vertex* parent_vertex)
{
	Node* previous_node = NULL;

	for (int i = 0; i < _roots.size(); i++)
	{
		Node * current_node = _roots[i];

		// placement at the correct level of roots to update node
		int pos = 0;

		do
		{
			previous_node = current_node;
			current_node = previous_node->get_child();

			pos += 1;
		} while (pos <= _ind_current_row);

		// si le noeud est trouvé, on le renvoie
		if (previous_node->get_vertex() == parent_vertex)
			return previous_node;
	}

	std::cout << "node not found" << std::endl;
	return NULL;

}
*/
//-------------------------------------------------------------------------------------
Node* CollapseMemory::set_new_node(Vertex* vertex_mother, Vertex* vertex_father)
{
	// search parents
	//Node *mother = search_parent_node(vertex_mother);
	//Node *father = search_parent_node(vertex_father);

	// get parents
	Node* mother = vertex_mother->get_vertex_node();
	Node* father = vertex_father->get_vertex_node();

	// set new node
	Node* new_node = mother->get_child();
	new_node->set_father(father);
		// delete old father's child
		delete father->get_child();
		// set new node as child of his father
		father->set_child(new_node);


	// set vertex node
	vertex_mother->set_vertex_node(new_node);

	return new_node;
	
}
//-----------------------------------------------------------------------------------

void CollapseMemory::increase_ind_row() { _ind_current_row += 1; }
void CollapseMemory::decrease_ind_row() { _ind_current_row -= 1; }

void CollapseMemory::delete_root(Node* node)
{
	if (node->get_child())
	{
		/*Node* child = init_node->get_child();
		Node* father = child->get_father();
		Node* mother = child->get_mother();
		father->set_child(NULL);
		mother->set_child(NULL);*/
		delete_root(node->get_child());
	}
	delete node;
}

CollapseMemory::~CollapseMemory()
{
	for (int i = 0; i < _roots.size(); i++)
		delete_root(_roots[i]);
}
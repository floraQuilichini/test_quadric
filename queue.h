#pragma once
#ifndef DEF_queue
#define DEF_queue

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <queue>
#include <list>
#include <vector>
#include <utility>
#include <iterator>

#include "halfedge.h"
#include "vertex.h"
#include "queue.h"

class Queue
{
public:
	template< typename InputIteratorType>
	void compute_threshold(InputIteratorType begin, InputIteratorType end)
	{
		_threshold = 0.0;
		for (InputIteratorType v = begin; v != end; v++)
			_threshold += (*v)->get_next_hedge()->get_cost();
		_threshold /= (double) std::distance(begin, end);
	}

	template< typename InputIteratorType>
	void fill_queue(InputIteratorType begin, InputIteratorType end)
	{
		compute_threshold(begin, end);
		for (InputIteratorType v = begin; v != end; v++)
		{
			if ((*v)->get_next_hedge()->get_cost() < _threshold)
				_queue.push(std::make_pair((*v)->get_next_hedge(), (*v)->get_next_hedge()->get_cost()));
		}
	}

	void pop_element() { _queue.pop(); }
	Halfedge* top_hedge() { return _queue.top().first; }
	bool is_empty() { return _queue.empty(); }

private:
	struct cmpCost
	{
		bool operator()(std::pair<Halfedge*, double> e1_c, std::pair<Halfedge*, double> e2_c) const
		{
			return e1_c.second > e2_c.second;
		}
	};
	std::priority_queue<std::pair<Halfedge*, double>, std::vector<std::pair<Halfedge*, double>>, cmpCost> _queue;
	double _threshold;
	
};

#endif



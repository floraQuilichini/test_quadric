#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/LU/FullPivLU.h>
#include <Eigen/Eigenvalues> 
#include <utility>
#include <iostream>
#include <tuple>

#include "quadric.h"
#include "point.h"



Quadric::Quadric(Eigen::Matrix3d& q1, Eigen::Matrix3d& q2): _quadric(q1 + q2) {}
Quadric::Quadric(Eigen::Matrix3d& q): _quadric(q) {}
Quadric::Quadric(): _quadric(Eigen::Matrix3d()) {}
void Quadric::compute_eigen_vectors_and_values()
{
	Eigen::Matrix2d m;
	m << _quadric(0, 0), _quadric(0, 1), _quadric(1, 0), _quadric(1, 1);

	//get eigenvectors and eigenvalues
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(m);
	_eigen_v1 = es.eigenvectors().col(0);
	_eigen_v2 = es.eigenvectors().col(1);
	_eigen_value1 = es.eigenvalues()[0];
	_eigen_value2 = es.eigenvalues()[1];

}
Point Quadric::compute_opt_pos()
{
		Eigen::Matrix3d dQ = _quadric;
		dQ(2, 0) = 0.0;
		dQ(2, 1) = 0.0;
		dQ(2, 2) = 1.0;

		Eigen::Vector3d v_opt;
		Eigen::Vector3d null_h(0.0, 0.0, 1.0);


		// find p_opt
		Eigen::FullPivLU<Eigen::Matrix3d> dQ_solving_check(dQ);
		if (dQ_solving_check.isInvertible())  // cas inversible
			v_opt = dQ_solving_check.inverse() * null_h;
		
		else // cas non inversible
		{
			// decomposition SVD
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(_quadric, Eigen::ComputeThinU | Eigen::ComputeThinV);
			// solve inverse
			v_opt = svd.solve(null_h);
			std::cout << "dQ is non inversible " << std::endl;
		}

		Eigen::Vector2d p_opt(v_opt[0], v_opt[1]);
		return p_opt;

}
std::tuple<double, double, double> Quadric::compute_ellipse_parameters()
{
	// first double is a (length of the bigger axe), second double is b (length of the smaller axe) and third double is theta (angle between axes of ellipse and cartesian frame)
	
		// les valeurs propres sont triées dans l'ordre décroissant
		// eigen vectors are already normalized


		// 1/lambda donne la longueur d'un axe de l'ellipse
		// la direction de cet axe est donnée par le vecteur propre associé à lambda

		// lambda1 est la valeur propre associée à v1
		// lambda2 est la valeur propre associée à v2
		compute_eigen_vectors_and_values();

		/*std::cout << "vec1 : " << _eigen_v1 << std::endl;
		std::cout << "val1 : " << _eigen_value1 << std::endl;
		std::cout << "vec2 : " << _eigen_v2 << std::endl;
		std::cout << "val2 : " << _eigen_value2 << std::endl;*/


		//on calcule cos(theta) 
		// theta est l'angle entre le repère cartésien principal et le repère formé par les axes de l'ellipse
		// acos renvoie des valeurs entre 0 et pi
		Eigen::Vector2d Ox(1.0, 0.0);
		Eigen::Vector2d Oy(0.0, 1.0);
		double theta;
		double cos_theta = Ox.dot(_eigen_v1);
		double sin_theta = Oy.dot(_eigen_v1);
		if (sin_theta > 0.0)
			theta = acos(cos_theta);
		else
			theta = 2 * 3.1416 - acos(cos_theta);


		return std::make_tuple(1.0 / _eigen_value1, 1.0 / _eigen_value2, theta);

}
std::pair<Eigen::Vector2d, double> Quadric::get_first_vector_and_value() { return std::make_pair(_eigen_v1, _eigen_value1); }
std::pair<Eigen::Vector2d, double> Quadric::get_second_vector_and_value() { return std::make_pair(_eigen_v2, _eigen_value2); }
Eigen::Matrix3d Quadric::get_quadric_value(){ return _quadric; }
void Quadric::set_quadric_value(Eigen::Matrix3d& q) { _quadric = q; }
Eigen::Matrix3d Quadric::split_quadric_value(double alpha, double beta, Quadric& Q_neighboor)
{
	// get Q_split and Q_neighboor
	Eigen::Matrix3d Qs, Qn, newQ;
	Qs = this->get_quadric_value();
	Qn = Q_neighboor.get_quadric_value();

	// compute the new quadric from one side of vsplit (the side of Q_neighboor)
	newQ = beta*Qs + alpha*Qn;

	return newQ;
}
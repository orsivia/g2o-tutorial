#ifndef TYPES_ONE_DOF_H
#define TYPES_ONE_DOF_H

#include "../core/base_vertex.h"
#include "../core/base_unary_edge.h"

#include <Eigen/Geometry>
#include <iostream>

namespace g2o{

typedef Eigen::Matrix<double,1,1> Vector1d;

class Vertex1 : public g2o::BaseVertex<1, Vector1d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    	Vertex1() : g2o::BaseVertex<1, Vector1d>(){}

    	virtual void setToOriginImpl() {
      		_estimate.setZero();
    	}

   	virtual void oplusImpl(const double* update)
   	{
   		_estimate[0] += update[0];
   	}

  	virtual bool read(std::istream& /*is*/)
  	{
    		return false;
  	}
  
  	virtual bool write(std::ostream& /*os*/) const
  	{
    		return false;
  	}		
}; // end class Vertex1

class Edge1 : public g2o::BaseUnaryEdge<1, Vector1d, Vertex1>
{
public:
	Edge1() : g2o::BaseUnaryEdge<1, Vector1d, Vertex1>(){}

	void computeError(){
		const Vertex1* v = static_cast<const Vertex1*>(_vertices[0]);
		Vector1d obs(_measurement);
		_error = obs - v->estimate();
	}

  	virtual bool read(std::istream& /*is*/)
  	{
    		return false;
  	}
  
  	virtual bool write(std::ostream& /*os*/) const
  	{
    		return false;
  	}	
}; // end class Edge1
}
#endif

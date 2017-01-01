#ifndef TYPES_THREE_DOF_H
#define TYPES_THREE_DOF_H

#include "../core/base_vertex.h"
#include "../core/base_unary_edge.h"

#include <Eigen/Geometry>
#include <iostream>

namespace g2o{
class Vertex3 : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
    	Vertex3() : g2o::BaseVertex<3, Eigen::Vector3d>(){}

    	virtual void setToOriginImpl() {
      		_estimate.setZero();
    	}

   	virtual void oplusImpl(const double* update)
   	{
   		_estimate[0] += update[0];
   		_estimate[1] += update[1];
   		_estimate[2] += update[2];
   	}

  	virtual bool read(std::istream& /*is*/)
  	{
    		return false;
  	}
  
  	virtual bool write(std::ostream& /*os*/) const
  	{
    		return false;
  	}		
}; // end class Vertex3

class Edge3 : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, Vertex3>
{
public:
	Edge3() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, Vertex3>(){}

	void computeError(){
		const Vertex3* v = static_cast<const Vertex3*>(_vertices[0]);
		Eigen::Vector3d obs(_measurement);
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
}; // end class Edge3
}
#endif

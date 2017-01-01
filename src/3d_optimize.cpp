#include "../g2o/g2o/types/types_three_dof.h"
#include "../g2o/g2o/core/solver.h"
#include "../g2o/g2o/core/block_solver.h"
#include "../g2o/g2o/core/sparse_optimizer.h"
#include "../g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "../g2o/g2o/core/robust_kernel_impl.h"
#include "../g2o/g2o/solvers/linear_solver_eigen.h"

#include <Eigen/StdVector>

#include <iostream>

using namespace std;
using namespace g2o;
using namespace Eigen;

int main(){
	int nIterations = 10;

  	// create the linear solver
  	//typedef BlockSolver< BlockSolverTraits<3, 3> > BlockSolver_3_3;
	BlockSolver_3_3::LinearSolverType* linearSolver = new LinearSolverEigen<BlockSolver_3_3::PoseMatrixType>();

  	// create the block solver on top of the linear solver
  	BlockSolver_3_3* blockSolver = new BlockSolver_3_3(linearSolver);

  	// create the algorithm to carry out the optimization
  	OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);

	// create the optimizer
	SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	optimizer.setAlgorithm(optimizationAlgorithm);

	// create vertices and edges
	Vertex3* v = new Vertex3();
	v->setId(0);
	cout <<  "initial estimate=\n" << v->estimate() << endl;
	//Vector3d estimate(0,0,0);
	//v->setEstimate(estimate);
	//v->setFixed(true);
	optimizer.addVertex(v);

	Edge3* e = new Edge3();
	e->setVertex(0, v);
	Vector3d measurement(10,5,3);
	e->setMeasurement(measurement);
	// DONT MISS THIS LINE!!!
	e->setInformation(Matrix3d::Identity());
	optimizer.addEdge(e);

	// optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);

  	cout <<  "computed estimate=\n" << v->estimate() << endl;
}

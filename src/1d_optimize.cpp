#include "../g2o/g2o/types/types_one_dof.h"
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
	typedef Eigen::Matrix<double,1,1> Vector1d;
	int nIterations = 10;

  	// create the linear solver
	BlockSolver_1_1::LinearSolverType* linearSolver = new LinearSolverEigen<BlockSolver_1_1::PoseMatrixType>();

  	// create the block solver on top of the linear solver
  	BlockSolver_1_1* blockSolver = new BlockSolver_1_1(linearSolver);

  	// create the algorithm to carry out the optimization
  	OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);

	// create the optimizer
	SparseOptimizer optimizer;
	optimizer.setVerbose(false);
	optimizer.setAlgorithm(optimizationAlgorithm);

	// create vertices
	Vertex1* v = new Vertex1();
	v->setId(0);
	optimizer.addVertex(v);
	cout <<  "initial estimate=\n" << v->estimate() << endl;

	// create edges
	Edge1* e = new Edge1();
	e->setVertex(0, v);
	// Obervations
	Vector1d measurement;
	measurement << 10.0;
	e->setMeasurement(measurement);
	// DONT MISS THIS LINE!!!
	e->setInformation(Vector1d::Identity());
	optimizer.addEdge(e);

	// optimize!
	optimizer.initializeOptimization();
	optimizer.optimize(nIterations);

  	cout <<  "computed estimate=\n" << v->estimate() << endl;
}

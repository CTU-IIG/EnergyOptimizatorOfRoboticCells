/*
	This file is part of the EnergyOptimizatorOfRoboticCells program.

	EnergyOptimizatorOfRoboticCells is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	EnergyOptimizatorOfRoboticCells is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with EnergyOptimizatorOfRoboticCells. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef HLIDAC_PES_SOLVER_INTERFACE_H
#define HLIDAC_PES_SOLVER_INTERFACE_H

/*!
 * \file SolverInterface.h
 * \author Libor Bukata
 * \brief Solver-independent interface for solving Integer Linear Programming problems.
 * \note Cplex, Gurobi, and lp_solve are currently supported.
 * However, other solvers can be easily added by implementing this interface, i.e. approx. 160 lines of code per solver.
 */

#include <iostream>
#include <string>
#include <vector>
#include "SolverConfig.h"
#include "ILPModel/ILPModel.h"
#include "Shared/Exceptions.h"
#include "Shared/NumericConstants.h"

//! Returns an identification of the used solver, e.g. 'Gurobi 6.0.4'.
std::string solverIdentification();

//! Constants specifying whether a solution is optimal, feasible, infeasible, unbounded, or undefined, respectively.
enum Status {
	ILP_OPTIMAL, ILP_FEASIBLE, ILP_INFEASIBLE, ILP_UNBOUNDED, ILP_UNKNOWN
};

//! Structure storing a solution of an Integer Linear Programming problem.
struct SolutionILP {
	//! The criterion value of the solution.
	double criterion;
	//! The best known lower or upper bound.
	double bound;
	//! The best found solution.
	std::vector<double> solution;
	//! %Solution status, see \ref Status enum.
	Status status;
};

/*!
 * \param numberOfThreads Number of concurrent threads.
 * \brief It enables the solver to initialize all the data structures
 * (e.g. expensive to construct) required for flawless performance of multiple threads.
 */
void initializeLocalEnvironments(int numberOfThreads);

/*!
 * \param m Integer Linear Programming problem to be solved.
 * \param verbose True if the solver should print some information to output, for multi-threading it must be disabled.
 * \param gap A solver is stopped if the relative gap from the best bound is achieved.
 * \param timeLimit Time limit of a solver, value 0.0 disables the time limit.
 * \param numberOfThreads Number of threads to be used concurrently.
 * \param threadId Thread id, possible values {0, ..., number of threads-1}.
 * \return The best feasible solution if found, otherwise infeasibility is indicated in the state of a solution.
 * \brief Integer Linear Programming solver is called to solve the problem and the solution is returned.
 */
SolutionILP solveILP(const ILPModel& m, bool verbose, double gap = 0.0, double timeLimit = 0.0, int numberOfThreads = 1, int threadId = 0);

#endif


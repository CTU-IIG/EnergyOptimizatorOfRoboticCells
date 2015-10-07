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

#ifndef HLIDAC_PES_ROBOTIC_LINE_SOLVER_LP_H
#define HLIDAC_PES_ROBOTIC_LINE_SOLVER_LP_H

/*!
 * \file RoboticLineSolverLP.h
 * \author Libor Bukata
 * \brief Declares LP solver which deals with partially fixed energy optimization problems created by the parallel heuristic.
 */

#include <vector>
#include "Settings.h"
#include "RoboticLine.h"
#include "Solution/Solution.h"
#include "Shared/PrecalculatedMapping.h"
#include "ILPModel/ILPModel.h"
#include "ILPSolver/VariableMappingLP.h"
#include "ILPSolver/ConstraintsGenerator.h"
#include "HeuristicSolver/KnowledgeBase.h"

/*!
 * The parallel heuristic \ref ParallelHeuristicSolver evaluates lots of partially fixed problems
 * where locations, power saving modes, and movements have been preselected by the heuristic.
 * \brief Fixed locations, power saving modes, and movements.
 * \note The spatial compatibility is resolved externally by an appropriate selection of locations.
 */
struct PartialSolution {
	//! Selected locations for each robot, time ordered.
	std::vector<std::vector<Location*>> locs;
	//! Selected power saving modes of related locations.
	std::vector<std::vector<RobotPowerMode*>> pwrms;
	//! Selected movements, implied from the selected locations and their order.
	std::vector<std::vector<Movement*>> mvs;
};

/*!
 * Linear Programming solver that determines an optimal timing of a partially fixed problem.
 * It is used by the parallel heuristic, see \ref ParallelHeuristicSolver, that evaluates lots of
 * partially fixed problems to find high quality solutions in terms of energy consumption.
 * \brief Determines an optimal timing of a partially fixed problem.
 */
class RoboticLineSolverLP {
	public:
		/*!
		 * \param r %Robot for which the timing should be evaluated.
		 * \param ps Partially fixed problem, i.e. selected locations, modes, and movements.
		 * \param m Fast mapping for the searching in the data structure of the robotic cell (containing one robot).
		 * \brief It builds a Linear Programming problem (variables, constraints, criterion) to be solved.
		 */
		RoboticLineSolverLP(Robot* r, const PartialSolution& ps, const PrecalculatedMapping& m);
		/*!
		 * \param l Robotic cell for which the timing should be evaluated.
		 * \param ps Partially fixed problem, i.e. selected locations, modes, and movements.
		 * \param m Fast mapping for the searching in the data structure of the robotic cell.
		 * \brief It builds a Linear Programming problem (variables, constraints, criterion) to be solved.
		 */
		RoboticLineSolverLP(const RoboticLine& l, const PartialSolution& ps, const PrecalculatedMapping& m);

		//! The energy optimal timing is determined by solving Linear Programming problem and the solution is returned.
		Solution solve() const;

		/*!
		 * \param i,j First and second considered activity, respectively.
		 * \param multipleOfCycleTime Multiple of the robot cycle time.
		 * \brief It adds a constraint \f$s_j \geq s_i+d_i+CT*\mathrm{multipleOfCycleTime}\f$ to resolve a collision
		 * where \f$s_i\f$ and \f$d_i\f$ are start time and duration of the activity \f$i\f$,
		 * respectively, and \f$\mathrm{CT}\f$ is the robot cycle time.
		 */
		void addCollisionResolution(Activity* i, Activity* j, const int32_t& multipleOfCycleTime);
	private:
		//! It adds additional float variables for optional activities.
		void addVariablesForSelectedDynamicActivities();
		/*!
		 * \param m Fast mapping for the searching in the data structure of the robotic cell.
		 * \param addTimeLags Whether time lags should be considered, i.e. a robotic cell with more than one robot.
		 * \brief Complete construction of the Linear Programming problem for the energy optimization.
		 * \note Gurobi convex functions can be employed to accelerate the solution process.
		 */
		void construct(const PrecalculatedMapping& m, bool addTimeLags = true);
		/*!
		 * \param m Fast mapping for the searching in the data structure of the robotic cell.
		 * \param numberOfRobots Number of robots considered in the problem.
		 * \brief It checks that the arguments passed to the constructor are valid and correct.
		 * \note It is only called in the debug mode due to the performance issues.
		 */
		void checkArguments(const PrecalculatedMapping& m, const uint32_t& numberOfRobots = 1u) const;

		//! Linear Programming problem to be solved.
		ILPModel mModel;
		//! Mapping of the continuous variables.
		VariableMappingLP mMapper;
		//! Generator of the constraints.
		ConstraintsGenerator mGenerator;
		//! Partially fixed problem, or in other words, a partial solution.
		PartialSolution mPartialSolution;
};

#endif

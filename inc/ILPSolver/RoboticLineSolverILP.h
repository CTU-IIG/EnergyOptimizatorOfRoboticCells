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

#ifndef HLIDAC_PES_ROBOTIC_LINE_SOLVER_ILP_H
#define HLIDAC_PES_ROBOTIC_LINE_SOLVER_ILP_H

/*!
 * \file RoboticLineSolverILP.h
 * \author Libor Bukata
 * \brief RoboticLineSolverILP class, declared in this file, addresses the energy optimization problem of robotic cells.
 */

#include "Settings.h"
#include "RoboticLine.h"
#include "Solution/Solution.h"
#include "ILPModel/ILPModel.h"
#include "ILPSolver/VariableMappingILP.h"
#include "ILPSolver/ConstraintsGenerator.h"

/*!
 * An exact solver for the energy optimization of robotic cells.
 * It employs an Integer Linear Programming solver to solve the problem.
 * Only small instances can be solved with a satisfactory quality.
 * \brief An exact solver for the energy optimization problem.
 * \see \ref math_form
 */
class RoboticLineSolverILP {
	public:
		/*!
		 * \param r Robot to be optimized.
		 * \param m Fast mapping for the searching in the data structure of the robotic cell (containing one robot).
		 * \brief Integer Linear Programming problem is built from the robot.
		 */
		RoboticLineSolverILP(Robot* r, const PrecalculatedMapping& m);
		/*!
		 * \param l Robotic cell to be energy optimized.
		 * \param m Fast mapping for the searching in the data structure of the robotic cell.
		 * \brief Integer Linear Programming problem is built from the robotic cell.
		 */
		RoboticLineSolverILP(const RoboticLine& l, const PrecalculatedMapping& m);

		/*!
		 * \param relGap A relative gap used as a stop criterion.
		 * \param timLim Time limit used as a stop criterion.
		 * \return The best found solution.
		 * \brief ILP solver optimizes the energy consumption of the robotic cell until a stop criterion is reached.
		 */
		Solution solve(double relGap = Settings::ILP_RELATIVE_GAP, double timLim = Settings::MAX_RUNTIME) const;
		/*!
		 * A tight lower bound neglects the global constraints, i.e. linkage between robots, and solves each robot individually
		 * as an ILP problem with a given time limit. The resulting bound is a summation of individual lower bounds.
		 * \param robots Robots located in a robotic cell.
		 * \param m Fast mapping for the searching in the data structure of the related robotic cell.
		 * \return Lower estimation on the optimal energy consumption of the robotic cell.
		 * \brief A tight lower bound on the energy consumption.
		 */
		static double lowerBoundOnEnergy(const std::vector<Robot*>& robots, const PrecalculatedMapping& m);

	private:
		/*!
		 * \param generator Instance of the class is responsible for the constraints generation.
		 * \param addGlobalConstraints Whether the global constraints should be considered.
		 * \brief It builds the Integer Linear Programming problem corresponding to the energy optimization problem of the robotic cell.
		 */
		void construct(ConstraintsGenerator& generator, bool addGlobalConstraints = true);

		//! The energy optimization of the robotic cell formulated as an Integer Linear Programming problem.
		ILPModel mModel;
		//! Mapping of both the continuous and binary variables.
		VariableMappingILP mMapper;
};

#endif

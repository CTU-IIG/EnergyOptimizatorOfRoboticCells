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

#ifndef HLIDAC_PES_SOLUTION_CHECKER_H
#define HLIDAC_PES_SOLUTION_CHECKER_H

/*!
 * \file SolutionChecker.h
 * \author Libor Bukata
 * \brief The file declares a class responsible for checking of solutions.
 */

#include <array>
#include <map>
#include <set>
#include <vector>
#include "RoboticLine.h"
#include "Shared/Utils.h"
#include "Solution/Solution.h"
#include "Shared/PrecalculatedMapping.h"

/*!
 * %SolutionChecker verifies whether a given solution is valid, or in other words,
 * it checks whether the solution is feasible if it was declared to be feasible or optimal.
 * It comprises verification of the time, spatial, and energy aspects.
 * \brief An instance of this class is devoted to the solution checking.
 * \see InstancesChecker
 */
class SolutionChecker {
	public:
		//! Initialization of the solution checker.
		SolutionChecker(const RoboticLine& l, const PrecalculatedMapping& m, const Solution& sol)
			: mLine(l), mSolution(sol), mMapping(m) { };

		/*!
		 * \return True if the solution is valid, otherwise false.
		 * \brief It calls the private member methods to verify the solution.
		 * \note If false is returned then there is a high probability that a used optimization algorithm is flawed.
		 */
		bool checkAll();
		//! It returns error message(s) describing why the solution is invalid.
		std::vector<std::string> errorMessages() const { return mErrorMsg; }
	private:
		//! Checks the calculation of the criterion.
		void checkCriterion();
		//! Checks whether the robot, i.e. production, cycle time is met.
		void checkProductionCycleTime();
		//! Checks durations of static activities.
		void checkStaticActivities();
		//! Checks durations of dynamic activities.
		void checkDynamicActivities();
		//! Checks whether each robot path is closed and hamiltonian (each static activity is visited just once).
		void checkScheduleContinuity();

		//! Verifies that time lags and spatial compatibility are not violated.
		void checkGlobalConstraints();
		//! Verifies that a collision does not occur in the solution.
		void checkCollisionZones();

		//! Auxiliary method that extracts selected modes (location/movement identifications) for activities \a a1 and \a a2, respectively.
		std::array<uint32_t, 2> extractModes(Activity* a1, Activity* a2) const;

		//! The data structure of the robotic cell for which the solution is verified.
		RoboticLine mLine;
		//! The solution to be checked.
		Solution mSolution;
		//! Precalculated mapping constructed for the robotic cell.
		PrecalculatedMapping mMapping;
		//! Error messages describing the reasons why the solution is invalid.
		std::vector<std::string> mErrorMsg;
};

#endif


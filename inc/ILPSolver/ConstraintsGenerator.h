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

#ifndef HLIDAC_PES_CONSTRAINTS_GENERATOR_H
#define HLIDAC_PES_CONSTRAINTS_GENERATOR_H

/*!
 * \file ConstraintsGenerator.h
 * \author Libor Bukata
 * \brief Declares a class for the generation of constraints.
 */

#include <cassert>
#include <map>
#include <vector>
#include <utility>
#include "RoboticLine.h"
#include "Shared/PrecalculatedMapping.h"
#include "ILPModel/ILPModel.h"
#include "ILPSolver/VariableMappingILP.h"

/*!
 * The generator constructs the constraints related to the energy optimization of robotic cells.
 * The description of the constraints is available at \ref math_form and \ref heuristic pages.
 * \brief Generation of the (I)LP constraints.
 * \see \ref math_form, \ref heuristic
 */
class ConstraintsGenerator {
	public:
		//! Empty constructor, the object needs to be assigned a robot or robotic cell later.
		ConstraintsGenerator() : mCycleTime(0.0), mConstraintsCounter(0ul), mLine(nullptr), mMapping(nullptr) { }
		//! Generation of the constraints for a given robot.
		ConstraintsGenerator(Robot* r, const PrecalculatedMapping* m) : mConstraintsCounter(0ul) { reset(r,m); }
		//! Generation of the constraints for a given robotic cell.
		ConstraintsGenerator(const RoboticLine& l, const PrecalculatedMapping* m) : mConstraintsCounter(0ul) { reset(l,m); }

		//! All the generated constraints are removed.
		void reset();
		//! Removes all the constraints and sets a new robot as active.
		void reset(Robot* r, const PrecalculatedMapping* m);
		//! Removes all the constraints and sets a new robotic cell as active.
		void reset(const RoboticLine& l, const PrecalculatedMapping* m);

		//! It efficiently moves all the generated constraints to the given model.
		uint64_t moveConstraintsToModel(ILPModel& m);

		/*!
		 * \param d Mapping of variables corresponding to activity durations.
		 * \param W Mapping of variables corresponding to activity consumptions.
		 * \param x Mapping of variables corresponding to the selection of locations.
		 * \param z Mapping of variables corresponding to the selection of power saving modes.
		 * \brief Add the constraints propagating the consumption of static activities to the criterion.
		 */
		void addEnergyFunctions1(const map1to1& d, const map1to1& W, const map2to1& x, const map2to1& z);
		/*!
		 * \param d Mapping of variables corresponding to activity durations.
		 * \param W Mapping of variables corresponding to activity consumptions.
		 * \param y Mapping of variables corresponding to the selection of movements.
		 * \brief Add the constraints propagating the consumption of dynamic activities to the criterion.
		 */
		void addEnergyFunctions2(const map1to1& d, const map1to1& W, const map2to1& y);
		//! It adds the energy functions of both the static and dynamic activities.
		void addEnergyFunctions(const map1to1& d, const map1to1& W, const map2to1& x, const map2to1& z, const map2to1& y);

		/*!
		 * \param x Mapping of variables corresponding to the selection of locations.
		 * \brief Just one location, i.e. coordinate, is selected for each static activity.
		 */
		void addUniquePointSelection(const map2to1& x);
		/*!
		 * \param z Mapping of variables corresponding to the selection of power saving modes.
		 * \brief Just one power saving mode (including the dummy one - motors) is applied for each static activity.
		 */
		void addUniquePowerModeSelection(const map2to1& z);
		 //! It calls \ref addUniquePointSelection and \ref addUniquePowerModeSelection methods.
		void addUniqueModeSelection(const map2to1& x, map2to1& z);

		/*!
		 * \param x Mapping of variables corresponding to the selection of locations.
		 * \param y Mapping of variables corresponding to the selection of movements.
		 * \brief Add flow preservation constraints ensuring that robot leaves the same location as it enters.
		 */
		void addFlowConstraints(const map2to1& x, const map2to1& y);

		/*!
		 * \param s Mapping of variables corresponding to start times of activities.
		 * \param d Mapping of variables corresponding to durations of activities.
		 * \brief Adds fixed precedences, i.e. enforces the have-to-be order of some activities.
		 */
		void addFixedPrecedences(const map1to1& s, const map1to1& d);
		/*!
		 * \param y Mapping of variables corresponding to the selection of movements.
		 * \param w Mapping of variables corresponding to the selection of optional dynamic activities.
		 * \brief According to the selected movements it decides which optional dynamic activities are performed.
		 */
		void addPrecedenceSelectionConstraints(const map2to1& y, const map2to1& w);
		/*!
		 * \param s Mapping of variables corresponding to start times of activities.
		 * \param d Mapping of variables corresponding to durations of activities.
		 * \param w Mapping of variables corresponding to the selection of optional dynamic activities.
		 * \brief It adds selectable precedences which model alternative orders of activities.
		 */
		void addSelectablePrecedences(const map1to1& s, const map1to1& d, const map2to1& w);
		//! It calls all the methods related to the order of activities.
		void addAllPrecedences(const map1to1& s, const map1to1& d, const map2to1& y, const map2to1& w);

		/*!
		 * \param d Mapping of variables corresponding to durations of activities.
		 * \param z Mapping of variables corresponding to the selection of power saving modes.
		 * \brief Constraints restricting the minimal duration of static activities with respect to their assigned power saving modes.
		 */
		void addMinimalDurationConstraints1(const map1to1& d, const map2to1& z);
		/*!
		 * \param d Mapping of variables corresponding to durations of activities.
		 * \param y Mapping of variables corresponding to the selection of movements.
		 * \brief Constraints restricting the minimal duration of dynamic activities with respect to their selected movements.
		 */
		void addMinimalDurationConstraints2(const map1to1& d, const map2to1& y);
		/*!
		 * \param d Mapping of variables corresponding to durations of activities.
		 * \param y Mapping of variables corresponding to the selection of movements.
		 * \brief Constraints restricting the maximal duration of dynamic activities with respect to their selected movements.
		 */
		void addMaximalDurationConstraints2(const map1to1& d, const map2to1& y);
		//! It constructs all the constraints that limit the duration of activities.
		void addDurationConstraints(const map1to1& d, const map2to1& z, const map2to1& y);

		/*!
		 * \param s Mapping of variables corresponding to start times of activities.
		 * \param allRequired Whether all the time lags have to be applicable (ILP yes, reduced LP not).
		 * \brief Time lags enforcing correct time synchronizations between robots are added.
		 */
		void addTimeLags(const map1to1& s, bool allRequired = true);
		/*!
		 * \param x Mapping of variables corresponding to the selection of locations.
		 * \brief It defines the spatial compatibility between robots, in other words
		 * a workpice is taken from the same place as it has been given.
		 */
		void addSpatialCompatibilityConstraints(const map2to1& x);
		/*!
		 * \param s Mapping of variables corresponding to the start times of activities.
		 * \param d Mapping of variables corresponding to the durations of activities.
		 * \param x Mapping of variables corresponding to the selection of locations.
		 * \param y Mapping of variables corresponding to the selection of movements.
		 * \param c Mapping of variables that decide the order of potentially colliding activities.
		 * \brief Adds constraints that enforce a collision-free solution.
		 */
		void addCollisions(const map1to1& s, const map1to1& d, const map2to1& x, const map2to1& y, const map4toN& c);
		//! Adds all the global constraints, i.e. time lags, spatial compatibility, and collision resolution.
		void addGlobalConstraints(const map1to1& s, const map1to1& d, const map2to1& x, const map2to1& y, const map4toN& c);

		/*!
		 * \param d Mapping of variables corresponding to the durations of activities.
		 * \param W Mapping of variables corresponding to activity consumptions.
		 * \param loc Selected location of a static activity.
		 * \param pwrm Power saving mode that is applied while the robot is stationary in the location.
		 * \brief Adds energy function for a static activity, its location, and its applied power saving mode.
		 */
		void addEnergyFunction(const map1to1& d, const map1to1& W, Location *loc, RobotPowerMode *pwrm);
		/*!
		 * \param d Mapping of variables corresponding to the durations of activities.
		 * \param W Mapping of variables corresponding to activity consumptions.
		 * \param mv Selected movement of a dynamic activity.
		 * \brief Adds a piece-wise linear function for a dynamic activity and its movement.
		 */
		void addEnergyFunction(const map1to1& d, const map1to1& W, Movement* mv);
		/*!
		 * \param s Mapping of variables corresponding to the start times of activities.
		 * \param d Mapping of variables corresponding to the durations of activities.
		 * \param mvs Selected movements that must occur in a solution.
		 * \brief Each has-to-be movement imposes a fixed precedence that is added to the formulation in the form of a constraint.
		 */
		void addSelectedPrecedences(const map1to1& s, const map1to1& d, const std::vector<Movement*>& mvs);
		/*!
		 * \param s Mapping of variables corresponding to the start times of activities.
		 * \param d Mapping of variables corresponding to the durations of activities.
		 * \param i,j First and second considered activity, respectively.
		 * \param multipleOfCycleTime Multiple of the robot cycle time.
		 * \brief It adds a constraint \f$s_{a_2} \geq s_{a_1}+d_{a_1}+CT*\mathrm{multipleOfCycleTime}\f$ to resolve a collision,
		 * i.e. it adds a precedence for a given multiple of the robot cycle time.
		 */
		void addCollisionResolution(const map1to1& s, const map1to1& d, uint32_t i, uint32_t j, int32_t multipleOfCycleTime);

	private:
		//! It initializes the member variables and removes all the constraints.
		void initializeDataStructures();
		/*!
		 * \param row A row of the constraint matrix, i.e. coefficients before the variables.
		 * \param op Equal, less or equal, greater or equal.
		 * \param b Right-hand side constant of the constraint.
		 * \param conDesc Description of the constraint.
		 * \brief Add constraint "row'*vars op b" into the formulation.
		 */
		void addConstraint(SparseMatrix<double>::Row& row, Operator op, const double& b, std::string conDesc = "");

		//! Copied robot cycle time.
		double mCycleTime;
		//! The number of constraints generated by an instance of this class.
		uint64_t mConstraintsCounter;

		//! Pointer to an active robotic cell.
		RoboticLine const* mLine;
		//! Robots located in the robotic cell.
		std::vector<Robot*> mRobots;
		//! Mapping calculated for the robotic cell.
		const PrecalculatedMapping* mMapping;
		//! Static activities filtered from the robotic cell.
		std::vector<StaticActivity*> mStaticActivities;
		//! Dynamic activities filtered from the robotic cell.
		std::vector<DynamicActivity*> mDynamicActivities;

		//! Constraint matrix.
		SparseMatrix<double> mA;
		//! Operators of constraints.
		std::vector<Operator> mOp;
		//! Right-hand side constants
		std::vector<double> mB;
		//! Description of the constraints.
		std::vector<std::string> mConDesc;
};

#endif

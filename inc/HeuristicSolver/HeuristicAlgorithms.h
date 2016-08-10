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

#ifndef HLIDAC_PES_HEURISTIC_ALGORITHMS_H
#define HLIDAC_PES_HEURISTIC_ALGORITHMS_H

/*!
 * \file HeuristicAlgorithms.h
 * \author Libor Bukata
 * \brief The optimization sub-heuristics and Linear Programming solver for partially fixed problems.
 */

#include "Shared/PrecalculatedMapping.h"
#include "HeuristicSolver/DataStructures.h"
#include "HeuristicSolver/KnowledgeBase.h"
#include "ILPSolver/RoboticLineSolverLP.h"

/*!
 * An instance of this class provides an access to the sub-heuristics and solver for partially fixed problems (called tuples).
 * The solver determines an energy-efficient timing for fixed locations, movements, and power saving modes.
 * The aim of the sub-heuristics is to modify tuples in such a way that would be resulting in an energy reduction.
 * \brief Defines the optimization part of the parallel heuristic, i.e. sub-heuristics and the evaluation of tuples.
 */
class HeuristicAlgorithms {
	public:
		//! Constructor makes references to the required data, i.e. the robotic cell, its mapping, and shared data of the heuristic.
		HeuristicAlgorithms(KnowledgeBase& kb, const RoboticLine& line, const PrecalculatedMapping& mapping)
			: mKB(kb), mLine(line), mPenaltyMultiplier(4.0), mMapping(mapping) { }

		/*!
		 * \param[in,out] ps A partial problem with given fixed locations (+their order).
		 * \return Fixed locations for each robot, indexed by robot.
		 * \brief It appends the fixed movements (implied from fixed locations),
		 * and optionally fastest power saving modes to \a ps data structure.
		 */
		std::vector<std::vector<Location*>> initializePartialSolution(PartialSolution& ps) const;
		/*!
		 * \param ps A partial problem with fixed locations, movements, and power saving modes.
		 * \param t A tuple corresponding to the partial problem.
		 * \param algo Specifies what was called before solving this partial problem, i.e. either a sub-heuristic or initialization phase.
		 * \return Energy-efficient timing of the robotic cell for a given tuple.
		 * \brief Employs Linear Programming to obtain energy-efficient timing for a given tuple.
		 */
		OptimalTiming solvePartialProblem(const PartialSolution& ps, const CircuitTuple& t, Algo algo);
		/*!
		 * \param ps A partial problem with fixed locations, movements, and power saving modes.
		 * \param s %Solution of the partially fixed problem.
		 * \return Extracted timing of a robotic cell.
		 * \brief Auxiliary method that converts a general form of the solution to the timing of the robotic cell.
		 */
		OptimalTiming convert(const PartialSolution& ps, const Solution& s) const;
		/*!
		 * \param m1 A movement or location of the first considered activity.
		 * \param s1, d1 Start time and duration of the first activity, respectively.
		 * \param m2 A movement or location of the second considered activity.
		 * \param s2, d2 Start time and duration of the second activity, respectively.
		 * \return How to resolve this collision. If the collision does not occur, then the pointers to activities are set to null.
		 * \brief It checks whether there is a collision between two specified activities, and eventually returns how to resolve this collision.
		 */
		CollisionResolution resolveCollision(ActivityMode* m1, double s1, double d1, ActivityMode* m2, double s2, double d2) const;
		/*!
		 * \param ot Timing of the robotic cell.
		 * \return How to resolve the worst collision that occurred in the given timing.
		 * \brief It finds the worst collision, i.e. with the longest time of active collision, and returns how to resolve it.
		 */
		CollisionResolution resolveTheWorstCollision(const OptimalTiming& ot) const;

		/*!
		 * \param[in,out] ot Timing of the robotic cell.
		 * \param[in,out] ps A partial problem with fixed locations, movements, and power saving modes.
		 * \param fixed Locations that cannot be changed because the spatial compatibility has to be preserved.
		 * \param[out] solutionChanged It indicates whether the partially fixed problem was modified.
		 * \return An estimation of energy saving after performing all the changes.
		 * \brief The sub-heuristic locally optimizes the robot paths (change locations) to reduce energy consumption.
		 */
		double heuristicLocationChanges(OptimalTiming& ot, PartialSolution& ps, const std::vector<std::vector<Location*>>& fixed, bool& solutionChanged) const;
		/*!
		 * \param ot Timing of the robotic cell.
		 * \param[in,out] ps A partial problem with fixed locations, movements, and power saving modes.
		 * \param tabuList A list with the forbidden modifications.
		 * \param[out] solutionChanged It indicates whether the partially fixed problem was modified.
		 * \return An estimation of energy saving after performing all the changes.
		 * \brief The sub-heuristic tries to switch from one power saving mode to another one for a location in order to reduce energy consumption.
		 */
		double heuristicPowerModeSelection(const OptimalTiming& ot, PartialSolution& ps, TabuList& tabuList, bool& solutionChanged) const;
	private:
		/*!
		 * \param[in,out] robotGantt Timing of the robot activities where some activities have modified durations.
		 * \return Estimated energy consumption of the related robot after the timing is changed.
		 * \brief It scales durations of activities in such a way that the robot cycle time is met.
		 * Updated start times and durations are reflected in the \a robotGantt variable.
		 * \see heuristicPowerModeSelection
		 */
		double scaleGanttToCycleTime(std::vector<ActivityModeInfo>& robotGantt) const;
		/*!
		 * \param robotGantt Timing of the robot activities where the robot cycle time is met.
		 * \param ot Timing of the whole robotic cell.
		 * \return Penalty for violating time lags and collisions.
		 * \brief It calculates the penalty for the violation of time lags and collisions.
		 * \see heuristicLocationChanges, heuristicPowerModeSelection, mPenaltyMultiplier
		 */
		double calculateBreakagePenalty(const std::vector<ActivityModeInfo>& robotGantt, const OptimalTiming& ot) const;

		//! Access to the shared data of the heuristic.
		KnowledgeBase& mKB;
		//! Access to the robotic cell to be optimized.
		const RoboticLine& mLine;
		//! Aggregated breakage time (see \ref calculateBreakagePenalty) is multiplied by this constant to get the penalty.
		const double mPenaltyMultiplier;
		//! Fast searching in the data structure of the robotic cell.
		const PrecalculatedMapping& mMapping;
};

#endif

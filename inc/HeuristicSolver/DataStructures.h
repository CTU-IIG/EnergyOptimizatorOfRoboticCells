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

#ifndef HLIDAC_PES_DATASTRUCTURES_H
#define HLIDAC_PES_DATASTRUCTURES_H

/*!
 * \file DataStructures.h
 * \author Libor Bukata
 * \brief Various data structures used by the heuristic.
 */

#include <numeric>
#include <vector>
#include <unordered_map>
#include "RoboticLine.h"
#include "Shared/Utils.h"
#include "Shared/Exceptions.h"

/* PROCESS PHASES OF THE PARALLEL SOLVER */

//! Defining constants for different states of the heuristic.
enum Algo {
        LP_INITIAL_PROBLEM = 0, POWER_MODE_HEURISTIC = 1, LOCATION_CHANGE_HEURISTIC = 2, PATH_CHANGE = 4
};

/* GRAPH REPRESENTATION FOR DISTANCE MATRIX COMPUTATION */

/*!
 * \brief A graph edge in the distance graph.
 * \see Graph
 */
struct Edge {
	//! %Edge leaves the activity \a f.
	StaticActivity *f;
	//! %Edge enters to activity \a t.
	StaticActivity *t;
	//! An index assigned to the activity \a f, used for an access to the distance matrix.
	uint32_t i;
	//! An index assigned to the activity \a t, used for an access to the distance matrix.
	uint32_t j;
	/*!
	 * The edge length corresponding to the minimal duration of activity \a f plus
	 * the minimal duration of the dynamic activity that connects activity \a f with activity \a t.
	 */
	double length;
};

/*!
 * \brief A graph in which random alternatives will be searched for.
 */
struct Graph {
	//! %Graph edges.
	std::vector<Edge> edges;
	//! An index of the start node.
	uint32_t startIdent;
	//! An index of the end node.
	uint32_t endIdent;
	//! Node index to static activity, note that dynamic activities are already incorporated in the lengths of edges.
	std::vector<StaticActivity*> idToActivity;
	//! Node index to leaving edges that enters to its (node) successors.
	std::vector<std::vector<Edge>> idToSuccessors;
};

/* HAMILTONIAN CIRCUIT REPRESENTATION */

/*!
 * \tparam T Either a pointer to Location or StaticActivity.
 * \brief Closed path through locations or static activities including an order of operations, i.e. alternative.
 */
template <class T>
struct HamiltonianCircuit	{
	//! The lower bound on the robot cycle time for this circuit (neglecting global constraints).
	double minLength;
	//! Closed path through locations or static activities, each static activity is visited just once.
	std::vector<T*> circuit;
};

/*!
 * The structure records the selected alternative, i.e. circuit, and the locations that have to be fixed to resolve spatial compatibility.
 * \brief Hamiltonian circuit through static activities and the fixed locations.
 */
struct ShortestCircuit {
	ShortestCircuit() = default;
	ShortestCircuit(const std::vector<StaticActivity*> arg1, const std::vector<Location*>& arg2) : circuit(arg1), fixedLocations(arg2) { }
	bool operator==(const ShortestCircuit& sc) const {
		return circuit == sc.circuit && fixedLocations == sc.fixedLocations;
	}

	//! Selected alternative, i.e. order of operations.
	std::vector<StaticActivity*> circuit;
	//! Locations required to be fixed to enforce the spatial compatibility between robots.
	std::vector<Location*> fixedLocations;
};

/*!
 * Selected alternative, fixed locations, and the corresponding shortest path through locations.
 * \brief Shortest closed path through locations.
 */
struct CircuitRecord {
	CircuitRecord(const ShortestCircuit& arg1, const HamiltonianCircuit<Location>& arg2) : sc(arg1), hc(arg2) { }
	//! It enables to sort the circuits according to their lengths (shorter is better).
	bool operator<(const CircuitRecord& cr)	const {
		return hc.minLength < cr.hc.minLength;
	}

	//! Selected alternative and fixed locations.
	ShortestCircuit sc;
	//! Shortest closed path through locations, fixed locations are included.
	HamiltonianCircuit<Location> hc;
};

/* GENERATED TUPLE - POTENTIALLY FEASIBLE SOLUTION */

/*!
 * The structure stores a partially fixed problem, i.e. selected locations and movements (indirectly).
 * The power saving modes with the minimal delay time are selected before the optimization phase to find a feasible solution.
 * \brief A partially fixed problem, i.e. tuple.
 */
struct CircuitTuple {
	//! Tuple, i.e. a partially fixed problem.
	std::vector<CircuitRecord> tuple;
};

/* REPRESENTATION OF LP SOLUTION */

/*!
 * A partial problem (see \ref PartialSolution) is evaluated by Linear Programming
 * and the assigned timing is converted to this data structure.
 * The order of elements in vectors is determined by the order of operations.
 * The size of *Locs vectors is about one bigger than *Mvs ones
 * since the first and the last location are the same (closing activity).
 * \brief Obtained timing for a partial problem.
 */
struct OptimalTiming {
	//! Energy consumption of a robotic cell for this timing.
	double totalEnergy;
	//! Start times assigned to static activities' locations.
	std::vector<std::vector<double>> startLocs;
	//! Durations assigned to static activities' locations.
	std::vector<std::vector<double>> durLocs;
	//! Start times assigned to dynamic activities' movements.
	std::vector<std::vector<double>> startMvs;
	//! Durations assigned to dynamic activities' movements.
	std::vector<std::vector<double>> durMvs;

	//! %Activity to the assigned start time and duration.
	std::map<Activity*, std::pair<double, double>> actToStartAndDur;
	//! Selected location/movement to the assigned start time and duration.
	std::map<ActivityMode*, std::pair<double, double>> modeToStartAndDur;
};

/* COLLISION RESOLUTION */

/*!
 * \brief The structure stores how to resolve one collision between robots.
 * \see RoboticLineSolverLP::addCollisionResolution
 */
struct CollisionResolution	{
	//! Empty collision constructed -- \ref a1 and \ref a2 are null pointers.
	CollisionResolution() : a1(nullptr), a2(nullptr), multiplier(0), intersection(0.0) { }
	//! First activity involved in the collision.
	Activity *a1;
	//! Second activity involved in the collision.
	Activity *a2;
	//! Multiplier addressing the time shift in the number of cycles.
	int32_t multiplier;
	//! Duration of the collision between activities \ref a1 and \ref a2.
	double intersection;
};

/* POWER SAVING MODE SELECTION */

/*!
 * \brief Records a potential energy impact if a power saving mode of a robot is switched to another one.
 * \see HeuristicAlgorithms::heuristicPowerModeSelection
 */
struct ModeSwitchInfo {
	ModeSwitchInfo() = default;
	ModeSwitchInfo(uint32_t r, uint32_t l, Location* relLoc, RobotPowerMode* f, RobotPowerMode *t, double ee)
		: robotIdx(r), locationIdx(l), loc(relLoc), from(f), to(t), estimatedEnergy(ee) { }

	//! Sorting the switching alternatives according to the estimated energy consumption (less is better).
	bool operator<(const ModeSwitchInfo& msi) const {
		return estimatedEnergy < msi.estimatedEnergy;
	}

	//! Index of the robot.
	uint32_t robotIdx;
	//! Index of the related location of the robot.
	uint32_t locationIdx;
	//! A pointer to the considered location.
	Location *loc;
	//! The original power saving mode.
	RobotPowerMode *from;
	//! The candidate power saving mode to switch to.
	RobotPowerMode *to;
	//! Estimated energy consumption after switching to the candidate power saving mode.
	double estimatedEnergy;
};

/*!
 * \brief Structure encapsulates the time and energy properties of an activity with its selected mode (a movement or location).
 * \see HeuristicAlgorithms::heuristicPowerModeSelection, HeuristicAlgorithms::scaleGanttToCycleTime
 */
struct ActivityModeInfo {
	ActivityModeInfo(ActivityMode* m, double s, double d) : mode(m), curStartTime(s), curDuration(d) { }
	ActivityModeInfo(ActivityMode* m, RobotPowerMode* pm, double e, double s, double d, double minDur, double maxDur)
		: mode(m), pwrm(pm), energy(e), curStartTime(s), curDuration(d), minDuration(minDur), maxDuration(maxDur) { }

	//! Sort the activities according to start time, left to right direction in the Gantt.
	bool operator<(const ActivityModeInfo& ami) const {
		return curStartTime < ami.curStartTime;
	}

	//! Selected mode of an activity, i.e. movement or location.
	ActivityMode *mode;
	//! Power saving mode of the robot if it applies, i.e. \ref mode can be cast to Location.
	RobotPowerMode *pwrm;
	//! Energy required by the activity for the current duration.
	double energy;
	//! Current start time of the activity.
	double curStartTime;
	//! Current duration of the activity.
	double curDuration;
	//! Minimal possible duration of the activity for the given mode, robot, and power saving mode (if mode ~ location).
	double minDuration;
	//! Maximal possible duration of the activity limited by e.g. the robot, performed operation, etc.
	double maxDuration;
};

/* TABU LIST */

/*!
 * Tabu search meta-heuristic employs a tabu list, a list of recently performed modifications,
 * to avoid cycling in the solution space. The list is a short-term memory, i.e. the oldest
 * modifications are gradually replaced by the newer ones, that allows to intensify the searching process.
 * \brief A short-term memory, containing a list of forbidden moves, that mitigates the risk of cycling.
 * \see HeuristicAlgorithms::heuristicPowerModeSelection
 */
class TabuList {
	public:
		//! Constructs a tabu list with the fixed size.
		TabuList(const uint32_t& tabuListSize) : mListIdx(0u), mTabu(tabuListSize) { }

		/*!
		 * \param l A location where the selected power saving mode will be changed.
		 * \param from, to The power saving mode of the robot will be switched from \a from to \a to.
		 * \return Whether the modification is allowed.
		 * \brief It checks whether the modification of a partial solution is allowed, i.e. the move/modification is not tabu.
		 */
		bool isTabu(Location *l, RobotPowerMode *from, RobotPowerMode *to) const {
			return std::find(mTabu.cbegin(), mTabu.cend(), Element(l, from, to)) != mTabu.cend();
		}

		/*!
		 * \param l A location where the selected power saving mode was applied.
		 * \param from, to The power saving mode \a from was switched to \a to.
		 * \brief It adds a performed modification into the tabu list.
		 */
		void addTabu(Location *l, RobotPowerMode *from, RobotPowerMode *to) {
			assert(!isTabu(l, from, to) && "Adding already forbidden combination!");
			if (mListIdx < mTabu.size())	{
				mTabu[mListIdx++] = {l, from, to};
				mListIdx = (mListIdx % mTabu.size());
			} else {
				throw SolverException(caller(), "Cannot add an element to an empty tabu list!");
			}
		}

		/*!
		 * If the tabu list contains all the viable modifications, i.e. there is not a modification that
		 * could be applied, then some random tabu list elements are removed to allow the sub-heuristic to progress.
		 */
		void diversify() {
			uint32_t startIdx = mListIdx;
			int32_t toErase = (int32_t) ceil(0.3*mTabu.size());
			do {
				if (!mTabu[startIdx].isEmpty())	{
					mTabu[startIdx] = Element();
					--toErase;
				}

				startIdx = ((startIdx+1) % mTabu.size());

			} while (startIdx != mListIdx && toErase > 0);
		}
	private:
		/*!
		 * \brief An element of the tabu list.
		 */
		struct Element {
			Element() : loc(nullptr), m1(nullptr), m2(nullptr) { }
			Element(Location *l, RobotPowerMode *rm1, RobotPowerMode *rm2) : loc(l), m1(rm1), m2(rm2) { }

			//! Whether a tabu list element is empty, i.e. it does not contain a modification.
			bool isEmpty() const {
				return loc == nullptr && m1 == nullptr && m2 == nullptr;
			}
			//! The element is equal to the another one if all the data members are equal.
			bool operator==(const Element& e) const {
				return loc == e.loc && m1 == e.m1 && m2 == e.m2;
			}

			//! %Location where a power saving mode was changed.
			Location *loc;
			//! Original power saving mode that was replaced by another one.
			RobotPowerMode *m1;
			//! The power saving mode which replaced the original one.
			RobotPowerMode *m2;
		};

		//! Current index to the tabu list, i.e. a current write position.
		uint32_t mListIdx;
		//! Tabu list containing the forbidden modifications.
		std::vector<Element> mTabu;
};

/* MOVING AVERAGE */

/*!
 * \tparam T Numerical type.
 * \brief The class encapsulating the calculation of the moving average.
 * \see ParallelHeuristicSolver::workerThread
 */
template <class T>
class MovingAverage {
	public:
		/*!
		 * \param length The number of values considered in the running average.
		 * \brief It constructs an instance of the moving average.
		 */
		MovingAverage(uint32_t length) : mLength(length) {
			if (mLength <= 0)
				throw InvalidArgument(caller(), "A positive length is expected!");
		}

		//! Current value of the running average.
		double average() const {
			if (!mData.empty())
				return std::accumulate(mData.cbegin(), mData.cend(), T())/mData.size();
			else
				return T();
		}
		//! Add a new value to the moving average.
		void addValue(const T& val)	{
			if (mData.size() < mLength)	{
				mData.push_back(val);
			} else {
				mData.erase(mData.begin());
				mData.push_back(val);
			}
		}
		//! Returns whether the average is calculated from the full length.
		bool filled() const {
			return mData.size() == mLength;
		}
		//! Remove all the values from which the running average is calculated.
		void clear() {
			mData.clear();
		}
	private:
		//! The maximal number of values used for the calculation of the moving average.
		uint32_t mLength;
		//! The vector of values from which the running average is calculated.
		std::vector<T> mData;
};

/* USEFUL SHORTCUTS FOR COMPLEX DATA TYPES */

/*!
 * \addtogroup hash_def
 * @{
 */

namespace std {
	//! Template specialization of the hash for the ShortestCircuit data-structure.
	template <>
	struct hash<ShortestCircuit> {
		uintptr_t operator() (const ShortestCircuit& sc) const {
			uintptr_t hash1 = hashOW(sc.circuit);
			uintptr_t hash2 = hashEW(sc.fixedLocations);
			return hash1^hash2;
		}
	};
}

//! @}

//! Mapping of ShortestCircuit to the related shortest closed path through locations.
using PrecalculatedCircuits = std::unordered_map<ShortestCircuit, HamiltonianCircuit<Location>>;

#endif


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

#ifndef HLIDAC_PES_PARALLEL_SOLVER_H
#define HLIDAC_PES_PARALLEL_SOLVER_H

/*!
 * \file ParallelHeuristicSolver.h
 * \author Libor Bukata
 * \brief A parallel hybrid heuristic solving the energy optimization problem of robotic cells.
 */

#include <atomic>
#include <map>
#include <vector>
#include "Shared/Algorithms.h"
#include "HeuristicSolver/KnowledgeBase.h"
#include "HeuristicSolver/DataStructures.h"
#include "HeuristicSolver/HeuristicAlgorithms.h"
#include "ILPSolver/RoboticLineSolverLP.h"

/*!
 * A parallel hybrid heuristic that solves the energy optimization problem of robotic cells
 * is implemented in this class. At first the heuristic generates a candidate tuples,
 * i.e. a partially fixed problems, that could be resulting in initial feasible solutions.
 * If an initial solution is feasible then it is further optimized by sub-heuristics
 * that iteratively change the fixed locations and power saving modes with respect to the energy reasoning.
 * The best found solution is returned after a given time limit, or an exception is thrown if such a solution does not exist.
 * \brief A parallel heuristic for the energy optimization of robotic cells.
 * \see HeuristicAlgorithms, KnowledgeBase
 */
class ParallelHeuristicSolver {
	public:
		/*!
		 * \param l Robotic cell to be optimized.
		 * \param m A mapping calculated for the given robotic cell that enables fast searching.
		 * \brief It initializes the heuristic, i.e. preallocates some arrays, calculates the length of the tabu list, etc.
		 */
		ParallelHeuristicSolver(const RoboticLine& l, const PrecalculatedMapping& m);
		/*!
		 * \brief It optimizes the robotic cell and returns the best found solution.
		 * \see Settings
		 */
		Solution solve();
	private:
		/*!
		 * The control thread starts with the generation of alternatives, i.e. various orders of operations, for individual robots.
		 * Afterwards, the worker threads are launched to find good quality solutions. Meanwhile the control thread prints various
		 * statistical information about the heuristic performance and generates the promising tuples, i.e. partially fixed problems.
		 * After a given time limit, the worker threads are stopped and possible errors, which occurred in worker threads, are reported.
		 * \brief Generates various alternatives, launches worker threads, prints progress info, joins the workers, and handles errors of workers.
		 * \note Thread identification of the control thread is 0.
		 */
		void controlThread();
		/*!
		 * \param threadId Identification of the thread, i.e. a number from 1 to \a n where \a n is the total number of worker threads.
		 * \param initialCircuits Various alternatives (orders of operations) for each robot, indexed by robots.
		 * \param precalculatedCircuits ShortestCircuit to the fastest closed path through robot locations (including the fixed ones).
		 * \brief The worker thread generates initial tuples, and optimizes them by local modifications proposed by sub-heuristics.
		 * \details The worker thread generates candidate tuples, i.e. partially fixed problems that could be resulting in feasible solutions,
		 * to be optimized. First of all, the tuple is loaded and an initial timing can be extracted from a solution of Linear Programming.
		 * If the timing can be obtained, the optimization phase starts and the sub-heuristics iteratively modifies the tuple in a way
		 * that could be resulting in an energy reduction. All the feasible solutions are passed to KnowledgeBase, where the list of elite solutions is located.
		 * \see HeuristicAlgorithms, MovingAverage, OptimalTiming, Settings
		 */
		void workerThread(uint32_t threadId, std::vector<std::vector<CircuitRecord>> initialCircuits, PrecalculatedCircuits precalculatedCircuits);

		/*!
		 * \param r A robot for which a distance graph is created.
		 * \brief Generates a distance graph for the given robot.
		 * \details Generates a graph where edges are weighted by the minimal possible durations
		 * between nodes (static activities). A path from the start node to the end node
		 * that visits all the other nodes is a robot alternative.
		 */
		Graph constructMinimalDurationGraph(Robot* r) const;
		/*!
		 * \param g Distance graph of a robot.
		 * \return Initial two-dimensional matrix of distances between graph nodes.
		 * \brief It creates an initial matrix of distances based on edge lengths.
		 */
		DistanceMatrix<double> createInitialDistanceMatrix(const Graph& g) const;
		/*!
		 * \param threadId Thread identification, i.e. a number between 0 and n-1 where n is the total number of executed threads.
		 * \param g Distance graph of a robot for which random alternatives will be searched for.
		 * \param m Distance matrix calculated for the given graph.
		 * \return Some random alternatives.
		 * \brief Generates some random alternatives, i.e. candidates for a feasible order, from a distance graph and distance matrix of a robot.
		 */
		std::vector<CircuitRecord> generateRandomAlternatives(const uint32_t& threadId, const Graph& g, const DistanceMatrix<double>& m);
		/*!
		 * \param threadId Thread identification, i.e. a number between 0 and n-1 where n is the total number of executed threads.
		 * \param circuit Closed path through static activities where each static activity of a robot is visited.
		 * \param fixed Locations that must be visited to satisfy the spatial compatibility.
		 * \param writeCircuit Whether the shortest circuit through locations should be written.
		 * \return The shortest circuit through locations (if writeCircuit = true) and its minimal length.
		 * \brief It calculates the shortest circuit through locations that visits each static activity in the same order as it is in \a circuit parameter.
		 */
		HamiltonianCircuit<Location> getShortestCircuit(const uint32_t& threadId, const std::vector<StaticActivity*>& circuit,
				const std::vector<Location*>& fixed, bool writeCircuit = true);
		/*!
		 * \param precalculatedCircuits ShortestCircuit to the fastest closed path through robot locations (including the fixed ones).
		 * \return Random alternatives and their shortest circuits through robot locations for each robot, indexed by robot.
		 * \brief It finds random alternatives for each robot and calculates their shortest closed paths through locations.
		 */
		std::vector<std::vector<CircuitRecord>> generateShortestCircuits(PrecalculatedCircuits& precalculatedCircuits);

		/*!
		 * \param minCycle A lower bound on the robot cycle time.
		 * \param demandedCycleTime  A desired robot cycle time.
		 * \return A penalty value quantifying the risk of exceeding the demanded robot cycle time.
		 * \brief The function expressing the risk of exceeding the robot cycle time depending on its lower estimation.
		 * \see generateShortestCircuits, addRandomTuplesToKB
		 */
		static double penaltyFunction(const double& minCycle, const double& demandedCycleTime);

		/*!
		 * \param alternatives Alternatives of a robot.
		 * \return Index of the selected alternative (the order of operations).
		 * \brief It randomly selects a robot alternative according to a distribution function
		 * that slightly prefers the alternatives with lower estimation of the robot cycle time.
		 */
		uint32_t selectAlternative(const std::vector<CircuitRecord>& alternatives) const;
		/*!
		 * \param threadId Thread identification, i.e. a number between 0 and n-1 where n is the total number of executed threads.
		 * \param currentCycleTime Current lower bound on the robot cycle time for the given alternative and fixed locations.
		 * \param alternative Considered robot alternative, i.e. the order of static activities.
		 * \param fixed Locations that should be fixed to enforce the spatial compatibility.
		 * \param toFix A new location to be fixed, it replaces another one in \a fixed vector.
		 * \param precalculatedCircuits ShortestCircuit to the fastest closed path through robot locations (including the fixed ones).
		 * \return A lower estimation on the robot cycle time if \a toFix location is fixed instead of another one in \a fixed vector.
		 * \brief A highly optimized function calculates a lower estimation on the robot cycle time if \a toFix location is newly fixed.
		 */
		double calculateNewCycleTime(const uint32_t& threadId, const double& currentCycleTime, std::vector<StaticActivity*>& alternative,
				std::vector<Location*>& fixed, Location *toFix, PrecalculatedCircuits& precalculatedCircuits);
		/*!
		 * \param threadId Thread identification, i.e. a number between 0 and n-1 where n is the total number of executed threads.
		 * \param numOfTuples The number of candidate tuples, i.e. partially fixed problems, to be generated.
		 * \param initialCircuits Alternatives and the related fixed locations and shortest paths through locations, indexed by robot.
		 * \param precalculatedCircuits ShortestCircuit to the fastest closed path through robot locations (including the fixed ones).
		 * \brief It generates and adds a requested number of tuples to the KnowledgeBase.
		 */
		void addRandomTuplesToKB(const uint32_t& threadId, const uint32_t& numOfTuples,
				std::vector<std::vector<CircuitRecord>>& initialCircuits, PrecalculatedCircuits& precalculatedCircuits);
		/*!
		 * \param threadId Thread identification. Currently only the control thread (with id 0) generates the promising tuples.
		 * \brief It generates promising tuples and adds them to the KnowledgeBase.
		 */
		void generatePromisingTuples(const uint32_t& threadId);

		/*!
		 * \param threadId Thread identification, i.e. a number between 1 and n where n is the total number of worker threads.
		 * \param t A partially fixed problem to be modified, i.e. tuple.
		 * \param ps Fixed locations, movements, and power saving modes.
		 * \param fixed Currently fixed locations.
		 * \return A lower bound on a robot cycle time after changing robot paths.
		 * \brief It randomly modifies the robot paths to enable the heuristic to search otherwise unreachable parts of the solution space.
		 */
		double changeRobotPaths(uint32_t threadId, const CircuitTuple& t, PartialSolution& ps, const std::vector<std::vector<Location*>>& fixed);

		/*!
		 * \param currentRuntime Current execution time of the heuristic.
		 * \brief Prints various information about the performance of the sub-heuristics,
		 * generation of tuples, and solving reduced problems by Linear Programming.
		 */
		void printProgressInfo(const double& currentRuntime);
		/*!
		 * \param initializationTime The time needed for the generation of robot alternatives and their shortest circuits through locations.
		 * \param finalRuntime The total real time required by the heuristic.
		 * \brief It writes various statistics about the heuristic performance to a CSV file.
		 */
		void writePerformanceRecordToLogFile(const double& initializationTime, const double& finalRuntime);

		//! The runtime of this heuristic, updated from time to time.
		double mRuntime;
		//! The number of tabu list elements calculated in the constructor.
		uint32_t mTabuListSize;

		//! Used to stop other OpenMP threads if the instance infeasibility is detected by a thread.
		std::atomic<bool> mFeasible;
		//! The control thread sets this flag to true to stop all the worker threads.
		std::atomic<bool> mStopCondition;

		//! Robotic cell to be optimized.
		RoboticLine mLine;
		//! Various data of the heuristic, e.g. generated tuples, performance statistics, etc.
		KnowledgeBase mKB;
		//! Fast searching in the data structure of the robotic cell.
		PrecalculatedMapping mMapping;
		//! Optimization sub-heuristics and the related Linear Programming solver for partially fixed problems.
		HeuristicAlgorithms algo;

		//! Preallocated 2-D array of distances for \ref getShortestCircuit method, indexed by thread.
		std::vector<std::vector<std::vector<double>>> mDist;
		//! Preallocated 2-D array of predecessors for \ref getShortestCircuit method, indexed by thread.
		std::vector<std::vector<std::vector<int64_t>>> mPreds;
		//! Preallocated 2-D array of locations for \ref getShortestCircuit method, indexed by thread.
		std::vector<std::vector<std::vector<Location*>>> mLocs;

		//! The vector of spatial compatibility pairs heuristically sorted in decreasing order of difficulty.
		std::vector<SpatialCmpTuple> mSortedSptCmp;
};

#endif

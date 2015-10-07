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

#ifndef HLIDAC_PES_KNOWLEDGE_BASE_H
#define HLIDAC_PES_KNOWLEDGE_BASE_H

/*!
 * \file KnowledgeBase.h
 * \author Libor Bukata
 * \brief Declares the class storing the best found solutions, tuples, and information about the heuristic performance.
 */

#include <algorithm>
#include <atomic>
#include <cassert>
#include <list>
#include <mutex>
#include <queue>
#include <set>
#include <vector>
#include <utility>
#include "RoboticLine.h"
#include "Solution/Solution.h"
#include "Shared/PrecalculatedMapping.h"
#include "HeuristicSolver/DataStructures.h"

/*!
 * Instance of the class stores the best found solutions, generated tuples,
 * and various statistics related to the heuristic performance.
 * \brief Protected access to the shared data of the threads of the parallel heuristic.
 */
class KnowledgeBase {
	public:
		KnowledgeBase();

		//! Returns a list of elite solutions and their tuples from which they have been calculated.
		std::list<std::pair<Solution, CircuitTuple>> eliteSolutions();
		//! Returns the best found solution, throws an exception if not available.
		Solution bestSolution();
		/*!
		 * \param s A candidate for an elite solution.
		 * \param t Tuple from which the solution was generated.
		 * \brief If a found solution ranks among the top ones, it is added to the list of elite solutions.
		 */
		void candidate(const Solution& s, const CircuitTuple& t);

		//! Add a tuple to the mutex protected queue of tuples.
		void addTuple(CircuitTuple&& t);
		//! Add an error message to the mutex protected vector of messages.
		void addErrorMessage(const std::string& msg);
		//! Returns all the error messages that occurred in the threads of the heuristic.
		std::vector<std::string> errorMessages() const { return mErrorMessages; }
		//! Returns a partially fixed solution called tuple.
		CircuitTuple getTuple();

		//! Reports that it was not possible to obtain a feasible solution from the generated tuple.
		void reportInfDueToLP() { ++mInfDueToLP; }
		//! Returns how many times it was not possible to obtain an initial solution, i.e. timing.
		uint64_t numberOfLPBreaks() const { return mInfDueToLP; }
		//! It reports that changes of \ref HeuristicAlgorithms::heuristicLocationChanges sub-heuristic caused infeasibility.
		void reportInfDueToLocHeur() { ++mInfDueToLocHeur; }
		//! Returns how many times the change locations sub-heuristic caused the infeasibility of the solution.
		uint64_t numberOfLocHeurBreaks() const { return mInfDueToLocHeur; }
		//! It reports that a power mode change of \ref HeuristicAlgorithms::heuristicPowerModeSelection sub-heuristic caused infeasibility.
		void reportInfDueToPwrmHeur() { ++mInfDueToPwrmHeur; }
		//! Returns how many times the (de)select power mode sub-heuristic caused infeasibility.
		uint64_t numberOfPwrmHeurBreaks() const { return mInfDueToPwrmHeur; }
		//! Reports that a diversification change of paths resulted in an infeasible solution.
		void reportInfDueToPathChange() { ++mInfDueToChangedPath; }
		//! Returns how many times the change of paths resulted in infeasibility.
		uint64_t numberOfChangePathBreaks() const { return mInfDueToChangedPath; }

		//! Reports that the solution of the Linear Programming problem was infeasible (one call of LP).
		void recordInfeasibleLP();
		//! Infeasibility rate of Linear Programming, i.e. the proportion of infeasible solutions to feasible and infeasible ones.
		double infeasibilityRate();
		//! Reports a real time (measured by a timer) required for solving a partially fixed problem by Linear Programming.
		void recordLPCall(double runtime);
		//! Returns the total number of Linear Programming calls and its average runtime for one worker thread.
		std::pair<uint64_t, double> infoLP();
		//! Reports a relative deterioration in the solution quality caused by the resolution of collisions.
		void recordLPFixDeterioration(double deterioration);
		//! Returns an average quality deterioration caused by the collisions resolution.
		double averageLPFixDeterioration();
		//! Returns an average number of added precedences required for collisions avoidance.
		double averageNumberOfLPCallsForLPFix();
		//! Reports that \ref HeuristicAlgorithms::solvePartialProblem method was called.
		void recordPartialProblemSolveCall();

		//! Reports a real time (measured by a timer) required by \ref HeuristicAlgorithms::heuristicLocationChanges sub-heuristic per call.
		void recordLocHeurCall(double runtime);
		//! Returns the total number of \ref HeuristicAlgorithms::heuristicLocationChanges calls and average runtime for one worker thread.
		std::pair<uint64_t, double> infoLocHeur();
		//! Reports a relative estimation error of the change locations sub-heuristic for an energy improvement.
		void recordLocHeurRelErr(double relativeEstError);
		//! Returns an average estimation error of the change locations sub-heuristic.
		double averageErrOfLocHeur();

		//! Reports a real time (measured by a timer) required by \ref HeuristicAlgorithms::heuristicPowerModeSelection sub-heuristic per call.
		void recordPwrmHeurCall(double runtime);
		//! Returns the total number of \ref HeuristicAlgorithms::heuristicPowerModeSelection calls and average runtime for one worker thread.
		std::pair<uint64_t, double> infoPwrmHeur();
		//! Reports a relative estimation error of the (de)select power mode sub-heuristic for an energy improvement.
		void recordPwrmHeurRelErr(double relativeEstError);
		//! Returns an average estimation error of the (de)select power mode sub-heuristic.
		double averageErrOfPwrmHeur();

		//! Reports that robot paths were diversified, i.e. \ref ParallelHeuristicSolver::changeRobotPaths method was called.
		void recordChangePathCall() { ++mPathChangeCalls; }
		//! The total number of \ref ParallelHeuristicSolver::changeRobotPaths calls.
		uint64_t pathChangeCalls() const { return mPathChangeCalls; }

		//! Reports a real time (measured by a timer) required for \ref ParallelHeuristicSolver::addRandomTuplesToKB call.
		void recordAddTuplesCall(double runtime);
		//! Returns the total number of generated tuples and the average generation time for one worker thread.
		std::pair<uint64_t, double> infoTuplesGeneration();
		//! Reports the number of optimization iterations performed for a loaded tuple.
		void recordNumberOfItersPerTuple(uint64_t iters);
		//! Returns the total number of tuples used for optimization and the average number of optimization iterations per tuple.
		std::pair<uint64_t, double> infoItersPerTuple();
		//! Percentage of the processed tuples.
		double percentageOfProcessed();
	private:

		/*!
		 * \tparam T Numerical type.
		 * \param value Value to be added to \a addTo.
		 * \param addTo Accumulated value.
		 * \brief It carries out "addTo += value" operation, however protected by a mutex.
		 */
		template <class T>
		void record(T value, T& addTo);

		/*!
		 * \tparam T Numerical type.
		 * \param value Value to be added to \a addTo.
		 * \param addTo Accumulated value.
		 * \param counter Counter which somehow relates with the previous parameters.
		 * \brief Performs "addTo += value" operation and increases the value of \a counter by one.
		 * \note The method is thread-safe.
		 */
		template <class T>
		void record(T value, T& addTo, std::atomic<uint64_t>& counter);

		/*!
		 * \param aggregatedValue A value with e.g. aggregated time of calls.
		 * \param counter The number of e.g. calls.
		 * \return A pair { counter, aggregatedValue/counter } if counter > 0.
		 * \brief Auxiliary method used for the calculation of an average time of a call based on a counter value and the total time.
		 */
		std::pair<uint64_t, double> getInfo(double& aggregatedValue, std::atomic<uint64_t>& counter);

		//! Queue with the generated tuples.
		std::queue<CircuitTuple> mTuples;
		//! Vector containing error messages of worker threads (to throw an exception later).
		std::vector<std::string> mErrorMessages;
		//! List of elite solutions.
		std::list<std::pair<Solution, CircuitTuple>> mEliteSolutions;

		//! Total number of generated tuples.
		std::atomic<uint64_t> mAddedTuples;
		//! The number of processed tuples.
		std::atomic<uint64_t> mProcessedTuples;
		//! Records the number of tuples coming to the optimization process of the heuristic.
		std::atomic<uint64_t> mOptimizationPhaseCounter;

		//! How many times it was not possible to obtain feasible timing for a tuple.
		std::atomic<uint64_t> mInfDueToLP;
		//! The number of feasibility breakings caused by change locations sub-heuristic.
		std::atomic<uint64_t> mInfDueToLocHeur;
		//! The number of feasibility breakings caused by (de)select power mode sub-heuristic.
		std::atomic<uint64_t> mInfDueToPwrmHeur;
		//! The number of feasibility breakings caused by the path diversification.
		std::atomic<uint64_t> mInfDueToChangedPath;
		//! How many times a Linear Programming solver returned an infeasible solution.
		std::atomic<uint64_t> mInfeasibleCounter;

		//! The number of Linear Programming calls.
		std::atomic<uint64_t> mLPCalls;
		//! How many times a partially fixed problem was successfully solved.
		std::atomic<uint64_t> mLPFixCalls;
		//! How many times a partially fixed problem was evaluated, i.e. the number of calls of \ref HeuristicAlgorithms::solvePartialProblem.
		std::atomic<uint64_t> mPartProbCalls;
		//! The number of change locations sub-heuristic calls.
		std::atomic<uint64_t> mLocHeurCalls;
		//! The number of (de)select power mode sub-heuristic calls.
		std::atomic<uint64_t> mPwrmHeurCalls;
		//! The number of robot path diversifications, i.e. the number of calls of \ref ParallelHeuristicSolver::changeRobotPaths.
		std::atomic<uint64_t> mPathChangeCalls;
		//! The total number of optimization iterations.
		std::atomic<uint64_t> mSumOfIters;

		//! Aggregated processor time of all the Linear Programming calls.
		double mLPTime;
		//! Aggregated processor time of all the change locations sub-heuristic calls.
		double mLocHeurTime;
		//! Aggregated processor time of all the (de)select power mode sub-heuristic calls.
		double mPwrmHeurTime;
		//! Aggregated processor time required for the generation of all the tuples.
		double mTupleGenTime;
		//! The sum of relative deteriorations of the energy consumption caused by the collision resolution.
		double mSumOfDeteriorations;
		//! The sum of the relative estimation errors (calculated by the change locations sub-heuristic) of energy improvements.
		double mSumOfLocHeurErrs;
		//! The sum of the relative estimation errors (calculated by the (de)select power mode sub-heuristic) of energy improvements.
		double mSumOfPwrmHeurErrs;

		//! Mutex protecting the access to the list of elite solutions.
		std::mutex mEliteMtx;
		//! Mutex protecting the access to the queue that contains tuples.
		std::mutex mTuplesMtx;
		//! Mutex protecting the access to the vector of error messages.
		std::mutex mErrorsMtx;
		//! Mutex ensuring the flawless float operations for multiple concurrent threads.
		std::mutex mStatMtx;
};

#endif

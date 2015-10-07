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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>
#include <set>
#include <random>
#include <thread>
#include <vector>
#include <unordered_map>
#include "SolverConfig.h"
#include "Shared/NumericConstants.h"
#include "Shared/Utils.h"
#include "HeuristicSolver/ParallelHeuristicSolver.h"

using namespace std;
using namespace std::chrono;

static random_device rd;

ParallelHeuristicSolver::ParallelHeuristicSolver(const RoboticLine& l, const PrecalculatedMapping& m) : mLine(l), mMapping(m), algo(mKB, mLine, mMapping) {
	mTabuListSize = 0u;
	uint32_t maxStaticActivities = 0u, maxLocations = 0u;
	for (Robot* r : l.robots())	{
		uint32_t numberOfStaticActivities = 0u, numberOfLocations = 0u, numberOfPowerModes = r->powerModes().size();
		for (Activity *a : r->activities())	{
			StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr)	{
				uint32_t numberOfActivityLocations = sa->locations().size();
				maxLocations = max(maxLocations, numberOfActivityLocations);
				numberOfLocations += numberOfActivityLocations;
				++numberOfStaticActivities;
			}
		}

		maxStaticActivities = max(maxStaticActivities, numberOfStaticActivities);
		mTabuListSize += numberOfLocations*numberOfPowerModes;
	}

	mDist.resize(Settings::NUMBER_OF_THREADS+1, vector<vector<double>>(maxStaticActivities+1, vector<double>(maxLocations, F64_INF)));
	mPreds.resize(Settings::NUMBER_OF_THREADS+1, vector<vector<int64_t>>(maxStaticActivities+1, vector<int64_t>(maxLocations, -1)));
	mLocs.resize(Settings::NUMBER_OF_THREADS+1, vector<vector<Location*>>(maxStaticActivities+1, vector<Location*>(maxLocations, nullptr)));

	mTabuListSize = (uint32_t) ceil(0.1*mTabuListSize);
}

Solution ParallelHeuristicSolver::solve()	{
	Solution s;
	try {
		controlThread();
		s = mKB.bestSolution();
		s.status = FEASIBLE;
	} catch (NoFeasibleSolutionExists& e)	{
		s.status = INFEASIBLE;
	} catch (EmptySolutionPool& e)	{
		s.status = UNKNOWN;
	} catch (...)	{
		throw_with_nested(SolverException(caller(), "Cannot get the solution, an exception occurred!"));
	}

	s.runTime = mRuntime;

	return s;
}

void ParallelHeuristicSolver::controlThread()	{
	// Set constants and the feasible flag.
	mRuntime = 0.0;
	mFeasible = true;
	mStopCondition = false;
	// Start timing...
	time_point<system_clock, duration<double>> startTime = high_resolution_clock::now();
	time_point<system_clock, duration<double>> expectedFinishTime = startTime+duration<double>(Settings::MAX_RUNTIME);
	// Initial phase of the heuristic.
	PrecalculatedCircuits precalculatedCircuits(1024);
	vector<vector<CircuitRecord>> initialCircuits = generateShortestCircuits(precalculatedCircuits);
	if (mFeasible == false)
		throw NoFeasibleSolutionExists(caller(), "The problem was proved to be infeasible!");

	double initializationTime = duration_cast<duration<double>>(high_resolution_clock::now()-startTime).count();
	if (Settings::VERBOSE)
		clog<<endl<<"Initial phase: "<<initializationTime<<" s"<<endl;

	#ifdef GUROBI_SOLVER
	initializeLocalEnvironments(Settings::NUMBER_OF_THREADS);
	#endif

	vector<thread> threads;
	for (uint32_t t = 0; t < Settings::NUMBER_OF_THREADS; ++t)
		threads.emplace_back(&ParallelHeuristicSolver::workerThread, this, t+1, initialCircuits, precalculatedCircuits);

	duration<double> timeToNextWakeUp = duration<double>(max(Settings::MAX_RUNTIME/10.0, 5.0));
	while (mStopCondition == false)	{
		duration<double> remainingTimeToWait = expectedFinishTime-high_resolution_clock::now();
		if (remainingTimeToWait > timeToNextWakeUp)
			remainingTimeToWait = timeToNextWakeUp;

		this_thread::sleep_for(remainingTimeToWait);

		mRuntime = duration_cast<duration<double>>(high_resolution_clock::now()-startTime).count();
		if (mRuntime > Settings::MAX_RUNTIME)	{
			mStopCondition = true;
		} else	{
			try {
				if (Settings::VERBOSE)
					printProgressInfo(mRuntime);

				generatePromisingTuples(0u);	// threadId of the control thread is 0
			} catch (exception& e)	{
				mKB.addErrorMessage(e.what());
				mStopCondition = true;
			}
		}
	}

	// do stuff - util wait condition is satisfied...
	for (thread& th : threads)
		th.join();

	// Get the final heuristic runtime.
	mRuntime = duration_cast<duration<double>>(high_resolution_clock::now()-startTime).count();
	writePerformanceRecordToLogFile(initializationTime, mRuntime);
	if (Settings::VERBOSE)
		cout<<"Total time: "<<mRuntime<<" s"<<endl;

	const vector<string>& msgs = mKB.errorMessages();
	if (!msgs.empty())	{
		string bigMessage = "\n";
		for (uint32_t m = 0; m < msgs.size(); ++m)	{
			bigMessage += msgs[m];
			if (m+1 < msgs.size())
				bigMessage += "\n\n";
		}

		throw SolverException(caller(), bigMessage);
	}
}

void ParallelHeuristicSolver::workerThread(uint32_t threadId, vector<vector<CircuitRecord>> initialCircuits, PrecalculatedCircuits precalculatedCircuits)	{
	try {
		TabuList tabuList(mTabuListSize);
		const uint32_t numOfTuplesBatch = 50u;

		while (!mStopCondition) {
			/* INITIAL PHASE - FEASIBILITY PROBLEM */
			CircuitTuple t;
			PartialSolution ps;
			OptimalTiming timing;
			vector<vector<Location*>> fixed;
			Algo selAlgo = LP_INITIAL_PROBLEM;
			try {
				// A fresh tuple to solve.
				t = mKB.getTuple();
				// Initialize the partial solution suitable for the LP solver.
				for (const CircuitRecord& cr : t.tuple)
					ps.locs.push_back(cr.hc.circuit);
				fixed = algo.initializePartialSolution(ps);
				// Solve the LP problem to optimality with the collision resolution.
				timing = algo.solvePartialProblem(ps, t, selAlgo);
			} catch (EmptySolutionPool& e)	{
				addRandomTuplesToKB(threadId, numOfTuplesBatch, initialCircuits, precalculatedCircuits);
				continue;
			} catch (NoFeasibleSolutionExists& e)	{
				// Try another tuple. This tuple is infeasible probably...
				continue;
			}

			/* FINAL PHASE - LOCAL OPTIMIZATION OF THE PROBLEM */
			selAlgo = POWER_MODE_HEURISTIC;
			PartialSolution prevPartSol = ps;
			OptimalTiming prevOptTiming = timing;
			bool readNextTuple = false, solutionChanged = false;
			const uint32_t lpLength = Settings::MIN_ITERS_PER_TUPLE;
			constexpr uint32_t locHeurLength = 3, pwrmHeurLength = 5;
			uint32_t numberOfIter = 0, infeasibilityConsecutiveCounter = 0;
			double energyDiffEstimation = 0.0, bestCriterion = timing.totalEnergy;
			MovingAverage<double> imprLocChg(locHeurLength), imprPwrmChg(pwrmHeurLength), imprLP(lpLength);
			do	{
				switch (selAlgo)	{
					case POWER_MODE_HEURISTIC:
						energyDiffEstimation = algo.heuristicPowerModeSelection(timing, ps, tabuList, solutionChanged);
						break;
					case LOCATION_CHANGE_HEURISTIC:
						energyDiffEstimation = algo.heuristicLocationChanges(timing, ps, fixed, solutionChanged);
						break;
					case PATH_CHANGE:
						changeRobotPaths(threadId, t, ps, fixed);
						fixed = algo.initializePartialSolution(ps);
						imprLocChg.clear(); imprPwrmChg.clear();
					default:
						break;
				}

				bool solutionInfeasible = false;
				double relativeImprovement = 0.0, heurRelEstError = 0.0;
				try {
					double previousCriterion = timing.totalEnergy;
					timing = algo.solvePartialProblem(ps, t, selAlgo);
					bestCriterion = min(bestCriterion, timing.totalEnergy);
					double energyDiffExact = previousCriterion-timing.totalEnergy;
					relativeImprovement = max(bestCriterion-timing.totalEnergy, 0.0)/(timing.totalEnergy+F64_EPS);
					prevPartSol = ps; prevOptTiming = timing;
					imprLP.addValue(relativeImprovement);

					if (abs(energyDiffExact) > F64_EPS)
						heurRelEstError = abs(energyDiffEstimation-energyDiffExact)/abs(energyDiffExact);

					infeasibilityConsecutiveCounter = 0u;
					readNextTuple = (imprLP.filled() && imprLP.average() < CRITERION_RTOL);
				} catch (NoFeasibleSolutionExists& e)	{
					// The solution was made infeasible by heuristic changes, restore the previous solution and switch to the next heuristic.
					solutionInfeasible = true;
					ps = prevPartSol; timing = prevOptTiming;
					if (infeasibilityConsecutiveCounter >= 4)
						readNextTuple = true;

					++infeasibilityConsecutiveCounter;
				}

				switch (selAlgo)	{
					case POWER_MODE_HEURISTIC:
						mKB.recordPwrmHeurRelErr(heurRelEstError);
						imprPwrmChg.addValue(relativeImprovement);
						if ((imprPwrmChg.filled() && imprPwrmChg.average() < CRITERION_RTOL) || !solutionChanged || solutionInfeasible)
							selAlgo = LOCATION_CHANGE_HEURISTIC;
						break;
					case LOCATION_CHANGE_HEURISTIC:
						mKB.recordLocHeurRelErr(heurRelEstError);
						imprLocChg.addValue(relativeImprovement);
						if ((imprLocChg.filled() && imprLocChg.average() < CRITERION_RTOL) || !solutionChanged || solutionInfeasible)
							selAlgo = PATH_CHANGE;
						break;
					case PATH_CHANGE:
						selAlgo = POWER_MODE_HEURISTIC;
					default:
						break;
				}

				++numberOfIter;

			} while (!readNextTuple && !mStopCondition);

			mKB.recordNumberOfItersPerTuple(numberOfIter);
		}
	} catch (exception& e)	{
		mKB.addErrorMessage(exceptionToString(e, 1));
	}
}

Graph ParallelHeuristicSolver::constructMinimalDurationGraph(Robot* r) const {
	uint32_t id = 0;
	vector<StaticActivity*> idToActivity;
	map<StaticActivity*, uint32_t> ident;
	StaticActivity *home = nullptr;

	for (Activity* a : r->activities())	{
		StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
		if (sa != nullptr)	{
			setValue(ident, sa, id++, caller());
			idToActivity.push_back(sa);

			if (sa->lastInCycle())
				home = sa;
		}
	}

	assert(home != nullptr && "Invalid checking of the correctness of instances!");

	uint32_t startId = id++, endId = getValue(ident, home, caller());
	idToActivity.push_back(home);

	vector<Edge> graphEdges;
	for (Activity *s : home->successors())	{
		DynamicActivity *da = dynamic_cast<DynamicActivity*>(s);
		assert(da != nullptr && "Logic error, the successor of the static activity must be a dynamic activity!");
		if (da != nullptr)	{
			StaticActivity *to = da->successor();
			graphEdges.push_back({home, to, startId, getValue(ident, to, caller()), home->minAbsDuration()+da->minAbsDuration()});
		}
	}

	for (Activity* a : r->activities())	{
		DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr && da->predecessor() != home)	{
			StaticActivity *from = da->predecessor(), *to = da->successor();
			graphEdges.push_back({from, to, getValue(ident, from, caller()), getValue(ident, to, caller()), from->minAbsDuration()+da->minAbsDuration()});
		}
	}

	vector<vector<Edge>> idToSucs(idToActivity.size());
	for (const Edge& e : graphEdges)	{
		assert(e.i < idToActivity.size() && "Invalid generation of identifications!");
		idToSucs[e.i].push_back(e);
	}

	return {graphEdges, startId, endId, idToActivity, idToSucs};
}

DistanceMatrix<double> ParallelHeuristicSolver::createInitialDistanceMatrix(const Graph& g) const	{
	size_t matrixSize = g.idToActivity.size();
	DistanceMatrix<double> mat(matrixSize, vector<double>(matrixSize, F64_INF));
	for (const Edge& e : g.edges)	{
		assert(e.i < matrixSize && e.j < matrixSize && "Invalid creation of the graph!");
		mat[e.i][e.j] = e.length;
	}

	for (uint32_t i = 0; i < matrixSize; ++i)
		mat[i][i] = 0.0;

	return mat;
}

vector<CircuitRecord> ParallelHeuristicSolver::generateRandomAlternatives(const uint32_t& threadId, const Graph& g, const DistanceMatrix<double>& m) {

	bool allFound = false;
	set<vector<uint32_t>> forbiddenPaths;
	default_random_engine threadGenerator(rd());
	vector<CircuitRecord> shortestAlternatives;
	double cycleTime = mLine.productionCycleTime();
	StaticActivity *home = g.idToActivity[g.startIdent];
	const uint32_t maxCircuits = Settings::MAX_ALTERNATIVES;
	uint64_t numOfNodes = g.idToSuccessors.size(), numOfSol = 0;

	while (allFound != true && numOfSol < maxCircuits && mFeasible)	{

		set<uint32_t> visited;
		vector<double> edgeDistances;
		uint32_t currentNode = g.startIdent;
		vector<StaticActivity*> path = { home };
		vector<uint32_t> nodePath = { currentNode };

		while ((currentNode != g.endIdent || nodePath.size() != numOfNodes) && allFound == false)	{

			bool backtrack = false;

			// Calculate the current length of the partial path.
			double pathLength = accumulate(edgeDistances.cbegin(), edgeDistances.cend(), 0.0);

			// Check the reachability from currentNode to unvisited nodes.
			for (uint32_t n = 0; n < numOfNodes && !backtrack; ++n)	{
				if (visited.count(n) == 0 && pathLength+m[currentNode][n]+m[n][g.endIdent] > cycleTime)
					backtrack = true;
			}

			if (backtrack == false)	{
				// Find eligible edges from the current node to successor nodes.
				vector<Edge> feasEdges;
				for (const Edge& e : g.idToSuccessors[currentNode])	{
					nodePath.push_back(e.j);

					if (visited.count(e.j) == 0 && forbiddenPaths.count(nodePath) == 0)
						feasEdges.push_back(e);

					nodePath.pop_back();
				}

				if (!feasEdges.empty())	{
					// It selects a random edge to move.
					uniform_int_distribution<uint32_t> edgeSelection(0, feasEdges.size()-1);
					const Edge& edge = feasEdges[edgeSelection(threadGenerator)];
					path.push_back(edge.t); nodePath.push_back(edge.j);
					edgeDistances.push_back(edge.length);
					visited.insert(edge.i); currentNode = edge.j;
				} else {
					// Cannot expand the node -> backtrack.
					backtrack = true;
				}
			}

			// Backtracking - add a new forbidden path and remove the last node.
			if (backtrack == true)	{
				assert(path.size() == nodePath.size() && "Expected the same size! A bug in the algorithm.");
				if (nodePath.size() >= 2)	{
					const vector<uint32_t>& prefixPath = nodePath;
					auto p = forbiddenPaths.emplace(nodePath.cbegin(), nodePath.cend());
					set<vector<uint32_t>>::iterator it = p.first, eraseStart = ++it, eraseStop = eraseStart;
					while (it != forbiddenPaths.end())	{
						const vector<uint32_t>& nextPath = *it;
						if (prefixPath.size() < nextPath.size())	{
							const auto& mitp = mismatch(prefixPath.cbegin(), prefixPath.cend(), nextPath.cbegin());
							if (mitp.first == prefixPath.end())
								eraseStop = ++it;
							else
								break;
						} else {
							break;
						}
					}
					forbiddenPaths.erase(eraseStart, eraseStop);
					visited.erase(nodePath.back());
					path.pop_back(); nodePath.pop_back(); edgeDistances.pop_back();
					currentNode = nodePath.back();
				} else {
					// All hamiltonian paths found...
					allFound = true;
				}
			}
		}

		// Save the found hamiltonian path...
		if (currentNode == g.endIdent)	{
			HamiltonianCircuit<Location> cl = getShortestCircuit(threadId, path, {});	// closed path - penalty free
			if (cl.minLength >= 0.0 && cl.minLength <= cycleTime)	{
				ShortestCircuit sc;
				sc.circuit = move(path);
				shortestAlternatives.emplace_back(move(sc), move(cl));
				++numOfSol;
			}

			forbiddenPaths.insert(nodePath);
		}
	}

	return shortestAlternatives;
}

HamiltonianCircuit<Location> ParallelHeuristicSolver::getShortestCircuit(const uint32_t& threadId, const vector<StaticActivity*>& circuit, const vector<Location*>& fixed, bool writeCircuit) {

	double bestDist = F64_INF;
	vector<vector<int64_t>> bestPreds;
	vector<vector<Location*>> bestLocs;
	uint32_t closedPathSize = circuit.size();
	double cycleTime = mLine.productionCycleTime();
	const double penaltyForViolation = cycleTime+1.0;

	assert(circuit.size() >= 2 && "Unexpected empty path! Invalid algorithm!");

	vector<StaticActivity*>::const_iterator bestStartIter = min_element(circuit.cbegin(), circuit.cend(),
			[](StaticActivity* sa1, StaticActivity* sa2) {
				return sa1->locations().size() < sa2->locations().size();
			}
	);

	vector<StaticActivity*> closedPath;
	closedPath.reserve(closedPathSize);
	StaticActivity *startAndEndAct = *bestStartIter;
	if (startAndEndAct != circuit.front())	{
		closedPath.insert(closedPath.end(), bestStartIter, circuit.cend());
		closedPath.insert(closedPath.end(), circuit.cbegin()+1, bestStartIter+1);
	} else {
		closedPath.insert(closedPath.end(), circuit.cbegin(), circuit.cend());
	}

	assert(closedPath.size() == circuit.size() && "Invalid rotation of the circuit, a bug in the algorithm!");

	for (Location* startAndEnd : startAndEndAct->locations())	{

		mDist[threadId][0][0] = 0.0;
		mPreds[threadId][0][0] = -1;
		mLocs[threadId][0][0] = startAndEnd;

		for (uint32_t i = 0; i+1 < closedPathSize; ++i)	{

			uint32_t fromSize = 1u, toSize = 1u;
			Location **from = &startAndEnd, **to = &startAndEnd;
			double staticActDur = closedPath[i]->minAbsDuration();
			if (i > 0)	{
				fromSize = closedPath[i]->mLocations.size();
				from = closedPath[i]->mLocations.data();
			}

			if (i+2 < closedPathSize)	{
				toSize = closedPath[i+1]->mLocations.size();
				to = closedPath[i+1]->mLocations.data();
			}

			pair<Location*, Location*> edge;
			fill(mDist[threadId][i+1].begin(), mDist[threadId][i+1].begin()+toSize, F64_INF);

			for (uint32_t f = 0; f < fromSize; ++f)	{
				edge.first = from[f];
				double distance = mDist[threadId][i][f]+staticActDur;
				const auto& sit = find_if(fixed.cbegin(), fixed.cend(), [&](Location *l) { return l->parent() == edge.first->parent(); });
				if (sit != fixed.cend() && *sit != edge.first)
					distance += penaltyForViolation;

				for (uint32_t t = 0; t < toSize; ++t)	{
					edge.second = to[t];
					const auto& mit = mMapping.locationsToMovement.find(edge);
					if (mit != mMapping.locationsToMovement.cend())	{
						Movement *mv = mit->second;
						double newDist = distance+mv->minDuration();
						if (newDist < mDist[threadId][i+1][t])	{
							mDist[threadId][i+1][t] = newDist;
							mLocs[threadId][i+1][t] = to[t];
							if (writeCircuit == true)
								mPreds[threadId][i+1][t] = f;
						}
					}
				}
			}
		}

		double circuitLength = mDist[threadId][closedPathSize-1][0];
		if (circuitLength < bestDist)	{
			bestDist = circuitLength;
			if (writeCircuit == true)	{
				bestPreds = mPreds[threadId];
				bestLocs = mLocs[threadId];
			}
		}
	}

	HamiltonianCircuit<Location> shortestCircuit;
	shortestCircuit.minLength = bestDist;

	if (writeCircuit == true && bestDist < F64_INF)	{
		// The shortest circuit found...
		vector<Location*> locs;
		int64_t currentPred = 0;
		locs.reserve(closedPathSize);
		for (int64_t i = closedPathSize-1; i >= 0; --i)	{
			locs.push_back(bestLocs[i][currentPred]);
			currentPred = bestPreds[i][currentPred];
			assert((currentPred != -1 || i == 0) && "Cannot reconstruct the path! A bug in the algorithm!");
		}

		assert(locs.size() == closedPathSize && "Invalid reconstruction of the path! Please report a bug.");

		reverse(locs.begin(), locs.end());
		shortestCircuit.circuit = move(locs);
	}

	// It can return a potentially infeasible solution, i.e. a solution with the cycle time greater than the production cycle time.
	return shortestCircuit;
}

vector<vector<CircuitRecord>> ParallelHeuristicSolver::generateShortestCircuits(PrecalculatedCircuits& precalculatedCircuits) {
	const vector<Robot*>& robots = mLine.robots();
	uint32_t numberOfRobots = robots.size();
	double cycleTime = mLine.productionCycleTime();
	constexpr double tuplesPer1000s = 1000000.0;	// 1 ms per tuple processing
	const uint32_t maxAlternativesPerRobot = 4*ceil(pow(tuplesPer1000s, 1.0/numberOfRobots));
	vector<vector<CircuitRecord>> shortestCircuits(numberOfRobots);

	uint32_t id = 1u;
	#pragma omp parallel num_threads(Settings::NUMBER_OF_THREADS)
	{
		uint32_t threadId;
		#pragma omp critical
		threadId = id++;

		#pragma omp for schedule(dynamic)
		for (uint32_t r = 0; r < numberOfRobots; ++r)	{
			Graph robotGraph = constructMinimalDurationGraph(robots[r]);
			DistanceMatrix<double> initialMatrix = createInitialDistanceMatrix(robotGraph);
			DistanceMatrix<double> minDistMatrix = floyd(initialMatrix);
			vector<CircuitRecord> allCircuits = generateRandomAlternatives(threadId, robotGraph, minDistMatrix);
			// From the potentially fastest robot cycles to the slowest ones.
			sort(allCircuits.begin(), allCircuits.end());
			// Select the suitable subset of alternatives.
			if (!allCircuits.empty())	{
				shortestCircuits[r].reserve(maxAlternativesPerRobot+2);
				if (allCircuits.size() > maxAlternativesPerRobot+2)	{
					// select shortest alternatives
					double minCycleTime = allCircuits.front().hc.minLength;
					double shortestAmount = 0.2+0.6*(penaltyFunction(minCycleTime, cycleTime)/10.0);
					uint32_t numOfShortest = ceil(maxAlternativesPerRobot*shortestAmount);
					shortestCircuits[r].insert(shortestCircuits[r].end(), allCircuits.cbegin(), allCircuits.cbegin()+numOfShortest);

					// select random alternatives
					default_random_engine threadGenerator(rd());
					uint32_t numOfRandom = ceil((1.0-shortestAmount)*maxAlternativesPerRobot);
					shuffle(allCircuits.begin()+numOfShortest, allCircuits.end(), threadGenerator);
					shortestCircuits[r].insert(shortestCircuits[r].end(), allCircuits.begin()+numOfShortest, allCircuits.begin()+numOfShortest+numOfRandom);

					// keep it sorted
					sort(shortestCircuits[r].begin(), shortestCircuits[r].end());
				} else {
					// all alternatives are added
					shortestCircuits[r] = allCircuits;
				}
			} else {
				mFeasible = false;
			}

			for (CircuitRecord& cr : shortestCircuits[r])	{
				cr.hc = getShortestCircuit(threadId, cr.sc.circuit, {});
				vector<Location*> fixed;
				for (uint32_t l = 0; l+1 < cr.hc.circuit.size(); ++l)	{
					Location *loc = cr.hc.circuit[l];
					if (mMapping.sptComp.count(loc->parent()) > 0)
						fixed.push_back(loc);
				}

				cr.sc.fixedLocations = move(fixed);

				#pragma omp critical
				precalculatedCircuits.emplace(cr.sc, cr.hc);
			}
		}
	}

	if (mFeasible)	{
		mSortedSptCmp = mMapping.sptCompVec;
		sort(mSortedSptCmp.begin(), mSortedSptCmp.end(),
				[&](const SpatialCmpTuple& t1, const SpatialCmpTuple& t2)	{
					uint32_t r11 = get<0>(t1), r12 = get<1>(t1);
					uint32_t r21 = get<0>(t2), r22 = get<1>(t2);
					double ct11 = shortestCircuits[r11].front().hc.minLength, ct12 = shortestCircuits[r12].front().hc.minLength;
					double ct21 = shortestCircuits[r21].front().hc.minLength, ct22 = shortestCircuits[r22].front().hc.minLength;
					double penalty1 = penaltyFunction(ct11, cycleTime)+penaltyFunction(ct12, cycleTime);
					double penalty2 = penaltyFunction(ct21, cycleTime)+penaltyFunction(ct22, cycleTime);
					return penalty1 > penalty2;	// First the tuples with the highest penalty.
				}
		    );
	}

	return shortestCircuits;
}

double ParallelHeuristicSolver::penaltyFunction(const double& minCycle, const double& demandedCycleTime)	{
	double penalty;
	// Penalization function: f(x) = k1/(k2-x) if 0 <= x <= 1 where 'x' is the normalized cycle time
	constexpr double k1 = 10.0/99.0, k2 = 100.0/99.0; // f(0) = 0.1, f(1) = 10.0, 0.1 <= f(x) <= 10.0
	if (minCycle <= demandedCycleTime)
		penalty = k1/(k2-(minCycle/demandedCycleTime));
	else
		penalty = 30.0;	// Extra penalty for guaranteed infeasibility, indication value.

	return penalty;
}

uint32_t ParallelHeuristicSolver::selectAlternative(const vector<CircuitRecord>& alternatives) const	{

	if (alternatives.empty())
		throw InvalidArgument(caller(), "No alternatives to select!");

	default_random_engine threadGenerator(rd());
	double cycleTime = mLine.productionCycleTime();
	uint32_t selIdx = 0, numberOfAlternatives = alternatives.size();
	const HamiltonianCircuit<Location>& ha = alternatives.front().hc, & hb = alternatives.back().hc;
	const double a = ha.minLength/cycleTime, b = hb.minLength/cycleTime, minDiff = TIME_TOL/cycleTime;

	if (b-a >= minDiff)	{
		// scaled distribution function (t in <0,1>): y(t) = n*(2-k*e^{l*t})
		constexpr double l = 2.0;
		const double k = 2.0/exp(l);
		const double sa = 2*a-(k/l)*exp(l*a);
		const double sb = 2*b-(k/l)*exp(l*b);
		const double n = 1/(sb-sa);	// integrate_{a}^{b} y(t) dt = 1

		// ,,Wheel of fortune" - select the area 's', i.e. 's' = integrate_{a}^{x} y(t) dt.
		uniform_real_distribution<double> area(0.0, 1.0);
		const double s = area(threadGenerator);

		/*
		 * Solve previously mentioned 'x' by the Newton method.
		 * Equation to solve: c = 2*x-(k/l)*e^{l*x}
		 * Update formula: x_{i+1} = x_i - ((k/l)*e^{l*x}-2*x+c)/(k*e^{l*x}-2)
		 * Initial estimate: x_0 = a+s*(b-a)
		 */
		uint32_t iter = 0, maxIter = 10;
		double c = s/n+sa, x0 = a+s*(b-a), x = x0;

		do {
			double diff = k*exp(l*x)-2.0;
			if (abs(diff) > 10.0*F64_EPS)
				x = x-((k/l)*exp(l*x)-2*x+c)/diff;
			else
				break;
		} while (x >= a && x <= b && ++iter < maxIter);

		if (x < a || x > b)	{
			// Newton method diverged, select initial estimate.
			x = x0;
		}

		// With respect to 'x' value calculate ,,float index'' of the circuit to select.
		const double selIdxFloat = (x-a)/(b-a);
		// "float index" -> "integer index"
		selIdx = static_cast<uint32_t>(selIdxFloat*numberOfAlternatives);
		selIdx = (selIdx < numberOfAlternatives) ? selIdx : numberOfAlternatives-1;
	} else {
		// Too small cycle time differences or only one alternative.
		uniform_int_distribution<uint32_t> selAlt(0, numberOfAlternatives-1);
		selIdx = selAlt(threadGenerator);
	}

	return selIdx;
}

double ParallelHeuristicSolver::calculateNewCycleTime(const uint32_t& threadId, const double& currentCycleTime, vector<StaticActivity*>& alternative,
		vector<Location*>& fixed, Location *toFix, PrecalculatedCircuits& precalculatedCircuits)	{

	const uint64_t& idx = find_if(fixed.cbegin(), fixed.cend(), [&toFix](Location* l) { return toFix->parent() == l->parent(); })-fixed.cbegin();
	if (idx < fixed.size())	{
		Location *previous = fixed[idx];
		// Check whether the location to be fixed is not already fixed.
		if (previous != toFix)	{
			// Create the set of newly fixed locations.
			ShortestCircuit sc;
			fixed[idx] = toFix;
			sc.circuit = move(alternative);
			sc.fixedLocations = move(fixed);
			// Check whether the cycle time was already calculated for the given alternative and fixed locations.
			const auto& sit = precalculatedCircuits.find(sc);
			if (sit != precalculatedCircuits.cend())	{
				// Yes, it was already calculated, get it from the hash map...
				fixed = move(sc.fixedLocations); fixed[idx] = previous;
				alternative = move(sc.circuit);
				return (sit->second).minLength;
			} else {
				// Copy values back as they need to be preserved.
				fixed = sc.fixedLocations; fixed[idx] = previous;
				alternative = sc.circuit;
				// Unfortunately, it is not ,,cached''. Insert a new record.
				const auto& p = precalculatedCircuits.emplace(move(sc), getShortestCircuit(threadId, sc.circuit, sc.fixedLocations, false));
				// Return the shortest possible circuit length.
				return (*p.first).second.minLength;
			}
		} else {
			// Since the location to be fixed has already been fixed, the cycle time will not change.
			return currentCycleTime;
		}
	} else {
		throw InvalidArgument(caller(), "Invalid toFix argument, it should belong to the related robot!");
	}
}

void ParallelHeuristicSolver::addRandomTuplesToKB(const uint32_t& threadId, const uint32_t& numOfTuples,
		vector<vector<CircuitRecord>>& initialCircuits, PrecalculatedCircuits& precalculatedCircuits)	{

	high_resolution_clock::time_point start = high_resolution_clock::now();

	default_random_engine threadGenerator(rd());
	uint32_t numberOfAddedTuples = 0, iter = 0;
	double cycleTime = mLine.productionCycleTime();
	uint32_t numberOfRobots = mLine.robots().size();
	const uint32_t maxTuples = numOfTuples, maxIter = 10*maxTuples;

	while (numberOfAddedTuples < maxTuples && iter < maxIter)	{

		// For each robot just one vector item.
		vector<double> currentLengths(numberOfRobots);
		vector<CircuitRecord*> selectedCircuits(numberOfRobots);
		vector<vector<Location*>> currentFixedLocations(numberOfRobots);

		// Select robot circuits
		for (uint32_t r = 0; r < numberOfRobots; ++r)	{
			uint32_t selIdx = selectAlternative(initialCircuits[r]);
			CircuitRecord *cr = &initialCircuits[r][selIdx];
			currentLengths[r] = cr->hc.minLength;
			selectedCircuits[r] = cr;
			currentFixedLocations[r] = cr->sc.fixedLocations;
		}

		bool feasibleTuple = true;
		// Try to resolve spatial compatibility for the selected circuits.
		for (auto it = mSortedSptCmp.cbegin(); it != mSortedSptCmp.cend() && feasibleTuple; ++it)	{

			// Extract information about the spatial compatibility pair.
			uint32_t r1 = get<0>(*it), r2 = get<1>(*it);	// related robots
			const vector<pair<Location*, Location*>>& spatialCompatibility = get<4>(*it); // compatible location pairs.
			assert(!spatialCompatibility.empty() && "Invalid initialization of spatialCompatibility!");

			// Evaluate suitability of each spatial compatibility pair in terms of penalty value.
			double maxPenalty = 0.0;
			vector<double> preferences;
			vector<pair<double, double>> lengths;
			lengths.reserve(spatialCompatibility.size());
			preferences.reserve(spatialCompatibility.size());

			for (const pair<Location*, Location*>& p : spatialCompatibility)	{
				uint32_t i = 0;
				double penalty = 0.0;
				array<double, 2> newLengths;
				vector<pair<uint32_t, Location*>> vec = { {r1, p.first}, {r2, p.second}};
				for (const pair<uint32_t, Location*>& e : vec)	{
					uint32_t robotId = e.first;
					Location *toFix = e.second;
					double curLength = currentLengths[robotId];
					// It calculates the cycle time for a newly fixed location 'toFix'.
					double newMinCycleTime = calculateNewCycleTime(threadId, curLength, selectedCircuits[robotId]->sc.circuit,
							currentFixedLocations[robotId], toFix, precalculatedCircuits);
					penalty += penaltyFunction(newMinCycleTime, cycleTime);
					newLengths[i++] = newMinCycleTime;
				}

				maxPenalty = max(maxPenalty, penalty);
				preferences.push_back(penalty);		// penalty will be transformed to the preference later
				lengths.emplace_back(newLengths[0], newLengths[1]);
			}

			// Transform penalties to selection preferences.
			double sumOfPreferences = 0.0;
			vector<double>::iterator prefArray = preferences.begin();
			for (uint32_t p = 0; p < preferences.size(); ++p)	{
				prefArray[p] = maxPenalty-prefArray[p];
				sumOfPreferences += prefArray[p];
			}

			// Wheel of fortune - wheelValue decides which pair will be used to resolve compatibility.
			uniform_real_distribution<double> wheel(0.0, sumOfPreferences);
			double wheelValue = wheel(threadGenerator), accumulatedValue = 0.0;

			// Find the index to the ,,randomly" selected pair.
			uint32_t selIdx = 0;
			for (uint32_t p = 0; p < preferences.size(); ++p)	{
				double currentPref = prefArray[p];
				if (accumulatedValue <= wheelValue && wheelValue <= accumulatedValue+currentPref)	{
					selIdx = p;
					break;
				}

				accumulatedValue += currentPref;
			}

			if (maxPenalty-prefArray[selIdx] <= 20.0)	{
				// The tuple is still feasible, i.e. only feasible circuits were selected as a spatial compatibility resolution.
				// Update the minimal lengths of the circuits going through the newly fixed locations.
				currentLengths[r1] = lengths[selIdx].first;
				currentLengths[r2] = lengths[selIdx].second;
				// Update the currently fixed locations with respect to the selected pair.
				const pair<Location*, Location*>& cmpPair = spatialCompatibility[selIdx];
				Location *newlyFixed1 = cmpPair.first, *newlyFixed2 = cmpPair.second;
				vector<Location*> *fl1 = &currentFixedLocations[r1], *fl2 = &currentFixedLocations[r2];
				auto pred = [&](Location* l)	{
					return l->parent() == newlyFixed1->parent() || l->parent() == newlyFixed2->parent();
				};

				replace_if(fl1->begin(), fl1->end(), pred, newlyFixed1);
				replace_if(fl2->begin(), fl2->end(), pred, newlyFixed2);
			} else {
				// Penalty higher than 20.0 indicates that the selected pair is resulting in an infeasible solution.
				feasibleTuple = false;
			}
		}

		if (feasibleTuple == true)	{
			CircuitTuple candidate;
			for (uint32_t r = 0; r < numberOfRobots; ++r)	{
				const vector<StaticActivity*> alternative = selectedCircuits[r]->sc.circuit;
				const vector<Location*>& fixed = currentFixedLocations[r];
				auto sit = precalculatedCircuits.find(ShortestCircuit(alternative, fixed));
				if (sit != precalculatedCircuits.cend())	{
					if (sit->second.circuit.empty())
						sit->second = getShortestCircuit(threadId, alternative, fixed);

					candidate.tuple.emplace_back(sit->first, sit->second);
				} else {
					throw SolverException(caller(), "A bug in the generation of tuples, it should be stored in the map!");
				}
			}

			mKB.addTuple(move(candidate));
			++numberOfAddedTuples;
		}

		++iter;
	}

	mKB.recordAddTuplesCall(duration_cast<duration<double>>(high_resolution_clock::now()-start).count());
}

void ParallelHeuristicSolver::generatePromisingTuples(const uint32_t& threadId) {
	PrecalculatedCircuits precalculated;
	const uint32_t numOfPromisingTuples = 10u;
	uint32_t numberOfRobots = mLine.robots().size();
	// vector<set<CircuitRecord>> should be used instead...
	vector<vector<CircuitRecord>> initialCircuits(numberOfRobots);
	list<pair<Solution, CircuitTuple>> elites = mKB.eliteSolutions();
	for (const pair<Solution, CircuitTuple>& p : elites)	{
		const vector<CircuitRecord>& circuits = p.second.tuple;
		assert(circuits.size() == numberOfRobots && "Invalid tuple stored in the knowledge base!");
		for (uint32_t r = 0; r < circuits.size(); ++r)	{
			initialCircuits[r].push_back(circuits[r]);
			precalculated.emplace(circuits[r].sc, circuits[r].hc);
		}
	}

	for (uint32_t r = 0; r < numberOfRobots; ++r)
		sort(initialCircuits[r].begin(), initialCircuits[r].end());

	if (elites.size() > 1u)	{
		// Remark: Control thread id is zero.
		addRandomTuplesToKB(threadId, numOfPromisingTuples, initialCircuits, precalculated);
	}
}

double ParallelHeuristicSolver::changeRobotPaths(uint32_t threadId, const CircuitTuple& t, PartialSolution& ps, const vector<vector<Location*>>& fixed) {
	double minCycleTime = 0.0;
	const vector<Robot*>& robots = mLine.robots();
	uint32_t numberOfRobots = robots.size();
	double cycleTime = mLine.productionCycleTime();
	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		vector<Location*> candidatesToFix;
		for (Activity *a : robots[r]->activities())	{
			StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr && mMapping.sptComp.count(sa) == 0)	{
				for (Location* l : sa->locations())
					candidatesToFix.push_back(l);
			}
		}

		shuffle(candidatesToFix.begin(), candidatesToFix.end(), default_random_engine(rd()));

		assert(r < fixed.size() && "Invalid dimensions of the input argument!");

		uint32_t idx = 0;
		bool pathChanged = false;
		vector<Location*> toFix = fixed[r], path = ps.locs[r];
		const vector<StaticActivity*>& alternative = t.tuple[r].sc.circuit;
		while (!pathChanged && idx < candidatesToFix.size())	{
			toFix.push_back(candidatesToFix[idx]);
			HamiltonianCircuit<Location> hc = getShortestCircuit(threadId, alternative, toFix, true);
			if (hc.minLength <= cycleTime)	{
				path = hc.circuit;
				minCycleTime = max(minCycleTime, hc.minLength);
				pathChanged = true;
			}
			toFix.pop_back();
			++idx;
		}

		ps.locs[r] = path;
	}

	mKB.recordChangePathCall();

	return minCycleTime;
}

void ParallelHeuristicSolver::printProgressInfo(const double& currentRuntime)	{
	const auto& tupleGen = mKB.infoTuplesGeneration(), &itersPerTuple = mKB.infoItersPerTuple();
	const auto& LPInfo = mKB.infoLP(), &infoLocHeur = mKB.infoLocHeur(), &infoPwrmHeur = mKB.infoPwrmHeur();
	clog<<string(35, '=')<<" STAT INFO "<<string(35, '=')<<endl;
	clog<<"current runtime: "<<currentRuntime<<endl;
	clog<<string(81, '-')<<endl;
	clog<<"generated tuples: "<<tupleGen.first<<" ("<<tupleGen.second*1000.0<<" ms per tuple)"<<endl;
	clog<<"percentage of processed: "<<mKB.percentageOfProcessed()<<" %"<<endl;
	clog<<"average number of iters per tuple: "<<itersPerTuple.second<<endl;
	clog<<string(81, '-')<<endl;
	clog<<"Linear Programming solver: "<<LPInfo.first<<" calls ("<<LPInfo.second*1000.0<<" ms per call)"<<endl;
	clog<<"Infeasibility rate: "<<mKB.infeasibilityRate()*100.0<<" %"<<endl;
	clog<<"Average number of LP calls for collisions resolution: "<<mKB.averageNumberOfLPCallsForLPFix()<<endl;
	clog<<"Average deterioration in quality after collisions resolution: "<<mKB.averageLPFixDeterioration()*100.0<<" %"<<endl;
	clog<<"Number of feasibility breaks: "<<mKB.numberOfLPBreaks()<<endl;
	clog<<string(81, '-')<<endl;
	clog<<"Change location heuristic: "<<infoLocHeur.first<<" calls ("<<infoLocHeur.second*1000.0<<" ms per call)"<<endl;
	clog<<"Average error of energy estimation: "<<mKB.averageErrOfLocHeur()*100.0<<" %"<<endl;
	clog<<"Number of feasibility breaks: "<<mKB.numberOfLocHeurBreaks()<<endl;
	clog<<string(81, '-')<<endl;
	clog<<"Change power mode heuristic: "<<infoPwrmHeur.first<<" calls ("<<infoPwrmHeur.second*1000.0<<" ms per call)"<<endl;
	clog<<"Average error of energy estimation: "<<mKB.averageErrOfPwrmHeur()*100.0<<" %"<<endl;
	clog<<"Number of feasibility breaks: "<<mKB.numberOfPwrmHeurBreaks()<<endl;
	clog<<string(81, '-')<<endl;
	clog<<"Change path algorithm: "<<mKB.pathChangeCalls()<<" calls"<<endl;
	clog<<"Number of feasibility breaks: "<<mKB.numberOfChangePathBreaks()<<endl;
	clog<<string(34, '=')<<" END OF INFO "<<string(34, '=')<<endl;
}

void ParallelHeuristicSolver::writePerformanceRecordToLogFile(const double& initializationTime, const double& finalRuntime)	{
	if (!Settings::RESULTS_DIRECTORY.empty())	{
		string pathToPerformaceLog = Settings::RESULTS_DIRECTORY+"performance_log.csv";
		bool exists = fileExists(pathToPerformaceLog);
		ofstream perfLog(pathToPerformaceLog.c_str(), ofstream::app);
		if (perfLog.good())	{
			if (!exists)	{
				perfLog<<"initialization time [s], generated tuples, time per tuple [ms], processed tuples [%], average number of iters per tuple, ";
				perfLog<<"number of calls of Linear Programming, time per call [ms], infeasibility rate [%], LP calls to collisions resolution, ";
				perfLog<<"average deterioration after collisions resolution [%], feasibility breaks, number of calls of change location heuristic, ";
				perfLog<<"time per call [ms], estimation error [%], feasibility breaks, number of calls of power mode heuristic, time per call [ms], ";
				perfLog<<"estimation error [%], feasibility breaks, number of calls of change path, feasibility breaks, runtime [s]"<<endl;
			}

			const auto& tupleGen = mKB.infoTuplesGeneration(), &itersPerTuple = mKB.infoItersPerTuple();
			const auto& LPInfo = mKB.infoLP(), &infoLocHeur = mKB.infoLocHeur(), &infoPwrmHeur = mKB.infoPwrmHeur();
			perfLog<<initializationTime<<", "<<tupleGen.first<<", "<<1000.0*tupleGen.second<<", "<<mKB.percentageOfProcessed()<<", "<<itersPerTuple.second<<", ";
			perfLog<<LPInfo.first<<", "<<1000.0*LPInfo.second<<", "<<100.0*mKB.infeasibilityRate()<<", "<<mKB.averageNumberOfLPCallsForLPFix()<<", ";
			perfLog<<100.0*mKB.averageLPFixDeterioration()<<", "<<mKB.numberOfLPBreaks()<<", "<<infoLocHeur.first<<", "<<1000.0*infoLocHeur.second<<", ";
			perfLog<<100.0*mKB.averageErrOfLocHeur()<<", "<<mKB.numberOfLocHeurBreaks()<<", "<<infoPwrmHeur.first<<", "<<1000.0*infoPwrmHeur.second<<", ";
			perfLog<<100.0*mKB.averageErrOfPwrmHeur()<<", "<<mKB.numberOfPwrmHeurBreaks()<<", "<<mKB.pathChangeCalls()<<", ";
			perfLog<<mKB.numberOfChangePathBreaks()<<", "<<finalRuntime<<endl;
			perfLog.close();
		} else {
			cerr<<"Warning: Cannot open '"<<pathToPerformaceLog<<"' file for writing!"<<endl;
		}
	}
}


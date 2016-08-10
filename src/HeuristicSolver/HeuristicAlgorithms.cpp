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

#include <chrono>
#include "Shared/Algorithms.h"
#include "HeuristicSolver/HeuristicAlgorithms.h"
#ifndef NDEBUG
#include "Solution/SolutionChecker.h"
#endif

using namespace std;
using namespace std::chrono;

static random_device rd;

/*!
 * \brief An unimodal function for two movements and their fixed total time.
 * \details An unimodal convex function, represented by this class, can be used in the Golden Section search
 * algorithm to find optimal timing of two movements where their total time is fixed.
 * \see goldenSearch, HeuristicAlgorithms::heuristicLocationChanges
 */
class UnimodalFunctionHeuristic {
	public:
		/*!
		 * \param mv1, mv2 Movements which optimal duration is to be determined.
		 * \param duration The total time, considered to be fixed, needed for both the movements.
		 * \brief Constructor initializes the member variables.
		 */
		UnimodalFunctionHeuristic(Movement *mv1, Movement *mv2, double duration) : mMv1(mv1), mMv2(mv2), mTotalDuration(duration) {
			if (mv1 == nullptr || mv2 == nullptr || duration < 0.0)
				throw InvalidArgument(caller(), "Attempt to pass the malicious constructor parameters!");
			else if (mv1->minDuration()+mv2->minDuration() >= duration+2*TIME_ERR || mv1->maxDuration()+mv2->maxDuration()+2*TIME_ERR <= duration)
				throw InvalidArgument(caller(), "Invalid constructor parameters, the solution is infeasible!");

			mFromX = max(duration-mv2->maxDuration(), mv1->minDuration());
			mToX = min(duration-mv2->minDuration(), mv1->maxDuration());
			mDirection = mToX-mFromX;
			assert(mDirection > -TIME_ERR && "A bug in the UnimodalFunctionHeuristic class, it should be positive!");
		}

		/*!
		 * As there is no use to solve the timing of movements with too high precision,
		 * \f$10^{-2}\,s\f$ is enough in the most cases, this method returns a relative tolerance
		 * ensuring that Golden Section search is stopped if just enough precise solution is found.
		 */
		double tolerance() const {
			if (mDirection >= TIME_TOL)
				return TIME_TOL/mDirection;
			else
				return 1.0;	// solution is fixed, nothing to be optimized
		}

		/*!
		 * \param alpha A step, i.e. a value from 0 to 1, used for obtaining the timing of the movements.
		 * \brief Calculates durations of the movements based on the relative step.
		 */
		pair<double, double> durations(const double& alpha) const {
			if (alpha < 0.0 || alpha > 1.0)
				throw InvalidArgument(caller(), "Invalid function parameter, it must be in a range from 0.0 to 1.0!");

			double d1 = mFromX+alpha*mDirection, d2 = mTotalDuration-d1;
			assert(abs(d1+d2-mTotalDuration) < 5*TIME_ERR && "Invalid golden search of the method, solution infeasible!");
			assert(mMv1->minDuration() < d1+5*TIME_ERR && d1 < mMv1->maxDuration()+5*TIME_ERR && "Duration of movement 1 is out of bound!");
			assert(mMv2->minDuration() < d2+5*TIME_ERR && d2 < mMv2->maxDuration()+5*TIME_ERR && "Duration of movement 2 is out of bound!");

			return { d1, d2 };
		}

		/*!
		 * \param alpha %Variable value corresponding to a relative step.
		 * \brief Returns a function value, i.e. total energy consumption of both movements.
		 */
		double functionValue(const double& alpha) const {
			const auto& durPair = durations(alpha);
			return mMv1->energyConsumption(durPair.first)+mMv2->energyConsumption(durPair.second);
		}

	private:
		//! A maximal possible difference in duration for both the movements.
		double mDirection;
		//! The minimal duration of the first movement.
		double mFromX;
		//! The maximal duration of the first movement.
		double mToX;
		//! The first movement to be considered.
		Movement *mMv1;
		//! The second movement to be considered.
		Movement *mMv2;
		//! The time needed for both the movements is equal to this fixed value.
		double mTotalDuration;
};

vector<vector<Location*>> HeuristicAlgorithms::initializePartialSolution(PartialSolution& ps) const	{
	const vector<Robot*>& robots = mLine.robots();
	uint32_t numberOfRobots = robots.size();
	if (ps.locs.size() != numberOfRobots)
		throw InvalidArgument(caller(), "Invalid initial partial solution!");

	ps.mvs.clear();
	bool writePwrms = ps.pwrms.empty();
	vector<vector<Location*>> fixed(numberOfRobots);
	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		vector<Movement*> mvs;
		const vector<Location*>& locs = ps.locs[r];
		assert(locs.size() > 1 && "Circuits in the partial solution should be filled in!");
		for (uint32_t l = 0; l+1 < locs.size(); ++l)	{
			Location *from = locs[l], *to = locs[l+1];
			Movement *mv = getValue(mMapping.locationsToMovement, {from, to}, caller());
			assert(mv != nullptr && "Unexpected nullptr! Should be checked by setValue previously!");
			mvs.push_back(mv);
			// Check whether the location must be fixed...
			if (mMapping.sptComp.count(from->parent()) > 0)
				fixed[r].push_back(from);
		}

		ps.mvs.push_back(move(mvs));
		if (writePwrms == true)
			ps.pwrms.emplace_back(locs.size(), robots[r]->fastestPowerSavingMode());
	}

	return fixed;
}

OptimalTiming HeuristicAlgorithms::solvePartialProblem(const PartialSolution& ps, const CircuitTuple& t, Algo algo) {
	Solution s;
	OptimalTiming ot;
	bool collisionsFeasible = false;
	high_resolution_clock::time_point startInit = high_resolution_clock::now();
	RoboticLineSolverLP solver(mLine, ps, mMapping);
	double extraTime = duration_cast<duration<double>>(high_resolution_clock::now()-startInit).count();
	double minCriterion = F64_MAX, maxCriterion = F64_MIN;

	mKB.recordPartialProblemSolveCall();

	while (!collisionsFeasible)	{
		high_resolution_clock::time_point startLP = high_resolution_clock::now();
		s = solver.solve();
		mKB.recordLPCall(duration_cast<duration<double>>(high_resolution_clock::now()-startLP).count()+extraTime);
		if (s.status == OPTIMAL)	{
			ot = convert(ps, s);
			minCriterion = min(minCriterion, ot.totalEnergy);
			maxCriterion = max(maxCriterion, ot.totalEnergy);
			const CollisionResolution& r = resolveTheWorstCollision(ot);
			if (r.a1 != nullptr && r.a2 != nullptr)
				solver.addCollisionResolution(r.a1, r.a2, r.multiplier);
			else
				collisionsFeasible = true;

			extraTime = 0.0;
		} else {
			switch (algo) {
				case LP_INITIAL_PROBLEM:
					mKB.reportInfDueToLP();
					break;
				case POWER_MODE_HEURISTIC:
					mKB.reportInfDueToPwrmHeur();
					break;
				case LOCATION_CHANGE_HEURISTIC:
					mKB.reportInfDueToLocHeur();
					break;
				case PATH_CHANGE:
					mKB.reportInfDueToPathChange();
					break;
			}

			mKB.recordInfeasibleLP();

			throw NoFeasibleSolutionExists(caller(), "Cannot solve the subproblem to optimality!");
		}
	}

	#ifndef NDEBUG
	SolutionChecker checker(mLine, mMapping, s);
	if (!checker.checkAll())
		throw SolverException(caller(), "A bug in solving the partial LP problem, the solution is infeasible!");
	#endif

	mKB.candidate(s, t);
	if (minCriterion > 0.0)
		mKB.recordLPFixDeterioration(maxCriterion/minCriterion-1.0);

	return ot;
}

OptimalTiming HeuristicAlgorithms::convert(const PartialSolution& ps, const Solution& s) const {
	OptimalTiming timing;
	timing.totalEnergy = s.totalEnergy;
	vector<double> startTimes, durations;
	uint32_t numberOfRobots = mLine.robots().size();
	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		const vector<Location*>& locs = ps.locs[r];
		for (Location *loc : locs)	{
			assert(loc != nullptr && loc->parent() != nullptr && "Unexpected null pointer!");

			pair<double, double> val;
			const auto& sit = s.startTimeAndDuration.find(loc->parent()->aid());
			if (sit != s.startTimeAndDuration.cend())
				val = sit->second;
			else
				throw SolverException(caller(), "Given the incomplete solution!");

			startTimes.push_back(val.first);
			durations.push_back(val.second);
			timing.actToStartAndDur[loc->parent()] = val;
			timing.modeToStartAndDur[loc] = val;
		}

		timing.startLocs.push_back(move(startTimes));
		timing.durLocs.push_back(move(durations));
		startTimes.clear(); durations.clear();

		const vector<Movement*>& mvs = ps.mvs[r];
		for (Movement *mv : mvs)	{
			assert(mv != nullptr && mv->parent() != nullptr && "Unexpected null pointer!");

			pair<double, double> val;
			const auto& sit = s.startTimeAndDuration.find(mv->parent()->aid());
			if (sit != s.startTimeAndDuration.cend())
				val = sit->second;
			else
				throw SolverException(caller(), "Given the incomplete solution!");

			startTimes.push_back(val.first);
			durations.push_back(val.second);
			timing.actToStartAndDur[mv->parent()] = val;
			timing.modeToStartAndDur[mv] = val;
		}

		timing.startMvs.push_back(move(startTimes));
		timing.durMvs.push_back(move(durations));
		startTimes.clear(); durations.clear();
	}

	return timing;
}

CollisionResolution HeuristicAlgorithms::resolveCollision(ActivityMode* m1, double s1, double d1, ActivityMode* m2, double s2, double d2) const	{
	CollisionResolution cl;
	double cycleTime = mLine.productionCycleTime();
	double s1m = fmod(s1, cycleTime), s2m = fmod(s2, cycleTime);
	if (s1m > s2m)	{
		swap(m1, m2); swap(s1, s2); swap(d1, d2); swap(s1m, s2m);
	}

	double intersection1 = min(max(s1m+d1-s2m, 0.0), d2);
	double intersection2 = min(max(s2m+d2-(s1m+cycleTime), 0.0), d1);
	cl.intersection = intersection1 + intersection2;

	// Remark: Since d1 + d2 <= CT, not both intersections are possible at once.
	if (cl.intersection > TIME_TOL || !(s2m+d2-s1m <= cycleTime+TIME_TOL && s1m+d1 <= s2m+TIME_TOL))	{
		if (intersection2 > TIME_TOL)
			s1m += cycleTime;

		if (s1m+0.5*d1 <= s2m+0.5*d2)	{
			cl.a1 = m1->baseParent();
			cl.a2 = m2->baseParent();
			cl.multiplier = (int32_t) round(((s2-s2m)/cycleTime)-((s1-s1m)/cycleTime));
		} else	{
			cl.a1 = m2->baseParent();
			cl.a2 = m1->baseParent();
			cl.multiplier = (int32_t) round(((s1-s1m)/cycleTime)-((s2-s2m)/cycleTime));
		}
	}

	return cl;
}

CollisionResolution HeuristicAlgorithms::resolveTheWorstCollision(const OptimalTiming& ot) const	{
	CollisionResolution ret;
	for (const auto& mit1 : ot.modeToStartAndDur)   {
		ActivityMode *m1 = mit1.first;
		double s1 = mit1.second.first, d1 = mit1.second.second;
		const auto& mit2 = mMapping.collisionSearch.find(m1);
		if (mit2 != mMapping.collisionSearch.cend())    {
			for (ActivityMode *m2 : mit2->second)   {
				const auto& sit = ot.modeToStartAndDur.find(m2);
				if (sit != ot.modeToStartAndDur.cend()) {
					double s2 = sit->second.first, d2 = sit->second.second;
					// It is necessary to check the possible collision.
					CollisionResolution cl = resolveCollision(m1, s1, d1, m2, s2, d2);
					if (ret.intersection < cl.intersection)
						ret = cl;
				}
			}
		}
	}

	return ret;
}

double HeuristicAlgorithms::heuristicLocationChanges(OptimalTiming& ot, PartialSolution& ps, const vector<vector<Location*>>& fixed, bool& solutionChanged) const	{

	high_resolution_clock::time_point startLocHeur = high_resolution_clock::now();

	solutionChanged = false;
	double energyImprovement = 0.0;
	default_random_engine threadGenerator(rd());
	double cycleTime = mLine.productionCycleTime();
	uint32_t numberOfRobots = mLine.robots().size();
	double averageInputPower = ot.totalEnergy/(numberOfRobots*cycleTime);
	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		uint32_t numberOfLocations = ps.locs[r].size();
		if (numberOfLocations <= 2)	// the first location is duplicated at the end
			continue;

		uniform_int_distribution<uint32_t> randomOffset(0u, 1u);
		for (uint32_t i = randomOffset(threadGenerator); i+1u < numberOfLocations; i += 2u)	{
			Location *bestLoc = ps.locs[r][i], *selLoc = bestLoc;
			assert(bestLoc != nullptr && bestLoc->parent() != nullptr && "Invalid initialization of the robotic cell!");
			if (count(fixed[r].cbegin(), fixed[r].cend(), bestLoc) == 0)	{
				// It is possible to change location without spatial compatibility breaking.
				int64_t prevIdxMv = (i > 0 ? i-1 : ps.mvs[r].size()-1);
				int64_t prevIdxLoc = (i > 0 ? i-1 : numberOfLocations-2);
				assert(prevIdxMv >= 0 && prevIdxLoc >= 0 && "Invalid index calculation, were the vectors initialized?!");
				Movement *bestEntering = ps.mvs[r][prevIdxMv], *bestLeaving = ps.mvs[r][i];
				RobotPowerMode *selectedPowerSavingMode = ps.pwrms[r][i];
				Location *prevLoc = ps.locs[r][prevIdxLoc], *nextLoc = ps.locs[r][i+1];
				// Get the current timing...
				double bestStart1 = ot.startMvs[r][prevIdxMv], bestDur1 = ot.durMvs[r][prevIdxMv];	// entering
				double bestStart2 = ot.startLocs[r][i], bestDur2 = ot.durLocs[r][i];	// inner location
				double bestStart3 = ot.startMvs[r][i], bestDur3 = ot.durMvs[r][i];	// leaving
				double stationaryDuration = bestDur2, dynamicDuration = bestDur1+bestDur3;
				// Remove a part from a robot gantt to update it later.
				ot.modeToStartAndDur.erase(bestEntering); ot.actToStartAndDur.erase(bestEntering->parent());
				ot.modeToStartAndDur.erase(bestLoc); ot.actToStartAndDur.erase(bestLoc->parent());
				ot.modeToStartAndDur.erase(bestLeaving); ot.actToStartAndDur.erase(bestLeaving->parent());
				// Calculate the current energy consumption.
				double initialEnergyConsumption = bestEntering->energyConsumption(bestDur1), bestEnergyConsumption = F64_INF;
				initialEnergyConsumption += bestLoc->energyConsumption(stationaryDuration, selectedPowerSavingMode);
				initialEnergyConsumption += bestLeaving->energyConsumption(bestDur3);
				// Try to select another location if it is possible and more energy efficient.
				for (Location *loc : selLoc->parent()->locations())	{
					const auto& sit1 = mMapping.locationsToMovement.find({prevLoc, loc});
					const auto& sit2 = mMapping.locationsToMovement.find({loc, nextLoc});
					if (sit1 != mMapping.locationsToMovement.cend() && sit2 != mMapping.locationsToMovement.cend())	{
						Movement *entering = sit1->second, *leaving = sit2->second;
						bool callGoldenSearch = entering->minDuration()+leaving->minDuration() < dynamicDuration+TIME_ERR;
						callGoldenSearch = callGoldenSearch && entering->maxDuration()+leaving->maxDuration()+TIME_ERR > dynamicDuration;
						if (callGoldenSearch == true)	{
							// Calculate total energy required for the pair of movements and location.
							UnimodalFunctionHeuristic unifFce(entering, leaving, dynamicDuration);
							const pair<double, double>& res = goldenSearch(unifFce);
							double totalEnergy = res.second;
							totalEnergy += loc->energyConsumption(stationaryDuration, selectedPowerSavingMode);
							// Get new timing for 'mv1 -> loc2 -> mv3'.
							const pair<double, double>& mvsDurs = unifFce.durations(res.first);
							double s1 = ot.startMvs[r][prevIdxMv], d1 = mvsDurs.first;
							double s2 = s1+d1, d2 = stationaryDuration;
							double s3 = s2+d2, d3 = mvsDurs.second;
							if (loc->parent()->lastInCycle())
								s3 -= cycleTime;
							// Estimate time violation after switching the location and penalize it in terms of energy.
							vector<ActivityModeInfo> partOfGantt = { { entering, s1, d1 }, { loc, s2, d2 }, { leaving, s3, d3 } };
							double timePenalty = calculateBreakagePenalty(partOfGantt, ot);
							totalEnergy += mPenaltyMultiplier*averageInputPower*timePenalty;
							// Decide whether it is an improvement or not...
							if (totalEnergy < bestEnergyConsumption)	{
								bestEnergyConsumption = totalEnergy;
								bestEntering = entering; bestLoc = loc; bestLeaving = leaving;
								bestStart1 = s1; bestDur1 = d1; bestStart2 = s2; bestDur2 = d2; bestStart3 = s3; bestDur3 = d3;
							}
						}
					}
				}

				if (bestEnergyConsumption < F64_INF)	{
					// Add an energy improvement after switching a location.
					energyImprovement += initialEnergyConsumption-bestEnergyConsumption;
					// Update the part of the Gantt according the the best selected location.
					ps.mvs[r][prevIdxMv] = bestEntering;
					ot.durMvs[r][prevIdxMv] = bestDur1;
					setValue(ot.modeToStartAndDur, bestEntering, {bestStart1, bestDur1}, caller());
					setValue(ot.actToStartAndDur, bestEntering->parent(), {bestStart1, bestDur1}, caller());
					ps.locs[r][i] = bestLoc;
					if (i == 0)
						ps.locs[r].back() = ps.locs[r].front();
					ot.startLocs[r][i] = bestStart2;
					setValue(ot.modeToStartAndDur, bestLoc, {bestStart2, bestDur2}, caller());
					setValue(ot.actToStartAndDur, bestLoc->parent(), {bestStart2, bestDur2}, caller());
					ps.mvs[r][i] = bestLeaving;
					ot.startMvs[r][i] = bestStart3;
					ot.durMvs[r][i] = bestDur3;
					setValue(ot.modeToStartAndDur, bestLeaving, {bestStart3, bestDur3}, caller());
					setValue(ot.actToStartAndDur, bestLeaving->parent(), {bestStart3, bestDur3}, caller());
					if (bestLoc != selLoc)
						solutionChanged = true;
				} else {
					throw SolverException(caller(), "Cannot select a suitable location, either the heuristic bug or invalid partial solution!");
				}
			}
		}
	}

	mKB.recordLocHeurCall(duration_cast<duration<double>>(high_resolution_clock::now()-startLocHeur).count());

	return energyImprovement;
}

double HeuristicAlgorithms::heuristicPowerModeSelection(const OptimalTiming& ot, PartialSolution& ps, TabuList& tabuList, bool& solutionChanged) const	{

	high_resolution_clock::time_point startPwrmHeur = high_resolution_clock::now();

	solutionChanged = false;
	const vector<Robot*>& robots = mLine.robots();
	double cycleTime = mLine.productionCycleTime();
	uint32_t numberOfRobots = robots.size();
	vector<uint32_t> numberOfLocations(numberOfRobots);
	vector<vector<ActivityModeInfo>> info(numberOfRobots);
	vector<double> consumedEnergyPerRobot(numberOfRobots, 0.0);
	double averageInputPower = ot.totalEnergy/(numberOfRobots*cycleTime);

	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		numberOfLocations[r] = ps.locs[r].size();
		for (uint32_t l = 0; l+1 < numberOfLocations[r]; ++l)	{
			Location *loc = ps.locs[r][l];
			RobotPowerMode *pwrm = ps.pwrms[r][l];
			double startLoc = ot.startLocs[r][l], durLoc = ot.durLocs[r][l], energyLoc = loc->energyConsumption(durLoc, pwrm);
			info[r].push_back({loc, pwrm, energyLoc, startLoc, durLoc,
					max(pwrm->minimalDelay(), loc->parent()->minAbsDuration()), loc->parent()->maxAbsDuration()});
			consumedEnergyPerRobot[r] += energyLoc;
		}

		for (uint32_t l = 0; l+1 < numberOfLocations[r]; ++l)	{
			Movement *mv = ps.mvs[r][l];
			double startMv = ot.startMvs[r][l], durMv = ot.durMvs[r][l], energyMv = mv->energyConsumption(durMv);
			info[r].push_back({mv, nullptr, energyMv, startMv, durMv, mv->minDuration(), mv->maxDuration()});
			consumedEnergyPerRobot[r] += energyMv;
		}
	}

	double originalEnergyConsumption = accumulate(consumedEnergyPerRobot.cbegin(), consumedEnergyPerRobot.cend(), 0.0);

	vector<ModeSwitchInfo> candidates;
	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		for (uint32_t l = 0; l+1 < numberOfLocations[r]; ++l)	{
			Location *selLoc = ps.locs[r][l];
			RobotPowerMode *selMode = ps.pwrms[r][l];
			StaticActivity *selAct = selLoc->parent();
			for (RobotPowerMode *pwrm : robots[r]->powerModes())	{
				if (pwrm != selMode && pwrm->minimalDelay() <= selAct->maxAbsDuration())	{
					vector<ActivityModeInfo> robotTiming = info[r];
					double minReqDur = max(selAct->minAbsDuration(), pwrm->minimalDelay());

					robotTiming[l].pwrm = pwrm;
					robotTiming[l].energy = selLoc->energyConsumption(minReqDur, pwrm);
					robotTiming[l].minDuration = robotTiming[l].curDuration = minReqDur;
					robotTiming[l].maxDuration = selAct->maxAbsDuration();

					double energyScaledRobot = scaleGanttToCycleTime(robotTiming);
					if (energyScaledRobot < F64_INF)	{
						double timePenalty = calculateBreakagePenalty(robotTiming, ot);
						double estimatedEnergy = originalEnergyConsumption-consumedEnergyPerRobot[r];
						estimatedEnergy += energyScaledRobot+mPenaltyMultiplier*averageInputPower*timePenalty;
						candidates.emplace_back(r, l, selLoc, selMode, pwrm, estimatedEnergy);
					}
				}
			}
		}
	}

	double energyImprovement = 0.0;
	if (!candidates.empty())	{
		ModeSwitchInfo selected;
		// Sort candidates according their energy estimation.
		sort(candidates.begin(), candidates.end());
		// Select the best candidate not in tabu list.
		const auto& sit = find_if(candidates.cbegin(), candidates.cend(), [&](const ModeSwitchInfo& i) { return !tabuList.isTabu(i.loc, i.from, i.to); });
		if (sit != candidates.cend())	{
			selected = *sit;
			// The selected mode switch is being added to the tabu list.
			tabuList.addTabu(selected.loc, selected.from, selected.to);
		} else {
			// All candidates are tabu, random selection and tabu list diversification.
			default_random_engine threadGenerator(rd());
			uniform_int_distribution<uint32_t> randSel(0, candidates.size()-1);
			uint32_t selIdx = randSel(threadGenerator);
			selected = candidates[selIdx];
			tabuList.diversify();
		}

		ps.pwrms[selected.robotIdx][selected.locationIdx] = selected.to;
		if (selected.locationIdx == 0)
			ps.pwrms[selected.robotIdx].back() = ps.pwrms[selected.robotIdx].front();

		energyImprovement = originalEnergyConsumption-selected.estimatedEnergy;
		solutionChanged = true;
	}

	mKB.recordPwrmHeurCall(duration_cast<duration<double>>(high_resolution_clock::now()-startPwrmHeur).count());

	return energyImprovement;
}

double HeuristicAlgorithms::scaleGanttToCycleTime(vector<ActivityModeInfo>& robotGantt) const	{
	double sumOfDur = 0.0;
	double robotEnergy = 0.0;
	double cycleTime = mLine.productionCycleTime();
	for (const ActivityModeInfo& ami : robotGantt)	{
		sumOfDur += ami.curDuration;
		robotEnergy += ami.energy;
	}

	double durDiff = sumOfDur-cycleTime;
	vector<bool> changeAllowed(robotGantt.size(), false);
	bool shorten = durDiff > 0.0, prolong = durDiff < 0.0;

	while (abs(durDiff) > TIME_TOL)	{
		double maxFeasChange = F64_INF;
		uint32_t numberOfModifiable = 0u;
		for (uint32_t i = 0; i < robotGantt.size(); ++i)	{
			if (shorten && robotGantt[i].curDuration > robotGantt[i].minDuration)	{
				maxFeasChange = min(maxFeasChange, robotGantt[i].curDuration-robotGantt[i].minDuration);
				changeAllowed[i] = true;
				++numberOfModifiable;
			} else if (prolong && robotGantt[i].curDuration < robotGantt[i].maxDuration)	{
				maxFeasChange = min(maxFeasChange, robotGantt[i].maxDuration-robotGantt[i].curDuration);
				changeAllowed[i] = true;
				++numberOfModifiable;
			} else {
				changeAllowed[i] = false;
			}
		}

		if (numberOfModifiable > 0 && maxFeasChange < F64_INF)	{
			maxFeasChange = min(abs(durDiff)/numberOfModifiable, maxFeasChange);
			if (shorten)
				maxFeasChange = -maxFeasChange;

			for (uint32_t i = 0; i < robotGantt.size(); ++i)	{
				if (changeAllowed[i])	{
					double energyUpdated = F64_INF;
					ActivityMode* mode = robotGantt[i].mode;
					double durationUpdated = robotGantt[i].curDuration + maxFeasChange;
					assert(robotGantt[i].minDuration <= durationUpdated+TIME_ERR
							&& durationUpdated <= robotGantt[i].maxDuration+TIME_ERR && "Invalid scaling of the robot graph!");

					Location *loc = dynamic_cast<Location*>(mode);
					if (loc != nullptr)
						energyUpdated = loc->energyConsumption(durationUpdated, robotGantt[i].pwrm);

					Movement *mv = dynamic_cast<Movement*>(mode);
					if (mv != nullptr)
						energyUpdated = mv->energyConsumption(durationUpdated);

					robotEnergy += energyUpdated-robotGantt[i].energy;

					robotGantt[i].energy = energyUpdated;
					robotGantt[i].curDuration = durationUpdated;
				}
			}
		} else {
			#ifndef NDEBUG
			double minCycleTime = 0.0;
			for (const ActivityModeInfo& ami : robotGantt)
				minCycleTime += ami.minDuration;

			if (minCycleTime+TIME_TOL < cycleTime)
				throw SolverException(caller(), "A bug in the method, the gantt should be scalable!");
			#endif

			// Cannot be scaled -> it will lead to infeasible solution.
			return F64_INF;
		}

		durDiff += numberOfModifiable*maxFeasChange;
	}

	// Update start times.
	sort(robotGantt.begin(), robotGantt.end());
	for (uint32_t i = 0; i+1 < robotGantt.size(); ++i)
		robotGantt[i+1].curStartTime = robotGantt[i].curStartTime+robotGantt[i].curDuration;

	#ifndef NDEBUG
	double checkCycleTime = 0.0;
	for (const ActivityModeInfo& ami : robotGantt)	{
		checkCycleTime += ami.curDuration;
		if (ami.minDuration > ami.curDuration+TIME_TOL || ami.maxDuration+TIME_TOL < ami.curDuration)
			throw SolverException(caller(), "Invalid update step of the algorithm.");
	}

	if (abs(checkCycleTime-cycleTime) > TIME_TOL)
		throw SolverException(caller(), "The robot gantt was not scaled to the desired cycle time, a bug in the algorithm!");
	#endif

	return robotEnergy;
}

double HeuristicAlgorithms::calculateBreakagePenalty(const std::vector<ActivityModeInfo>& robotGantt, const OptimalTiming& ot) const	{

	double timePenalty = 0.0;
	double cycleTime = mLine.productionCycleTime();

	for (uint32_t i = 0; i < robotGantt.size(); ++i)	{
		ActivityMode *m1 = robotGantt[i].mode;
		double s1 = robotGantt[i].curStartTime, d1 = robotGantt[i].curDuration;

		// check collisions
		const auto& sit = mMapping.collisionSearch.find(m1);
		if (sit != mMapping.collisionSearch.cend())	{
			for (ActivityMode *m2 : sit->second)	{
				const auto& mit = ot.modeToStartAndDur.find(m2);
				if (mit != ot.modeToStartAndDur.cend())	{
					double s2 = mit->second.first, d2 = mit->second.second;
					CollisionResolution cl = resolveCollision(m1, s1, d1, m2, s2, d2);
					timePenalty += cl.intersection;
				}
			}
		}

		// check time lags
		uint32_t idx = 0;
		for (const map<Activity*, vector<TimeLag>>* mapPtr : { &mMapping.timeLagsFromAct, &mMapping.timeLagsToAct })	{
			const auto& sit1 = mapPtr->find(m1->baseParent());
			if (sit1 != mapPtr->cend())	{
				for (const TimeLag& tl : sit1->second)	{
					Activity *relAct = (idx == 0 ? tl.to() : tl.from());
					const auto& mit = ot.actToStartAndDur.find(relAct);
					if (mit != ot.actToStartAndDur.cend())	{
						double s2 = mit->second.first;
						if (idx == 0)
							timePenalty += max(s1-s2+tl.length()-tl.height()*cycleTime, 0.0);
						else
							timePenalty += max(s2-s1+tl.length()-tl.height()*cycleTime, 0.0);
					}
				}
			}

			++idx;
		}
	}

	assert(timePenalty >= 0.0 && "Time penalty must be positive, a bug in the algorithm!");

	return timePenalty;
}


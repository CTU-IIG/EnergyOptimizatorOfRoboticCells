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
#include <cmath>
#include <chrono>
#include <map>
#include <string>
#include "Settings.h"
#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "ILPModel/SolverInterface.h"
#include "ILPSolver/RoboticLineSolverLP.h"

using namespace std;
using namespace std::chrono;

RoboticLineSolverLP::RoboticLineSolverLP(Robot* r, const PartialSolution& ps, const PrecalculatedMapping& m) : mMapper(r), mGenerator(r, &m), mPartialSolution(ps) {
	#ifndef NDEBUG
	checkArguments(m);
	#endif
	addVariablesForSelectedDynamicActivities();
	construct(m, false);
}

RoboticLineSolverLP::RoboticLineSolverLP(const RoboticLine& l, const PartialSolution& ps, const PrecalculatedMapping& m) : mMapper(l), mGenerator(l, &m), mPartialSolution(ps) {
	size_t numOfRobots = l.robots().size();
	#ifndef NDEBUG
	checkArguments(m, numOfRobots);	// Filled by the heuristics - should be OK.
	#endif
	addVariablesForSelectedDynamicActivities();
	construct(m, numOfRobots > 1);
}

Solution RoboticLineSolverLP::solve() const {

	Solution s;
	high_resolution_clock::time_point start = high_resolution_clock::now();
	SolutionILP sLP = solveILP(mModel, false);
	s.runTime = duration_cast<duration<double>>(high_resolution_clock::now()-start).count();
	s.status = convert(sLP.status);

	if (sLP.status == ILP_OPTIMAL)	{

		s.totalEnergy = sLP.criterion;

		try {
			double minStartTime = F64_MAX;
			map<uint32_t, double> idToStart = inverseMapping(sLP.solution, mMapper.s);
			map<uint32_t, double> idToDuration = inverseMapping(sLP.solution, mMapper.d);

			if (idToStart.size() != idToDuration.size())
				throw SolverException(caller(), "LP model is invalid probably!");

			for (const auto& p : idToStart)
				minStartTime = min(minStartTime, p.second);

			for (const auto& p : idToStart)	{
				uint32_t aid = p.first;
				double start = p.second-minStartTime, duration = getValue(idToDuration, aid, caller());
				setValue(s.startTimeAndDuration, aid, {start, duration}, caller());
			}

			uint32_t numberOfRobots = mPartialSolution.locs.size();
			for (uint32_t r = 0; r < numberOfRobots; ++r)	{
				const vector<Location*>& circuit = mPartialSolution.locs[r];
				const vector<RobotPowerMode*>& modes = mPartialSolution.pwrms[r];
				for (uint32_t i = 0; i+1 < min(circuit.size(), modes.size()); ++i)	{
					Location *l = circuit[i];
					RobotPowerMode *m = modes[i];
					assert(l != nullptr && m != nullptr && l->parent() != nullptr && "Pointers should be initialized!");
					uint32_t point = l->point(), pid = m->pid(), aid = l->parent()->aid();
					setValue(s.pointAndPowerMode, aid, {point, pid}, caller());
				}
			}

		} catch (...)	{
			throw_with_nested(SolverException(caller(), "Cannot map LP solution to the problem solution!\nCheck the validity of the model..."));
		}
	}

	return s;
}

void RoboticLineSolverLP::addCollisionResolution(Activity* i, Activity* j, const int32_t& multipleOfCycleTime)	{
	mGenerator.addCollisionResolution(mMapper.s, mMapper.d, i->aid(), j->aid(), multipleOfCycleTime);
	mGenerator.moveConstraintsToModel(mModel);
}

void RoboticLineSolverLP::addVariablesForSelectedDynamicActivities()	{
	vector<Activity*> selDynActs;
	for (const vector<Movement*>& mvs : mPartialSolution.mvs)	{
		for (Movement *mv : mvs)	{
			assert(mv != nullptr && mv->parent() != nullptr && "Invalid initialization of the RoboticLine data-structure!");
			Activity *da = mv->parent();
			if (da->optional())
				selDynActs.push_back(da);
		}
	}

	#ifdef GUROBI_SOLVER
	mMapper.addActivities(selDynActs, false, false);
	#else
	mMapper.addActivities(selDynActs, false, true);
	#endif
}

void RoboticLineSolverLP::construct(const PrecalculatedMapping& m, bool addTimeLags)	{
	try {
		uint64_t numVars = mMapper.numberOfVariables();
		mModel.c.resize(numVars, 0.0);
		assert(!mModel.c.empty() && "Default coefficient values must be set even for Gurobi!");

		#ifdef GUROBI_SOLVER
		mModel.gurobiC.resize(numVars, nullptr);
		#else
		for (const auto& p : mMapper.W)
			mModel.c[p.second] = 1.0;
		#endif

		uint32_t numberOfRobots = mPartialSolution.locs.size();
		for (uint32_t r = 0; r < numberOfRobots; ++r)	{
			const vector<Location*>& locs = mPartialSolution.locs[r];
			const vector<RobotPowerMode*>& pwrms = mPartialSolution.pwrms[r];
			for (uint32_t i = 0; i+1 < locs.size(); ++i)	{
				assert(locs[i] != nullptr && locs[i]->parent() != nullptr && pwrms[i] != nullptr && "Invalid data structures!");

				uint32_t mappedIdx;
				StaticActivity *sa = locs[i]->parent();
				const auto& dit = mMapper.d.find(sa->aid());
				if (dit != mMapper.d.cend())
					mappedIdx = dit->second;
				else
					throw SolverException(caller(), "Invalid mapping of variables!");

				#ifdef GUROBI_SOLVER
				const auto& sit = m.locToEnergyFunc.find({locs[i], pwrms[i]});
				if (sit != m.locToEnergyFunc.cend())
					mModel.gurobiC[mappedIdx] = &sit->second;
				else
					throw SolverException(caller(), "Cannot find the discretized energy function for the location and power saving mode!");
				#else
				mGenerator.addEnergyFunction(mMapper.d, mMapper.W, locs[i], pwrms[i]);
				#endif

				Variable& d = mMapper.variables[mappedIdx];
				d.lowerBound = max(d.lowerBound, pwrms[i]->minimalDelay());
				assert(d.lowerBound <= d.upperBound && "Invalid variable bound (location)!");
			}
		}


		for (uint32_t r = 0; r < numberOfRobots; ++r)	{
			const vector<Movement*> mvs = mPartialSolution.mvs[r];
			for (Movement* mv : mvs)	{
				assert(mv->parent() != nullptr && "Movement should belong to a dynamic activity!");

				uint32_t mappedIdx;
				DynamicActivity *da = mv->parent();
				const auto& dit = mMapper.d.find(da->aid());
				if (dit != mMapper.d.cend())
					mappedIdx = dit->second;
				else
					throw SolverException(caller(), "Invalid mapping of variables!");

				#ifdef GUROBI_SOLVER
				const auto& sit = m.mvToEnergyFunc.find(mv);
				if (sit != m.mvToEnergyFunc.cend())
					mModel.gurobiC[mappedIdx] = &sit->second;
				else
					throw SolverException(caller(), "Cannot find discretized movement "+to_string(mv->mid())+"!");
				#else
				mGenerator.addEnergyFunction(mMapper.d, mMapper.W, mv);
				#endif

				Variable& d = mMapper.variables[mappedIdx];
				d.lowerBound = max(d.lowerBound, mv->minDuration());
				d.upperBound = min(d.upperBound, mv->maxDuration());
				assert(d.lowerBound <= d.upperBound && "Invalid variable bound (movement)!");
			}

			mGenerator.addSelectedPrecedences(mMapper.s, mMapper.d, mvs);
		}


		if (addTimeLags)
			mGenerator.addTimeLags(mMapper.s, false);

	} catch (...) {
		string msg = "Cannot add constraints to the LP model!\nEither variable mapping or RoboticLine data-structure is corrupted!";
		throw_with_nested(SolverException(caller(), msg));
	}

	mModel.x = move(mMapper.variables);
	mModel.varDesc = move(mMapper.varDesc);
	mGenerator.moveConstraintsToModel(mModel);
	#ifndef NDEBUG
	mModel.checkFormulation();
	#endif
}

void RoboticLineSolverLP::checkArguments(const PrecalculatedMapping& m, const uint32_t& numberOfRobots)	const {
	const PartialSolution& ps = mPartialSolution;
	if (ps.locs.size() != numberOfRobots || ps.pwrms.size() != numberOfRobots || ps.mvs.size() != numberOfRobots)
		throw InvalidArgument(caller(), "Invalid argument 'ps', dimensions mismatch!");

	for (uint32_t r = 0; r < numberOfRobots; ++r)	{
		if (ps.locs[r].size() != ps.pwrms[r].size() || ps.locs[r].empty())
			throw InvalidArgument(caller(), "The number of locations must be the same as the number of power saving modes!");
		if (ps.locs[r].size() != ps.mvs[r].size()+1 || ps.mvs[r].empty())
			throw InvalidArgument(caller(), "The number of movements should be about one less than the number of locations!");

		const vector<Location*>& locs = ps.locs[r];
		const vector<RobotPowerMode*>& pwrms = ps.pwrms[r];
		const vector<Movement*>& mvs = ps.mvs[r];
		if (count(locs.cbegin(), locs.cend(), nullptr) != 0 || count(pwrms.cbegin(), pwrms.cend(), nullptr) != 0 || count(mvs.cbegin(), mvs.cend(), nullptr) != 0)
			throw InvalidArgument(caller(), "Null pointer in the given PartialSolution data-structure!");

		try {
			for (uint32_t i = 0; i+1 < locs.size(); ++i)	{
				Location *from = locs[i], *to = locs[i+1];
				Movement *mv = mvs[i];
				Movement *mvRight = getValue(m.locationsToMovement, {from, to}, caller());
				if (mv != mvRight)
					throw InvalidArgument(caller(), "Given invalid partial solution to the LP solver!");
			}
		} catch (InvalidArgument& e)	{
			throw e;
		} catch (SolverException& e)	{
			throw_with_nested(InvalidArgument(caller(), "Invalid partial solution, a non-existing movement between two locations!"));
		}
	}
}


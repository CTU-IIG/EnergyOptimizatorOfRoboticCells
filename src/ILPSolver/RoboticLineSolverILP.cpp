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
#include <set>
#include <string>
#include "Settings.h"
#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "Shared/Exceptions.h"
#include "ILPModel/SolverInterface.h"
#include "ILPSolver/RoboticLineSolverILP.h"

using namespace std;
using namespace std::chrono;

RoboticLineSolverILP::RoboticLineSolverILP(Robot* r, const PrecalculatedMapping& m) : mMapper(r)  {
	ConstraintsGenerator generator(r, &m);
	construct(generator, false);
}

RoboticLineSolverILP::RoboticLineSolverILP(const RoboticLine& l, const PrecalculatedMapping& m) : mMapper(l)  {
	ConstraintsGenerator generator(l, &m);
	construct(generator, l.robots().size() > 1);
}

Solution RoboticLineSolverILP::solve(double relGap, double timLim) const	{

	#ifdef GUROBI_SOLVER
	initializeLocalEnvironments(1u);
	#endif

	Solution s;
	high_resolution_clock::time_point start = high_resolution_clock::now();
	SolutionILP sILP = solveILP(mModel, Settings::VERBOSE, relGap, timLim, Settings::NUMBER_OF_THREADS);
	s.runTime = duration_cast<duration<double>>(high_resolution_clock::now()-start).count();
	s.status = convert(sILP.status);

	if (sILP.status == ILP_OPTIMAL || sILP.status == ILP_FEASIBLE)	{

		s.lowerBound = sILP.bound;
		s.totalEnergy = sILP.criterion;

		try {
			set<uint32_t> selectedActivities;
			map<uint32_t, double> idToStart = inverseMapping(sILP.solution, mMapper.s);
			map<uint32_t, double> idToDuration = inverseMapping(sILP.solution, mMapper.d);
			map<uint32_t, uint32_t> idToPoint = inverseMapping(sILP.solution, mMapper.x);
			map<uint32_t, uint32_t> idToPowerMode = inverseMapping(sILP.solution, mMapper.z);
			map<uint32_t, uint32_t> idToMovement = inverseMapping(sILP.solution, mMapper.y);

			if (idToStart.size() != idToDuration.size() && idToPoint.size() != idToPowerMode.size())
				throw SolverException(caller(), "ILP model is invalid probably!");

			double minStartTime = F64_MAX;
			for (const auto& p : idToPoint)	{
				uint32_t aid = p.first, point = p.second;
				uint32_t pwrm = getValue(idToPowerMode, aid, caller());
				setValue(s.pointAndPowerMode, aid, {point, pwrm}, caller());
				minStartTime = min(minStartTime, getValue(idToStart, aid, caller()));
				selectedActivities.insert(aid);
			}

			for (const auto& p : idToMovement)	{
				minStartTime = min(minStartTime, getValue(idToStart, p.first, caller()));
				selectedActivities.insert(p.first);
			}

			for (const uint32_t& aid : selectedActivities)	{
				double start = getValue(idToStart, aid, caller())-minStartTime;	// Set initial time of Gantt chart to zero.
				double duration = getValue(idToDuration, aid, caller());
				setValue(s.startTimeAndDuration, aid, {start, duration}, caller());
			}
		} catch (...)	{
			throw_with_nested(SolverException(caller(), "Cannot map ILP solution to the problem solution!\nCheck validity of the model..."));
		}

	} else if (sILP.status != ILP_UNKNOWN)	{
		s.lowerBound = sILP.bound;
	}

	return s;
}

double RoboticLineSolverILP::lowerBoundOnEnergy(const vector<Robot*>& robots, const PrecalculatedMapping& m)	{
	double lowerBound = 0.0;
	for (Robot* r : robots)	{
		RoboticLineSolverILP solver(r, m);
		Solution s = solver.solve(0.0, Settings::RUNTIME_OF_LOWER_BOUND/robots.size());
		lowerBound += s.lowerBound;
	}
	
	return lowerBound;
}

void RoboticLineSolverILP::construct(ConstraintsGenerator& generator, bool addGlobalConstraints)	{

	try {
		generator.addEnergyFunctions(mMapper.d, mMapper.W, mMapper.x, mMapper.z, mMapper.y);
		generator.addUniqueModeSelection(mMapper.x, mMapper.z);
		generator.addFlowConstraints(mMapper.x, mMapper.y);
		generator.addAllPrecedences(mMapper.s, mMapper.d, mMapper.y, mMapper.w);
		generator.addDurationConstraints(mMapper.d, mMapper.z, mMapper.y);
		if (addGlobalConstraints)
			generator.addGlobalConstraints(mMapper.s, mMapper.d, mMapper.x, mMapper.y, mMapper.c);

		mModel.c.resize(mMapper.numberOfVariables(), 0.0);
		for (const auto& p : mMapper.W)
			mModel.c[p.second] = 1.0;

	} catch (...) {
		string msg = "Cannot add constraints to the ILP model!\nEither variable mapping or RoboticLine data-structure is corrupted!";
		throw_with_nested(SolverException(caller(), msg));
	}

	mModel.x = move(mMapper.variables);
	mModel.varDesc = move(mMapper.varDesc);
	generator.moveConstraintsToModel(mModel);
	#ifndef NDEBUG
	mModel.checkFormulation();
	#endif

	if (Settings::VERBOSE == true)	{
		cout<<endl;
		mModel.printStatistics();
		cout<<endl;
	}
}


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

#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "Shared/Exceptions.h"
#include "ILPSolver/VariableMappingLP.h"

using namespace std;

VariableMappingLP::VariableMappingLP(Robot* r) {
	addActivities(r->activities(), true);
}

VariableMappingLP::VariableMappingLP(const RoboticLine& l) {
	for (Robot* r : l.robots())
		addActivities(r->activities(), true);
}

void VariableMappingLP::addActivities(const vector<Activity*>& activities, bool mandatory, bool mapW)	{
	uint32_t idx = numberOfVariables();
	for (Activity *a : activities)	{
		if ((mandatory == true && a->mandatory()) || (mandatory == false && a->optional()))	{
			try {
				// Boolean mapW is useful for Gurobi criterion where variables 'W' can be omitted.
				if (mapW == true)	{
					variables.emplace_back(FLT, 0.0);
					varDesc.emplace_back("W_{"+to_string(a->aid())+"}");
					W.emplace(a->aid(), idx++);
				}

				RoboticLine* l = a->parent()->parent();
				const vector<Robot*>& robots = l->robots();
				variables.emplace_back(FLT, 0.0, l->productionCycleTime()*robots.size());
				varDesc.emplace_back("s_{"+to_string(a->aid())+"}");
				s.emplace(a->aid(), idx++);

				variables.emplace_back(FLT, a->minAbsDuration(), a->maxAbsDuration());
				varDesc.emplace_back("d_{"+to_string(a->aid())+"}");
				d.emplace(a->aid(), idx++);
			} catch (...)	{
				throw_with_nested(SolverException(caller(), "Activity identifications are not unique!"));
			}
		}
	}
}

map<uint32_t, double> inverseMapping(const vector<double>& solution, const map1to1& mapping)	{
	map<uint32_t, double> m;
	for (const auto& p : mapping)	{
		const auto& ret = m.emplace(p.first, solution[p.second]);
		if (ret.second != true)
			throw SolverException(caller(), "Inverse mapping is not unique!");
	}

	return m;
}


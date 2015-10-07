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

#include <cmath>
#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "ILPSolver/VariableMappingILP.h"

using namespace std;

constexpr double BOOL_TOL = 0.001;

VariableMappingILP::VariableMappingILP(Robot* r) : VariableMappingLP(r) {
	try {
		VariableMappingLP::addActivities(r->activities(), false);
		addActivityBinaryVariables(r);
		addActivityOrderBinaryVariables(r);
	} catch (...)	{
		throw_with_nested(SolverException(caller(), "Mapping keys are not unique!"));
	}
}

VariableMappingILP::VariableMappingILP(const RoboticLine& l) : VariableMappingLP(l) {
	try {
		for (Robot* r : l.robots())	// First LP mappings must be created to have right indices.
			VariableMappingLP::addActivities(r->activities(), false);

		for (Robot* r : l.robots())	{
			addActivityBinaryVariables(r);
			addActivityOrderBinaryVariables(r);
		}

		addCollisionBinaryVariables(l);
	} catch (...)	{
		throw_with_nested(SolverException(caller(), "Activity identifications are not unique!"));
	}
}

uint32_t VariableMappingILP::numberOfVariables() const	{
	uint32_t varCount = VariableMappingLP::numberOfVariables();
	varCount += x.size()+z.size()+y.size()+w.size();
	for (const auto& p : c)
		varCount += p.second.size();

	return varCount;
}

void VariableMappingILP::addActivityBinaryVariables(Robot* r)	{
	uint32_t idx = numberOfVariables();
	for (Activity* a : r->activities())	{
		StaticActivity* sa = dynamic_cast<StaticActivity*>(a);
		if (sa != nullptr)	{
			for (Location* l : sa->locations())	{
				variables.emplace_back(BIN, 0.0, 1.0);
				varDesc.emplace_back("x_{"+to_string(sa->aid())+"}#{"+to_string(l->point())+"}");
				setValue(x, {sa->aid(), l->point()}, idx++, caller());
			}

			for (RobotPowerMode* pm : sa->parent()->powerModes())	{
				variables.emplace_back(BIN, 0.0, 1.0);
				varDesc.emplace_back("z_{"+to_string(sa->aid())+"}#{"+to_string(pm->pid())+"}");
				setValue(z, {sa->aid(), pm->pid()}, idx++, caller());
			}
		}

		DynamicActivity* da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr)	{
			for (Movement* mv : da->movements())	{
				variables.emplace_back(BIN, 0.0, 1.0);
				varDesc.emplace_back("y_{"+to_string(da->aid())+"}#{"+to_string(mv->mid())+"}");
				setValue(y, {da->aid(), mv->mid()}, idx++, caller());
			}
		}
	}
}

void VariableMappingILP::addActivityOrderBinaryVariables(Robot* r)	{
	uint32_t idx = numberOfVariables();
	for (Activity* a : r->activities())	{
		DynamicActivity* da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr && da->optional())	{
			Variable var(BIN, 0.0, 1.0);
			variables.push_back(var);
			varDesc.emplace_back("w_{"+to_string(da->aid())+"}#{"+to_string(da->successor()->aid())+"}");
			setValue(w, {da->aid(), da->successor()->aid()}, idx++, caller());
		}
	}
}

void VariableMappingILP::addCollisionBinaryVariables(const RoboticLine& l)	{
	uint32_t idx = numberOfVariables();
	for (const pair<ActivityMode*, ActivityMode*>& col : l.collisions())	{
		int32_t numberOfRobots = l.robots().size();
		ActivityMode *am1 = col.first, *am2 = col.second;
		assert(am1->baseParent() != nullptr && am2->baseParent() != nullptr && "Unexpected null pointers!");
		const Activity *act1 = am1->baseParent(), *act2 = am2->baseParent();
		for (int32_t r = -numberOfRobots; r <= numberOfRobots; ++r)	{
			uint32_t m1 = am1->id(), m2 = am2->id();
			uint32_t a1 = act1->aid(), a2 = act2->aid();

			variables.emplace_back(BIN, 0.0, 1.0);
			varDesc.emplace_back("c_{"+to_string(a1)+","+to_string(m1)+"}#{"+to_string(a2)+","+to_string(m2)+"}#{" +to_string(r)+"}");
			c[{pack(a1,m1), pack(a2,m2)}].push_back(idx++);
		}
	}
}

map<uint32_t, uint32_t> inverseMapping(const vector<double>& solution, const map2to1& mapping)	{
	map<uint32_t, uint32_t> m;
	for (const auto& p : mapping)	{
		if (abs(solution[p.second]-1.0) < BOOL_TOL)
			setValue(m, p.first.first, p.first.second, caller());
	}

	return m;
}


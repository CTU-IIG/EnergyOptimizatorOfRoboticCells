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
#include <iterator>
#include <string>
#include "SolverConfig.h"
#include "Shared/PrecalculatedMapping.h"

using namespace std;

Movement* getSelectedMovement(const Solution& s, const PrecalculatedMapping& m, DynamicActivity* da)	{
	try {
		StaticActivity *pred = da->predecessor(), *suc = da->successor();
		auto p1 = getValue(s.pointAndPowerMode, pred->aid(), caller()), p2 = getValue(s.pointAndPowerMode, suc->aid(), caller());
		return getValue(m.pointsToMovement, {p1.first, p2.first}, caller());
	} catch (...)	{
		throw_with_nested(SolverException(caller(), "Either incomplete solution or flawed PrecalculatedMapping data-structure!"));
	}
}

vector<Location*> getLocations(Location* loc)	{
	return { loc };
}

vector<Location*> getLocations(StaticActivity* act)	{
	return act->locations();
}

template <class X, class Y>
vector<Movement*> findMovements(X from, Y to)	{

	vector<Movement*> foundMvs, leavMvs, entMvs;

	const vector<Location*> &f = getLocations(from), &t = getLocations(to);
	for (Location *loc : f)	{
		const vector<Movement*>& sortedRange1 = leavMvs, &sortedRange2 = loc->leavingMovements();
		vector<Movement*> sortedRange(sortedRange1.size()+sortedRange2.size());
		merge(sortedRange1.cbegin(), sortedRange1.cend(), sortedRange2.cbegin(), sortedRange2.cend(), sortedRange.begin());
		leavMvs = move(sortedRange);
	}

	for (Location *loc : t)	{
		const vector<Movement*>& sortedRange1 = entMvs, &sortedRange2 = loc->enteringMovements();
		vector<Movement*> sortedRange(sortedRange1.size()+sortedRange2.size());
		merge(sortedRange1.cbegin(), sortedRange1.cend(), sortedRange2.cbegin(), sortedRange2.cend(), sortedRange.begin());
		entMvs = move(sortedRange);
	}

	set_intersection(leavMvs.cbegin(), leavMvs.cend(), entMvs.cbegin(), entMvs.cend(), back_inserter(foundMvs));

	return foundMvs;
}

template vector<Movement*> findMovements<StaticActivity*, Location*>(StaticActivity*, Location*);
template vector<Movement*> findMovements<Location*, StaticActivity*>(Location*, StaticActivity*);
template vector<Movement*> findMovements<StaticActivity*, StaticActivity*>(StaticActivity*, StaticActivity*);

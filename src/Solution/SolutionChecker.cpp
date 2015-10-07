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
#include <array>
#include <cmath>
#include <iterator>
#include <typeinfo>
#include <string>
#include <utility>
#include "SolverConfig.h"
#include "Shared/NumericConstants.h"
#include "Solution/SolutionChecker.h"

using namespace std;

bool SolutionChecker::checkAll()	{
	if (mSolution.status == OPTIMAL || mSolution.status == FEASIBLE)	{
		// Feasible solution.
		void (SolutionChecker::* fce[])() = {
			&SolutionChecker::checkCriterion, &SolutionChecker::checkProductionCycleTime, &SolutionChecker::checkStaticActivities,
			&SolutionChecker::checkDynamicActivities, &SolutionChecker::checkScheduleContinuity,
			&SolutionChecker::checkGlobalConstraints, &SolutionChecker::checkCollisionZones
		};

		// Call all checks.
		for (auto f : fce)	{
			try {
				(this->*f)();
			} catch (...)	{
				throw_with_nested(SolverException(caller(), "Mapping or data structures are flawed!"));
			}
		}
	}

	return mErrorMsg.empty();
}

void SolutionChecker::checkCriterion() {
	double consumedEnergy = 0.0;
	for (const auto& p : mSolution.startTimeAndDuration)	{
		Activity* a = getValue(mMapping.aidToActivity, p.first, caller());
		double duration = p.second.second;

		StaticActivity* sa = dynamic_cast<StaticActivity*>(a);
		if (sa != nullptr)	{
			const auto j = getValue(mSolution.pointAndPowerMode, sa->aid(), caller());
			uint32_t lid = getValue(mMapping.pointToLocation, j.first, caller())->lid();
			consumedEnergy += sa->energyConsumption(duration, lid, j.second);
		}

		DynamicActivity* da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr)	{
			Movement *mv = getSelectedMovement(mSolution, mMapping, da);
			consumedEnergy += mv->energyConsumption(duration);
		}
	}

	if (consumedEnergy < 0.0 || abs(consumedEnergy-mSolution.totalEnergy)/(consumedEnergy+F64_EPS) > CRITERION_RTOL)	{
		mErrorMsg.emplace_back("Invalid calculation of the criterion!");
		mErrorMsg.emplace_back("Calculated - "+to_string(consumedEnergy)+" J; given - "+to_string(mSolution.totalEnergy)+" J.");
		mErrorMsg.emplace_back("Discretization of energy functions could be imprecise (more than "+to_string(CRITERION_RTOL*100.0)+" % error)!");
		mErrorMsg.emplace_back("You can try to increase the number of segments for each energy function.");
	}
}


void SolutionChecker::checkProductionCycleTime() {
	for (Robot* r : mLine.robots())	{
		double cycleTime = 0.0;
		for (Activity *a : r->activities())	{
			const auto& sit = mSolution.startTimeAndDuration.find(a->aid());
			if (sit != mSolution.startTimeAndDuration.cend())
				cycleTime += sit->second.second;
		}

		if (abs(cycleTime-mLine.productionCycleTime()) > TIME_TOL)	{
			mErrorMsg.emplace_back("Cycle time for robot '"+r->name()+"' does not correspond to the production cycle time!");
			mErrorMsg.emplace_back("(robot cycle time "+to_string(cycleTime)+")/(production cycle time "+to_string(mLine.productionCycleTime())+")");
		}
	}
}

void SolutionChecker::checkStaticActivities() {
	for (const auto& p : mSolution.pointAndPowerMode)	{
		uint32_t aid = p.first, pwrm = p.second.second;
		double duration = getValue(mSolution.startTimeAndDuration, aid, caller()).second;

		Activity *a = getValue(mMapping.aidToActivity, aid, caller());
		assert(a->parent() != nullptr && "Each activity should belong to some robot!");

		double minDuration = a->minAbsDuration();
		vector<RobotPowerMode*> m = a->parent()->powerModes();
		auto sit = find_if(m.cbegin(), m.cend(), [&pwrm](RobotPowerMode* k) { return k->pid() == pwrm; });
		if (sit != m.cend())	{
			minDuration = max(minDuration, (*sit)->minimalDelay());
			if (duration-minDuration < -TIME_TOL || duration-a->maxAbsDuration() > TIME_TOL)
				mErrorMsg.emplace_back("Static activity "+to_string(aid)+" (power saving mode "+to_string(pwrm)+") has invalid duration!");
		} else {
			mErrorMsg.emplace_back("Cannot find the robot power mode "+to_string(pwrm)+" for static activity "+to_string(aid));
		}
	}
}

void SolutionChecker::checkDynamicActivities()	{
	for (const auto& p : mSolution.startTimeAndDuration)	{
		DynamicActivity *da = dynamic_cast<DynamicActivity*>(getValue(mMapping.aidToActivity, p.first, caller()));
		if (da != nullptr)	{
			double duration = p.second.second;
			Movement *mv = getSelectedMovement(mSolution, mMapping, da);
			if (duration-mv->minDuration() < -TIME_TOL || duration-mv->maxDuration() > TIME_TOL)
				mErrorMsg.emplace_back("Activity "+to_string(da->aid())+"'s movement "+to_string(mv->mid())+" has set invalid duration!");
		}
	}
}

void SolutionChecker::checkScheduleContinuity() {
	for (Robot* r : mLine.robots())	{
		set<uint32_t> activityId;
		for (const auto& p : mSolution.startTimeAndDuration)
			activityId.insert(p.first);

		vector<Activity*> filteredRobotActivities, robotActivities = r->activities();
		copy_if(robotActivities.cbegin(), robotActivities.cend(), back_inserter(filteredRobotActivities),
				[&activityId](Activity* a) { return activityId.count(a->aid()) > 0 ? true : false; });

		uint32_t endId = 0;
		map<uint32_t, Activity*> successors;
		try {
			for (Activity *a : filteredRobotActivities)	{
				DynamicActivity* da = dynamic_cast<DynamicActivity*>(a);
				if (da != nullptr)	{
					Activity *suc = da->successor(), *pred = da->predecessor();
					setValue(successors, pred->aid(), a, caller());
					setValue(successors, a->aid(), suc, caller());
				}

				if (a->lastInCycle())
					endId = a->aid();
			}

			if (successors.size() != filteredRobotActivities.size() || filteredRobotActivities.empty())	{
				mErrorMsg.emplace_back("Uncomplete graph of the solution!");
				return;
			}
		} catch (...)	{
			// Method 'setValue' indirectly checks whether each activity has exactly one successor in the solution graph.
			mErrorMsg.emplace_back("The process flow of the robot '"+r->name()+"' is broken!");
			mErrorMsg.emplace_back("The graph of the solution is either acyclic or disconnected.");
			return;
		}

		bool cycleFinished = false;
		Activity *curAct = getValue(successors, endId, caller());
		while (!cycleFinished)	{
			const auto p = getValue(mSolution.startTimeAndDuration, curAct->aid(), caller());
			double startCur = p.first, duration = p.second;
			Activity *next = getValue(successors, curAct->aid(), caller());
			double startNext = getValue(mSolution.startTimeAndDuration, next->aid(), caller()).first;

			if (startNext-startCur < -TIME_TOL || abs(startNext-startCur-duration) > TIME_TOL)	{
				mErrorMsg.emplace_back("The process flow of the robot '"+r->name()+"' is broken!");
				mErrorMsg.emplace_back("Check start times and activity durations!");
				return;
			}

			if (next->aid() == endId)
				cycleFinished = true;
			else
				curAct = next;
		}
	}
}

void SolutionChecker::checkGlobalConstraints() {
	double productionCycleTime = mLine.productionCycleTime();
	for (InterRobotOperation* o : mLine.interRobotOperations())	{
		for (const TimeLag& tl : o->timeLags())	{
			Activity *from = tl.from(), *to = tl.to();
			const auto& sit1 = mSolution.startTimeAndDuration.find(from->aid());
			const auto& sit2 = mSolution.startTimeAndDuration.find(to->aid());
			if (sit1 != mSolution.startTimeAndDuration.cend() && sit2 != mSolution.startTimeAndDuration.cend())	{
				double s1 = sit1->second.first, s2 = sit2->second.first;
				if (s2-s1+TIME_TOL < tl.length()-productionCycleTime*tl.height())	{
					mErrorMsg.emplace_back("Time lag '"+to_string(from->aid())+" -> "+to_string(to->aid())+"' is broken!");
					mErrorMsg.emplace_back("Check the formulation of the problem.");
				}
			}
		}
	}

	for (const auto& t : mMapping.sptCompVec)	{
		Activity *a1 = get<2>(t), *a2 = get<3>(t);
		try {
			const array<uint32_t, 2>& m = extractModes(a1, a2);
			const vector<pair<Location*, Location*>>& modes = get<4>(t);
			const auto sit = find_if(modes.cbegin(), modes.cend(), [=](pair<Location*, Location*> p) { return m[0] == p.first->lid() && m[1] == p.second->lid(); });
			if (sit == modes.cend())	{
				mErrorMsg.emplace_back("Incompatible modes for activities "+to_string(a1->aid())+" and "+to_string(a2->aid())+"!");
				mErrorMsg.emplace_back("Check the used algorithm!");
			}
		} catch (...)	{
			string msg = "Cannot find the modes for static activities "+to_string(a1->aid())+" and "+to_string(a2->aid())+"!";
			throw_with_nested(SolverException(caller(), msg+"\nCorrupted data structures!"));
		}
	}
}

void SolutionChecker::checkCollisionZones()	{
	for (const pair<ActivityMode*, ActivityMode*>& col : mLine.collisions())	{
		try {
			ActivityMode *am1 = col.first, *am2 = col.second;
			uint32_t m1 = am1->id(), m2 = am2->id();
			Activity *a1 = am1->baseParent(), *a2 = am2->baseParent();

			array<uint32_t, 2> selectedModes = extractModes(a1, a2);
			if (selectedModes[0] == m1 && selectedModes[1] == m2)	{
				double cycleTime = mLine.productionCycleTime();
				const auto& p1 = getValue(mSolution.startTimeAndDuration, a1->aid(), caller());
				const auto& p2 = getValue(mSolution.startTimeAndDuration, a2->aid(), caller());

				double d1 = p1.second, d2 = p2.second;
				double s1 = fmod(p1.first, cycleTime), s2 = fmod(p2.first, cycleTime);
				if (s1 > s2)	{
					swap(s1, s2);
					swap(d1, d2);
				}

				if (!(s2+d2-s1 <= cycleTime+TIME_TOL && s1+d1 <= s2+TIME_TOL))	{
					mErrorMsg.emplace_back("Collision for activity "+to_string(a1->aid())+" (mode "+to_string(m1)
							+") and "+to_string(a2->aid())+" (mode "+to_string(m2)+") not resolved!");
					mErrorMsg.emplace_back("Please check the collision resolution in the selected algorithm.");
				}
			}
		} catch (...)	{
			// Collision not applicable...
		}
	}
}

array<uint32_t, 2> SolutionChecker::extractModes(Activity* a1, Activity* a2) const	{

	uint32_t i = 0;
	array<uint32_t, 2> mode = {{ 0xffffffff, 0xffffffff }};
	for (Activity* a : {a1, a2})	{
		StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
		if (sa != nullptr)	{
			uint32_t point = getValue(mSolution.pointAndPowerMode, sa->aid(), caller()).first;
			mode[i] = getValue(mMapping.pointToLocation, point, caller())->lid();
		}

		DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr)
			mode[i] = getSelectedMovement(mSolution, mMapping, da)->mid();

		++i;
	}

	return mode;
}


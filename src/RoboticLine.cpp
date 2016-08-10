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
#include <map>
#include <mutex>
#include <stdexcept>
#include <typeinfo>
#include "RoboticLine.h"
#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "Shared/Exceptions.h"
#include "Shared/NumericConstants.h"

using namespace std;

double Movement::energyConsumption(const double& duration) const {
	assert(duration > 0.0 && mMinDuration <= duration+TIME_ERR && duration <= mMaxDuration+TIME_ERR && "Invalid duration of the movement!");

	double consumption = 0.0;
	for (const Monomial& m : mEnergyFunction)
		consumption += m.coeff*pow(duration, m.degree);

	return consumption;
}

double Location::inputPower(RobotPowerMode* m) const	{
	double inputPower = 0.0;
	if (m != nullptr)
		inputPower = m->expectedInputPower();
	else
		throw InvalidArgument(caller(), "Malicious attempt to pass a null pointer as an argument!");

	for (const auto& ldpc : mLocationDependentPowerConsumption)	{
		if (ldpc.robotPowerMode() == m)
			inputPower = ldpc.inputPower();
	}

	return inputPower;
}

double Location::energyConsumption(const double& duration, RobotPowerMode* m) const	{
	assert(duration >= 0.0 && m->minimalDelay() <= duration+TIME_ERR && mParent->minAbsDuration() <= duration+TIME_ERR && "Invalid duration for the location!");
	return inputPower(m)*duration;
}

void Location::parent(StaticActivity *parent)	{
	mParent = parent;
	for (LocationDependentPowerConsumption& ldpc : mLocationDependentPowerConsumption)
		ldpc.parent(this);
}

Movement* DynamicActivity::findMovement(const uint32_t& mid) const	{
	for (Movement* mv : mMovements)	{
		if (mv->mid() == mid)
			return mv;
	}

	string msg = "Cannot find dynamic activity " + to_string(mAid) + "'s movement with 'mid=" + to_string(mid) + "'!";
	throw InvalidArgument(caller(), msg);
}

StaticActivity* DynamicActivity::cessor(const string& prefix) const	{

	const vector<Activity*>& related = (prefix == "suc" ? mSuccessors : mPredecessors);

	if (related.size() == 1)	{
		StaticActivity *sa = dynamic_cast<StaticActivity*>(related.front());
		if (sa != nullptr)	{
			return sa;
		} else	{
			string msg = "The " + prefix + "cessor of the dynamic activity " + to_string(mAid) + " must be static activity!";
			throw SolverException(caller(), msg);
		}
	} else {
		string msg = "Dynamic activity " + to_string(mAid) + " must have exactly one " + prefix + "cessor!";
		throw SolverException(caller(), msg);
	}
}

bool DynamicActivity::mandatory() const	{
	if (mPredecessors.size() == 1)
		return (predecessor()->successors().size() == 1) ? true : false;
	else
		throw SolverException(caller(), "Predecessors were not set!");
}

void DynamicActivity::parent(Robot *r) {
	mParent = r;
	for (Movement *mv : mMovements)
		mv->parent(this);
}

void DynamicActivity::freeAllocatedMemory()	{
	for (Movement *mv : mMovements)
		delete mv;
}

void StaticActivity::findMovementsForLocations()	{
	map<Location*, vector<Movement*> > enteringMovements, leavingMovements;
	for (Activity *a : predecessors())	{
		DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr)	{
			for (Movement *mv : da->movements())
				enteringMovements[mv->to()].push_back(mv);
		} else {
			string pa = to_string(a->aid()), ca = to_string(aid());
			string msg = "Static activity "+ca+"'s predecessor "+pa+" should be dynamic activity!";
			throw SolverException(caller(), msg);
		}
	}

	for (Activity *a : successors())	{
		DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr)	{
			for (Movement* mv : da->movements())
				leavingMovements[mv->from()].push_back(mv);
		} else {
			string sa = to_string(a->aid()), ca = to_string(aid());
			string msg = "Static activity "+ca+"'s successor "+sa+" should be dynamic activity!";
			throw SolverException(caller(), msg);
		}
	}

	for (Location* loc : mLocations) {
		auto ite = enteringMovements.find(loc);
		auto itl = leavingMovements.find(loc);
		if (ite != enteringMovements.cend() && itl != leavingMovements.cend())	{
			vector<Movement*>& entMvs = ite->second, &leavMvs = itl->second;
			// Sort movements for the fast search, do not remove!
			sort(entMvs.begin(), entMvs.end());
			sort(leavMvs.begin(), leavMvs.end());
			loc->enteringMovements(entMvs);
			loc->leavingMovements(leavMvs);
		} else {
			string ca = to_string(aid()), pt = to_string(loc->point());
			string msg = "Static activity "+ca+" has isolated point "+pt+"!";
			throw InvalidDatasetFile(caller(), msg);
		}
	}
}

Location* StaticActivity::findLocation(const uint32_t& lid) const	{
	for (Location* l : mLocations)	{
		if (l->lid() == lid)
			return l;
	}

	string msg = "Cannot find static activity " + to_string(mAid) + "'s location with 'lid=" + to_string(lid) + "'!";
	throw InvalidArgument(caller(), msg);
}

double StaticActivity::inputPower(const uint32_t& lid, const uint32_t& pid) const	{
	assert(mParent != nullptr && "Pointers should be initialized by the InstancesReader class!");

	RobotPowerMode *mode = nullptr;
	Location *loc = findLocation(lid);
	const vector<RobotPowerMode*>& modes = mParent->powerModes();
	auto sit = find_if(modes.cbegin(), modes.cend(), [&pid](RobotPowerMode* m) { return m->pid() == pid; });
	if (sit != modes.cend())
		mode = *sit;
	else
		throw InvalidArgument(caller(), "Invalid 'pid' argument!");

	return loc->inputPower(mode);
}

double StaticActivity::energyConsumption(double duration, const uint32_t& lid, const uint32_t& pid) const	{
	assert(duration >= 0.0 && "Invalid duration value!");
	return inputPower(lid, pid)*duration;
}

void StaticActivity::parent(Robot *r)	{
	mParent = r;
	for (Location* loc : mLocations)
		loc->parent(this);
}

void StaticActivity::freeAllocatedMemory()	{
	for (Location* loc : mLocations)
		delete loc;
}

RobotPowerMode* Robot::fastestPowerSavingMode() const	{
	RobotPowerMode* fastest = nullptr;
	for (RobotPowerMode* pwrm : mRobotModes)	{
		if (fastest == nullptr || fastest->minimalDelay() > pwrm->minimalDelay())
			fastest = pwrm;
	}

	assert(fastest != nullptr && "Invalid data-structure! Each robot has to have some power saving modes!");

	return fastest;
}

void Robot::parent(RoboticLine* parent)	{
	mParent = parent;
	for (Activity *a : mActivities)
		a->parent(this);
	for (RobotPowerMode *m : mRobotModes)
		m->parent(this);
}

void Robot::setActivitiesRelations(const map<uint32_t, Location*>& pointToLocation, const map<Movement*, pair<uint32_t, uint32_t>>& movementToPoints)	{
	for (Activity* a : mActivities)	{
		DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
		if (da != nullptr)	{
			StaticActivity *sa1 = nullptr, *sa2 = nullptr;
			for (Movement* mv : da->movements())	{
				const auto& points = getValue(movementToPoints, mv, caller());
				uint32_t from = points.first, to = points.second;
				auto fSit = pointToLocation.find(from), tSit = pointToLocation.find(to);
				if (fSit == pointToLocation.cend() || tSit == pointToLocation.cend())	{
					string msg = "Invalid movement "+to_string(mv->mid());
					msg += " from point "+to_string(from)+" to point "+to_string(to)+"!";
					msg += "\nThe point is not connected with the static activity!";
					throw InvalidDatasetFile(caller(), msg);
				}

				assert(fSit->second->parent() != nullptr && tSit->second->parent() != nullptr && "Should be set by XmlReader class!");
				Activity* a1 = fSit->second->parent(), *a2 = tSit->second->parent();

				if (sa1 == nullptr || sa2 == nullptr)	{
					sa1 = dynamic_cast<StaticActivity*>(a1);
					sa2 = dynamic_cast<StaticActivity*>(a2);
					if (sa1 == nullptr || sa2 == nullptr)	{
						string msg = "Dynamic activity "+to_string(da->aid());
						msg += " must be interconnected by two static activities!";
						msg += "\nCheck type of activities "+to_string(a1->aid())+" and "+to_string(a2->aid())+".";
						throw InvalidDatasetFile(caller(), msg);
					}
				} else {
					if (sa1 != a1 || sa2 != a2)	{
						string msg = "Dynamic activity "+to_string(da->aid());
						msg += " can interconnect only two static activities!";
						throw InvalidDatasetFile(caller(), msg);
					}
				}

				mv->from(fSit->second);
				mv->to(tSit->second);

				// Filter monomials with zero coefficients.
				auto it = remove_if(mv->mEnergyFunction.begin(), mv->mEnergyFunction.end(), [](Monomial m) { return m.coeff == 0.0; });
				mv->mEnergyFunction.erase(it, mv->mEnergyFunction.end());
			}

			if (sa1 != nullptr && sa2 != nullptr)	{
				sa1->addSuccessor(da);
				da->addPredecessor(sa1);
				da->addSuccessor(sa2);
				sa2->addPredecessor(da);
			} else {
				string msg = "There are no movements for the dynamic activity "+to_string(da->aid())+"!";
				throw InvalidDatasetFile(caller(), msg);
			}
		}
	}

	for (Activity *a : mActivities)	{
		StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
		if (sa != nullptr)
			sa->findMovementsForLocations();
	}
}

void Robot::freeAllocatedMemory()	{
	for (Activity *a : mActivities)	{
		a->freeAllocatedMemory();
		delete a;
	}

	for (RobotPowerMode *m : mRobotModes)
		delete m;
}

mutex refCounterMutex;
map<int64_t*, set<RoboticLine*>> RoboticLine::liveInstances;

RoboticLine::RoboticLine() : mRefCounter(new int64_t(1)), mProductionCycleTime(-1) {
	refCounterMutex.lock();
	liveInstances[mRefCounter].insert(this);
	refCounterMutex.unlock();
}

RoboticLine::RoboticLine(const RoboticLine& l)	{
	refCounterMutex.lock();
	++*l.mRefCounter;
	liveInstances[l.mRefCounter].insert(this);
	refCounterMutex.unlock();
	copyDataStructures(l);
}

RoboticLine& RoboticLine::operator=(const RoboticLine& l)	{
	if (this != &l)	{
		refCounterMutex.lock();
		++*l.mRefCounter;
		liveInstances[mRefCounter].erase(this);
		liveInstances[l.mRefCounter].insert(this);
		freeAllocatedMemory();
		refCounterMutex.unlock();
		copyDataStructures(l);
	}

	return *this;
}

void RoboticLine::setParentOfChildren(RoboticLine* rl)	{
	for (InterRobotOperation* op : mInterRobotOperations)
		op->parent(rl);
	for (Robot* r : mRobots)
		r->parent(rl);
}

void RoboticLine::initialiseDataStructures(const map<uint32_t, Location*>& pointToLocation, const map<Movement*, pair<uint32_t, uint32_t>>& movementToPoints)	{
	setParentOfChildren(this);
	for (Robot* r : mRobots)
		r->setActivitiesRelations(pointToLocation, movementToPoints);
}

void RoboticLine::copyDataStructures(const RoboticLine& l)	{
	mRefCounter = l.mRefCounter; mName = l.mName; mDescription = l.mDescription;
	mRobots = l.mRobots; mInterRobotOperations = l.mInterRobotOperations;
	mCollisions = l.mCollisions; mProductionCycleTime = l.mProductionCycleTime;
}

RoboticLine::~RoboticLine()	{
	refCounterMutex.lock();
	liveInstances[mRefCounter].erase(this);
	freeAllocatedMemory();
	refCounterMutex.unlock();
}

void RoboticLine::freeAllocatedMemory()	{
	if (--*mRefCounter <= 0)	{
		for (Robot *r : mRobots)	{
			r->freeAllocatedMemory();
			delete r;
		}

		for (InterRobotOperation *op : mInterRobotOperations)
			delete op;

		delete mRefCounter;
	} else {
		// Cannot free the memory, more references to the data still exists, reset parent to an existing robotic cell.
		if (!liveInstances[mRefCounter].empty())	{
			setParentOfChildren(*liveInstances[mRefCounter].begin());
		} else {
			refCounterMutex.unlock();
			throw SolverException(caller(), "Corrupted robotic cell data-structure, memory handling is invalid!");
		}
	}
}


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
#include <set>
#include <utility>
#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "Shared/Exceptions.h"
#include "Shared/Algorithms.h"
#include "Shared/Exceptions.h"
#include "Shared/NumericConstants.h"
#include "InstancesReader/InstancesReader.h"
#include "InstancesReader/XmlReader.h"

using namespace std;

void InstancesReader::readInstances()	{
	XmlReader reader;
	reader.readDatasetFromXmlFile();
	mLines = move(reader.mLines);
	mMapping = move(reader.mMapping);
	mDatasetName = move(reader.mDatasetName);
	mDatasetDescription = move(reader.mDatasetDescription);

	for (const RoboticLine& l : mLines)	{
		InstanceChecker checker(l);
		checker.checkInstance();
	}

	completeFillingOfMapping();
}

void InstancesReader::completeFillingOfMapping()	{
	for (size_t i = 0; i < min(mLines.size(), mMapping.size()); ++i)	{
		const RoboticLine& line = mLines[i];
		PrecalculatedMapping& mapping = mMapping[i];
		fillActivitiesRelations(mapping, line);
		fillDiscretizedEnergyFunctions(mapping, line);
		fillInterRobotOperations(mapping, line);
		fillCollisionZones(mapping, line);
	}
}

void InstancesReader::fillActivitiesRelations(PrecalculatedMapping& mapping, const RoboticLine& line)	{
	uint64_t numberOfMovements = 0;
	for (Robot* robot : line.robots())	{
		for (Activity* a : robot->activities())	{
			DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
			if (da != nullptr)
				numberOfMovements += da->movements().size();
		}
	}

	mapping.locationsToMovement.reserve(numberOfMovements);
	mapping.pointsToMovement.reserve(numberOfMovements);

	for (Robot* robot : line.robots())	{
		for (Activity* a : robot->activities())	{
			DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
			if (da != nullptr)	{
				for (Movement *mv : da->movements())	{
					try {
						Location *f = mv->from(), *t = mv->to();
						setValue(mapping.locationsToMovement, {f, t}, mv, caller());
						setValue(mapping.pointsToMovement, {f->point(), t->point()}, mv, caller());
						mapping.pointToSuccessorPoints[f->point()].push_back(t->point());
						mapping.pointToPredecessorPoints[t->point()].push_back(f->point());
					} catch (...)	{
						throw_with_nested(InvalidDatasetFile(caller(), "Only one move is allowed between two points!"));
					}
				}
			}
		}
	}
}

void InstancesReader::fillDiscretizedEnergyFunctions(PrecalculatedMapping& mapping, const RoboticLine& line)	{
	for (Robot* robot : line.robots())	{
		for (Activity* a : robot->activities())	{
			DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
			if (da != nullptr)	{
				for (Movement *mv : da->movements())	{
					const uint32_t numberOfSegments = Settings::NUMBER_OF_SEGMENTS;
					const double dFrom = mv->minDuration(), dTo = mv->maxDuration();
					const double h = (dTo-dFrom)/(10.0*numberOfSegments), hsq = h*h, h2 = 2.0*h, h01 = 0.1*h;

					// Calculate second derivates (direction changes) of the energy function.
					vector<pair<double, double>> penalty;
					double d1 = dFrom, d2 = dFrom+h, d3 = dFrom+h2;
					double e1 = mv->energyConsumption(d1), e2 = mv->energyConsumption(d2), e3, secondDiff, sumOfPenalties = 0.0;
					for (double d = d3; d < dTo+h01; d += h)	{
						e3 = mv->energyConsumption(d);
						secondDiff = abs(e3-2*e2+e1)/hsq;
						penalty.emplace_back(d-h, secondDiff);
						sumOfPenalties += secondDiff;
						e1 = e2; e2 = e3;
					}

					// Add extra penalty (higher penalty smaller distance between points) for prolongation.
					double penaltyForProlongation = sumOfPenalties/penalty.size();
					for (auto& p : penalty)
						p.second += penaltyForProlongation;
					sumOfPenalties *= 2.0;

					// Discretize the function according to the penalty.
					vector<double> d = { dFrom };
					vector<double> e = { mv->energyConsumption(dFrom) };
					double accumulated = 0.0, toAccumulateForPoint = sumOfPenalties/numberOfSegments;
					for (const pair<double, double>& p : penalty)	{
						if (accumulated >= toAccumulateForPoint)	{
							d.push_back(p.first);
							e.push_back(mv->energyConsumption(p.first));
							accumulated -= toAccumulateForPoint;
						}
						accumulated += p.second;
					}
					d.push_back(dTo);
					e.push_back(mv->energyConsumption(dTo));

					tuple<vector<double>, vector<double>> ef { d, e };
					setValue(mapping.mvToEnergyFunc, mv, ef, caller());
				}
			}

			StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr)	{
				for (Location *l : sa->locations())	{
					for (RobotPowerMode *pwrm : robot->powerModes())	{
						double inputPower = l->inputPower(pwrm);
						vector<double> x = { 0.0, 1.0 }, y = { 0.0, inputPower };
						tuple<vector<double>, vector<double>> ef { x, y };
						setValue(mapping.locToEnergyFunc, {l, pwrm}, ef, caller());
					}
				}
			}
		}
	}
}

void InstancesReader::fillInterRobotOperations(PrecalculatedMapping& mapping, const RoboticLine& line)	{
	const vector<Robot*>& robots = line.robots();
	map<StaticActivity*, uint32_t> activityToRobotId;
	for (uint32_t r = 0; r < robots.size(); ++r)	{
		for (Activity *a : robots[r]->activities())	{
			StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr)
				setValue(activityToRobotId, sa, r, caller());
		}
	}

	for (InterRobotOperation* op : line.interRobotOperations())	{
		for (const pair<Location*, Location*>& p : op->spatialCompatibility())	{
			assert(p.first != nullptr && p.second != nullptr && p.first->parent() != nullptr && p.second->parent() != nullptr && "Unexpected null pointers!");
			Location *l1 = p.first, *l2 = p.second;
			StaticActivity *a1 = l1->parent(),  *a2 = l2->parent();

			bool checkOk = true;
			const auto& mit1 = mapping.sptComp.find(a1);
			if (mit1 != mapping.sptComp.cend() && mit1->second.count(a2) == 0)
				checkOk = false;	// Spatial compatibility supported only between pairs of activities...

			if (a1 != nullptr && a2 != nullptr && checkOk == true)	{
				mapping.sptComp[a1][a2].push_back({l1, l2});
				mapping.sptComp[a2][a1].push_back({l2, l1});
			} else {
				string msg;
				if (checkOk == false)	{
					msg += "Spatial compatibility is only supported between pairs of static activities.";
				} else	{
					msg += "Spatial compatibility must be between two static activities!\n";
					msg += "Was the instance checked against the xml schema?!";
				}

				throw InvalidDatasetFile(caller(), msg);
			}
		}

		for (TimeLag tl : op->timeLags())	{
			mapping.timeLagsFromAct[tl.from()].push_back(tl);
			mapping.timeLagsToAct[tl.to()].push_back(tl);
		}
	}

	set<pair<StaticActivity*, StaticActivity*>> uniquePairs;
	for (const auto& mit1 : mapping.sptComp)	{
		StaticActivity *a1 = mit1.first;
		uint32_t r1 = getValue(activityToRobotId, a1, caller());
		for (const auto& mit2 : mit1.second)	{
			StaticActivity *a2 = mit2.first;
			uint32_t r2 = getValue(activityToRobotId, a2, caller());
			if (uniquePairs.count({a1, a2}) == 0 && uniquePairs.count({a2, a1}) == 0)	{
				auto compatibilityPairs = mit2.second;
				mapping.sptCompVec.emplace_back(r1, r2, a1, a2, compatibilityPairs);
				uniquePairs.insert({a1, a2});
			}
		}
	}
}

void InstancesReader::fillCollisionZones(PrecalculatedMapping& mapping, const RoboticLine& line)	{
	for (const pair<ActivityMode*, ActivityMode*>& collision : line.collisions())	{
		mapping.collisionSearch[collision.first].insert(collision.second);
		mapping.collisionSearch[collision.second].insert(collision.first);
	}
}

void InstanceChecker::checkInstance() const	{
	for (Robot* r : mLine.robots())	{
		uint32_t lastInCycleCounter = 0;
		for (Activity* a : r->activities())	{
			StaticActivity *sa = dynamic_cast<StaticActivity*>(a);
			DynamicActivity *da = dynamic_cast<DynamicActivity*>(a);
			if (sa != nullptr)	{
				checkStaticActivity(sa);
				if (sa->lastInCycle())
					lastInCycleCounter++;
			}
			if (da != nullptr)	{
				checkDynamicActivity(da);
			}
		}

		if (lastInCycleCounter != 1)
			throw InvalidDatasetFile(caller(), "Robot '"+r->name()+"' must have one static activity which ends the cycle!");

		checkRobotGraph(r);
	}

	for (InterRobotOperation* op : mLine.interRobotOperations())
		checkOperation(op);

	for (const pair<ActivityMode*, ActivityMode*>& col : mLine.collisions())
		checkCollision(col);
}

void InstanceChecker::checkStaticActivity(StaticActivity* sa) const {
	if (sa->minAbsDuration() > sa->maxAbsDuration())	{
		string msg = "Invalid duration of static activity "+to_string(sa->aid())+" or instance trivially infeasible!";
		throw InvalidDatasetFile(caller(), msg);
	}

	for (Location* l : sa->locations())	{
		set<uint32_t> pwrModeIds;
		for (const LocationDependentPowerConsumption& ldpc : l->locationDependentPowerConsumption())
			pwrModeIds.insert(ldpc.robotPowerMode()->pid());

		assert(sa->parent() != nullptr && "Parent should be set by the XmlReader class!");

		Robot *robot = sa->parent();
		for (const RobotPowerMode *pm : robot->powerModes())	{
			if (pwrModeIds.count(pm->pid()) == 0 && pm->expectedInputPower() < 0.0)	{
				string msg = "Robot power mode "+to_string(pm->pid()) + " has to have either defined expected input power or ";
				msg += "if it is location depedent the power consumption for activity " + to_string(sa->aid());
				msg += " must be stated in 'consumption' element of location " + to_string(l->lid()) + "!";
				throw InvalidDatasetFile(caller(), msg);
			}
		}
	}
}

void InstanceChecker::checkDynamicActivity(DynamicActivity* da) const {
	for (Movement *mv : da->movements())	{
		if (mv->minDuration() > mv->maxDuration())	{
			string msg = "Invalid duration of dynamic activity " + to_string(da->aid()) + "'s movement " + to_string(mv->mid()) + "!";
			throw InvalidDatasetFile(caller(), msg);
		}

		for (const Monomial& m : mv->energyFunction())	{
			if (m.degree != 0 && m.degree != 1 && m.coeff < 0.0)	{
				string msg = "Energy function of movement " + to_string(mv->mid()) + " (activity ";
				msg += to_string(da->aid()) + ") is not convex!\nPlease update coefficients!";
				throw InvalidDatasetFile(caller(), msg);
			}
		}
	}
}

void InstanceChecker::checkRobotGraph(Robot* r) const {

	const vector<Activity*>& activities = r->activities();

	const uint64_t numAct = activities.size();
	DistanceMatrix<double> distMatrix(numAct, vector<double>(numAct, F64_INF));

	for (uint64_t a = 0; a < numAct; ++a)	{

		const vector<Activity*>& pred = activities[a]->predecessors();
		const vector<Activity*>& succ = activities[a]->successors();

		uint8_t phase = 0;
		for (const vector<Activity*>& predOrSucc : {pred, succ})	{
			for (Activity *ps : predOrSucc)	{
				if (ps->parent() == activities[a]->parent())	{
					uint64_t psIdx = (uint64_t) (find(activities.cbegin(), activities.cend(), ps)-activities.cbegin());
					if (phase == 0)
						distMatrix[psIdx][a] = 1.0;
					else
						distMatrix[a][psIdx] = 1.0;
				} else {
					string msg = "Activity " + to_string(activities[a]->aid());
					if (phase == 0)
						msg += " has predecessor ";
					else
						msg += " has successor ";
					msg += to_string(ps->aid()) + " from different robot!";

					throw InvalidDatasetFile(caller(), msg);
				}
			}
			++phase;
		}
	}

	distMatrix = floyd(distMatrix);

	for (uint32_t i = 0; i < numAct; ++i)	{
		for (uint32_t j = 0; j < numAct; ++j)	{
			if (distMatrix[i][j] == F64_INF)	{
				string msg = "Robot '" + r->name() + "' has not enough connected graph of activities!";
				throw InvalidDatasetFile(caller(), msg);
			}
		}
	}
}

void InstanceChecker::checkOperation(InterRobotOperation* op) const	{
	for (const TimeLag& lag : op->timeLags())	{
		if (lag.from()->parent() == lag.to()->parent())	{
			string msg = "Time-lag from activity "+to_string(lag.from()->aid()) + " to activity " + to_string(lag.to()->aid());
			msg += " is invalid. The edge must be between two different robots!";
			msg += "\nCheck operation " + to_string(op->oid()) + ".";
			throw InvalidDatasetFile(caller(), msg);
		}
	}

	for (const pair<Location*, Location*>& p : op->spatialCompatibility())	{

		assert(p.first != nullptr && p.second != nullptr && p.first->parent() != nullptr && p.second->parent() != nullptr && "Unexpected null pointers!");
		StaticActivity *sa1 = p.first->parent(),  *sa2 = p.second->parent();
		assert(sa1->parent() != nullptr && sa2->parent() != nullptr && "Unexpected null pointers, it should already be initialized!");

		if (sa1->parent() == sa2->parent())	{
			string msg = "Static activities "+to_string(sa1->aid()) + " and " + to_string(sa2->aid()) + " must be located at different robots!";
			msg += "\nPlease check operation " + to_string(op->oid()) + "'s spatial compatibility pairs!";
			throw InvalidDatasetFile(caller(), msg);
		}
	}
}

void InstanceChecker::checkCollision(const pair<ActivityMode*, ActivityMode*>& col) const	{
	assert(col.first != nullptr && col.second != nullptr && "Unexpected null pointers!");
	const Activity *a1 = col.first->baseParent(), *a2 = col.second->baseParent();
	assert(a1 != nullptr && a2 != nullptr && a1->parent() != nullptr && a2->parent() != nullptr && "Unexpected null pointers!");
	Robot *r1 = a1->parent(), *r2 = a2->parent();
	if (r1 == r2)	{
		string msg = "Collision can occur only between two different robots!";
		msg += "\nPlease check the collision pair with activities " + to_string(a1->aid());
		msg += " and " + to_string(a2->aid()) + "!";
		throw InvalidDatasetFile(caller(), msg);
	}
}


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

#include <iterator>
#include "Settings.h"
#include "SolverConfig.h"
#include "Shared/Utils.h"
#include "ILPSolver/ConstraintsGenerator.h"
#define UB_W 1000000.0

using namespace std;

void ConstraintsGenerator::reset() {
	mA.clear(); mOp.clear(); mB.clear(); mConDesc.clear();
}

void ConstraintsGenerator::reset(Robot* r, const PrecalculatedMapping* m) {
	assert(r != nullptr && "Invalid pointer to robot!");
	mRobots = {r}; mLine = nullptr; mMapping = m;
	initializeDataStructures();
}

void ConstraintsGenerator::reset(const RoboticLine& l, const PrecalculatedMapping* m)	{
	mRobots = l.robots(); mLine = &l; mMapping = m;
	initializeDataStructures();
}

uint64_t ConstraintsGenerator::moveConstraintsToModel(ILPModel& m)	{
	uint64_t numberOfMovedConstraints = mA.numberOfRows();

	for (uint32_t r = 0; r < numberOfMovedConstraints; ++r)
		m.A.addRow(mA[r]);

	m.ops.insert(m.ops.end(), make_move_iterator(mOp.begin()), make_move_iterator(mOp.end()));
	m.b.insert(m.b.end(), make_move_iterator(mB.begin()), make_move_iterator(mB.end()));
	m.conDesc.insert(m.conDesc.end(), make_move_iterator(mConDesc.begin()), make_move_iterator(mConDesc.end()));

	reset();

	return numberOfMovedConstraints;
}

// a_{i,p}^m*d_i-\overline{W}*(2-z_i^m-x_i^p) \leq W_i (1)
void ConstraintsGenerator::addEnergyFunctions1(const map1to1& d, const map1to1& W, const map2to1& x, const map2to1& z)	{
	for (StaticActivity* sa : mStaticActivities)	{
		for (Location* l : sa->locations())	{
			assert(sa->parent() != nullptr && "Each static activity had to have assigned a robot!");
			for (RobotPowerMode* pm : sa->parent()->powerModes())	{
				double b = 2*UB_W; Operator op = LESS_EQUAL;
				double inputPower = sa->inputPower(l->lid(), pm->pid());
				SparseMatrix<double>::Row row = {
					{getValue(d, sa->aid(), caller()), inputPower}, {getValue(z, {sa->aid(), pm->pid()}, caller()), UB_W},
					{getValue(x, {sa->aid(), l->point()}, caller()), UB_W}, {getValue(W, sa->aid(), caller()), -1.0}
				};

				addConstraint(row, op, b, "(1$"+to_string(mConstraintsCounter++)+")");
			}
		}
	}
}

// a_{i,k}^{t}*d_i+b_{i,k}^t-\overline{W}*(1-y_i^t) \leq W_i (2)
void ConstraintsGenerator::addEnergyFunctions2(const map1to1& d, const map1to1& W, const map2to1& y)	{
	for (DynamicActivity* da : mDynamicActivities)	{
		for (Movement* mv : da->movements())	{
			assert(mMapping != nullptr && "Mapping should be set!");
			const tuple<vector<double>, vector<double>>& ef = getValue(mMapping->mvToEnergyFunc, mv, caller());
			const vector<double>& t = get<0>(ef), & e = get<1>(ef);
			for (size_t s = 0; s+1 < min(t.size(), e.size()); ++s)	{
				double k = (e[s+1]-e[s])/(t[s+1]-t[s]), q = e[s]-k*t[s];
				double b = UB_W-q; Operator op = LESS_EQUAL;
				SparseMatrix<double>::Row row = {
					{getValue(d, da->aid(), caller()), k},
					{getValue(y, {da->aid(), mv->mid()}, caller()), UB_W}, {getValue(W, da->aid(), caller()), -1.0}
				};

				addConstraint(row, op, b, "(2$"+to_string(mConstraintsCounter++)+")");
			}
		}
	}
}

void ConstraintsGenerator::addEnergyFunctions(const map1to1& d, const map1to1& W, const map2to1& x, const map2to1& z, const map2to1& y)	{
	addEnergyFunctions1(d, W, x, z);
	addEnergyFunctions2(d, W, y);
}

// \sum_{\forall p \in P_i} x_i^p = 1 (3)
void ConstraintsGenerator::addUniquePointSelection(const map2to1& x)	{
	for (StaticActivity* sa : mStaticActivities)	{
		double b = 1.0; Operator op = EQUAL;
		SparseMatrix<double>::Row row;
		for (Location* l : sa->locations())
			row.emplace_back(getValue(x, {sa->aid(), l->point()}, caller()), 1.0);

		addConstraint(row, op, b, "(3$"+to_string(mConstraintsCounter++)+")");
	}
}

// \sum_{\forall m \in M_i} z_i^m = 1 (4)
void ConstraintsGenerator::addUniquePowerModeSelection(const map2to1& z)	{
	for (StaticActivity* sa : mStaticActivities)	{
		double b = 1.0; Operator op = EQUAL;
		SparseMatrix<double>::Row row;
		assert(sa->parent() != nullptr && "Each static activity had to have assigned a robot!");
		for (RobotPowerMode* pm : sa->parent()->powerModes())
			row.emplace_back(getValue(z, {sa->aid(), pm->pid()}, caller()), 1.0);

		addConstraint(row, op, b, "(4$"+to_string(mConstraintsCounter++)+")");
	}
}

void ConstraintsGenerator::addUniqueModeSelection(const map2to1& x, map2to1& z)	{
	addUniquePointSelection(x);
	addUniquePowerModeSelection(z);
}

/*
 * \sum_{\forall j \in PRED(i)} \sum_{\forall t \in T_j(p_{from},p)} y_j^t = x_i^p (5)
 * \sum_{\forall j \in SUC(i)} \sum_{\forall t \in T_j(p,p_{to})} y_j^t = x_i^p (6)
 */
void ConstraintsGenerator::addFlowConstraints(const map2to1& x, const map2to1& y)	{
	for (StaticActivity* sa : mStaticActivities)	{
		for (Location* l : sa->locations())	{
			double b = 0.0; Operator op = EQUAL;
			SparseMatrix<double>::Row row1 = {{getValue(x, {sa->aid(), l->point()}, caller()), -1.0}}, row2 = row1;
			for (Movement* mv : l->enteringMovements())
				row1.emplace_back(getValue(y, {mv->parent()->aid(), mv->mid()}, caller()), 1.0);
			for (Movement* mv : l->leavingMovements())
				row2.emplace_back(getValue(y, {mv->parent()->aid(), mv->mid()}, caller()), 1.0);

			addConstraint(row1, op, b, "(5$"+to_string(mConstraintsCounter++)+")");
			addConstraint(row2, op, b, "(6$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

// s_j-s_i = d_i - CT*h_{i,j} (7)
void ConstraintsGenerator::addFixedPrecedences(const map1to1& s, const map1to1& d)	{
	for (Robot* r : mRobots)	{
		for (Activity* i : r->activities())	{
			if (i->mandatory())	{
				for (Activity* j : i->successors())	{
					Operator op = EQUAL;
					double b = (i->lastInCycle() == false) ? 0.0 : -mCycleTime;
					SparseMatrix<double>::Row row = {
						{getValue(s, j->aid(), caller()), 1.0}, {getValue(s, i->aid(), caller()), -1.0}, {getValue(d, i->aid(), caller()), -1.0}
					};

					addConstraint(row, op, b, "(7$"+to_string(mConstraintsCounter++)+")");
				}
			}
		}
	}
}

// \sum_{\forall t \in T_i} y_i^t = w_i^{SUC(i)} (8)
void ConstraintsGenerator::addPrecedenceSelectionConstraints(const map2to1& y, const map2to1& w)	{
	for (DynamicActivity* da : mDynamicActivities)	{
		if (da->optional())	{
			SparseMatrix<double>::Row row;
			double b = 0.0; Operator op = EQUAL;
			row.emplace_back(getValue(w, {da->aid(), da->successor()->aid()}, caller()), -1.0);
			for (Movement* mv : da->movements())
				row.emplace_back(getValue(y, {da->aid(), mv->mid()}, caller()), 1.0);

			addConstraint(row, op, b, "(8$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

/*
 * s_j-s_i+(1-w_{i,j})*CT \geq d_i-CT*h_{i,j} (9)
 * s_j-s_i-(1-w_{i,j})*CT \leq d_i-CT*h_{i,j} (10)
 */
void ConstraintsGenerator::addSelectablePrecedences(const map1to1& s, const map1to1& d, const map2to1& w)	{
	for (DynamicActivity* i : mDynamicActivities)	{
		if (i->optional())	{
			for (Activity *j : i->successors())	{
				Operator op1 = GREATER_EQUAL, op2 = LESS_EQUAL;
				double b1 = (i->lastInCycle() == false) ?  -mCycleTime : -2.0*mCycleTime;
				double b2 = (i->lastInCycle() == false) ? mCycleTime : 0.0;
				SparseMatrix<double>::Row row = {
					{getValue(s, j->aid(), caller()), 1.0}, {getValue(s, i->aid(), caller()), -1.0}, {getValue(d, i->aid(), caller()), -1.0}
				}, row1 = row, row2 = row;
				row1.emplace_back(getValue(w, {i->aid(), j->aid()}, caller()), -mCycleTime);
				row2.emplace_back(getValue(w, {i->aid(), j->aid()}, caller()), mCycleTime);

				addConstraint(row1, op1, b1, "(9$"+to_string(mConstraintsCounter++)+")");
				addConstraint(row2, op2, b2, "(10$"+to_string(mConstraintsCounter++)+")");
			}
		}
	}
}

void ConstraintsGenerator::addAllPrecedences(const map1to1& s, const map1to1& d, const map2to1& y, const map2to1& w)	{
	addFixedPrecedences(s, d);
	addPrecedenceSelectionConstraints(y, w);
	addSelectablePrecedences(s, d, w);
}

// \underline{d_i^m}*z_i^m \leq d_i (11)
void ConstraintsGenerator::addMinimalDurationConstraints1(const map1to1& d, const map2to1& z)	{
	for (StaticActivity* sa : mStaticActivities)	{
		assert(sa->parent() != nullptr && "Each static activity had to have assigned a robot!");
		for (RobotPowerMode* pm : sa->parent()->powerModes())	{
			double b = 0.0; Operator op = LESS_EQUAL;
			SparseMatrix<double>::Row row = {
				{getValue(z, {sa->aid(), pm->pid()}, caller()), pm->minimalDelay()}, {getValue(d, sa->aid(), caller()), -1.0}
			};

			addConstraint(row, op, b, "(11$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

// \underline{d_i^t}*y_i^t \leq d_i (12)
void ConstraintsGenerator::addMinimalDurationConstraints2(const map1to1& d, const map2to1& y)	{
	for (DynamicActivity* da : mDynamicActivities)	{
		for (Movement* mv : da->movements())	{
			double b = 0.0; Operator op = LESS_EQUAL;
			SparseMatrix<double>::Row row = {
				{getValue(y, {da->aid(), mv->mid()}, caller()), mv->minDuration()}, {getValue(d, da->aid(), caller()), -1.0}
			};

			addConstraint(row, op, b, "(12$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

// d_i \leq \overline{d_i^t}+CT*(1-y_i^t) (13)
void ConstraintsGenerator::addMaximalDurationConstraints2(const map1to1& d, const map2to1& y)	{
	for (DynamicActivity* da : mDynamicActivities)	{
		for (Movement* mv : da->movements())	{
			Operator op = LESS_EQUAL;
			double b = mCycleTime+mv->maxDuration();
			SparseMatrix<double>::Row row = {
				{getValue(d, da->aid(), caller()), 1.0}, {getValue(y, {da->aid(), mv->mid()}, caller()), mCycleTime}
			};

			addConstraint(row, op, b, "(13$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

void ConstraintsGenerator::addDurationConstraints(const map1to1& d, const map2to1& z, const map2to1& y)	{
	addMinimalDurationConstraints1(d, z);
	addMinimalDurationConstraints2(d, y);
	addMaximalDurationConstraints2(d, y);
}

// s_j-s_i \geq l_{i,j}-CT*h_{i,j} (14)
void ConstraintsGenerator::addTimeLags(const map1to1& s, bool allRequired)	{
	assert(mRobots.size() > 1 && mLine != nullptr && "More than one robot expected!");
	for (InterRobotOperation* op : mLine->interRobotOperations())	{
		for (const TimeLag& l : op->timeLags())	{
			map1to1::const_iterator it1 = s.find(l.to()->aid()), it2 = s.find(l.from()->aid());
			if (it1 != s.cend() && it2 != s.cend())	{
				Operator op = GREATER_EQUAL;
				double b = l.length()-mCycleTime*l.height();
				assert(l.from() != nullptr && l.to() != nullptr && "Unexpected null pointers!");
				SparseMatrix<double>::Row row = {
					{it1->second, 1.0}, {it2->second, -1.0}
				};

				addConstraint(row, op, b, "(14$"+to_string(mConstraintsCounter++)+")");
			} else {
				if (allRequired)
					throw SolverException(caller(), "Robotic cell data structures are corrupted! Cannot find the time lag.");
			}
		}
	}
}

// x_i^p \leq \sum_{\forall p' \in CP(i,p)} x_j^p' (15)
void ConstraintsGenerator::addSpatialCompatibilityConstraints(const map2to1& x)	{
	assert(mRobots.size() > 1 && mLine != nullptr && "More than one robot expected!");
	for (InterRobotOperation* op : mLine->interRobotOperations())	{
		map<uint32_t, uint32_t> pointToId;
		map<uint32_t, vector<uint32_t> > pointToPoints;
		for (const pair<Location*, Location*>& p : op->spatialCompatibility())	{
			Location *l1 = p.first, *l2 = p.second;
			StaticActivity *sa1 = l1->parent(), *sa2 = l2->parent();
			pointToId[l1->point()] = sa1->aid(); pointToId[l2->point()] = sa2->aid();
			pointToPoints[l1->point()].push_back(l2->point()); pointToPoints[l2->point()].push_back(l1->point());
		}

		for (map<uint32_t, vector<uint32_t> >::const_iterator it = pointToPoints.cbegin(); it != pointToPoints.cend(); ++it)	{
			double b = 0.0; Operator op = LESS_EQUAL;
			SparseMatrix<double>::Row row;
		        row.emplace_back(getValue(x, {getValue(pointToId, it->first, caller()), it->first}, caller()), 1.0);
			for (const uint32_t& pt : it->second)
				row.emplace_back(getValue(x, {getValue(pointToId, pt, caller()), pt}, caller()), -1.0);

			addConstraint(row, op, b, "(15$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

/*
 * s_j + r CT + 2*|R| CT (3-c_{i,j}^{k1,k2,r}-m_i^k1-m_j^k2) \geq s_i + d_i (16)
 * s_i + 2*|R| CT (2+c_{i,j}^{k1,k2,r}-m_i^k1-m_j^k2) \geq s_j + d_j + r CT (17)
 */
void ConstraintsGenerator::addCollisions(const map1to1& s, const map1to1& d, const map2to1& x, const map2to1& y, const map4toN& c)	{
	assert(mRobots.size() > 1 && mLine != nullptr && "More than one robot expected!");
	for (const pair<ActivityMode*, ActivityMode*>& col : mLine->collisions())	{
		int32_t numberOfRobots = mRobots.size();
		for (int32_t r = -numberOfRobots; r <= numberOfRobots; ++r)	{
			Operator op = GREATER_EQUAL;
			SparseMatrix<double>::Row row;
			double constVal = 2.0*numberOfRobots*mCycleTime;
			double b1 = -3.0*constVal-r*mCycleTime, b2 = -2.0*constVal+r*mCycleTime;

			uint32_t a[2], m[2], i = 0;
			for (ActivityMode* mode : {col.first, col.second})	{
				Movement *mv = dynamic_cast<Movement*>(mode);
				if (mv != nullptr)	{
					DynamicActivity *da = mv->parent();
					a[i] = da->aid(); m[i] = mv->mid();
					row.emplace_back(getValue(y, {a[i], m[i]}, caller()), -constVal);
				}

				Location *loc = dynamic_cast<Location*>(mode);
				if (loc != nullptr)	{
					StaticActivity *sa = loc->parent();
					a[i] = sa->aid(); m[i] = loc->lid();
					row.emplace_back(getValue(x, {a[i], loc->point()}, caller()), -constVal);
				}

				++i;
			}

			SparseMatrix<double>::Row row1 = row, row2 = row;
			const vector<uint32_t>& v = getValue(c, {pack(a[0], m[0]), pack(a[1], m[1])}, caller());
			row1.insert(row1.end(), {{getValue(s, a[1], caller()), 1.0}, {getValue(s, a[0], caller()), -1.0}, {getValue(d, a[0], caller()), -1.0}, {v[r+numberOfRobots], -constVal}});
			row2.insert(row2.end(), {{getValue(s, a[0], caller()), 1.0}, {getValue(s, a[1], caller()), -1.0}, {getValue(d, a[1], caller()), -1.0}, {v[r+numberOfRobots], constVal}});

			addConstraint(row1, op, b1, "(16$"+to_string(mConstraintsCounter++)+")");
			addConstraint(row2, op, b2, "(17$"+to_string(mConstraintsCounter++)+")");
		}
	}
}

void ConstraintsGenerator::addGlobalConstraints(const map1to1& s, const map1to1& d, const map2to1& x, const map2to1& y, const map4toN& c)	{
	addTimeLags(s);
	addSpatialCompatibilityConstraints(x);
	addCollisions(s, d, x, y, c);
}

// a_{i,p}^m*d_i \leq W_i (1)
void ConstraintsGenerator::addEnergyFunction(const map1to1& d, const map1to1& W, Location *loc, RobotPowerMode *pwrm)	{
	assert(loc->parent() != nullptr && "Each location must be a part of a static activity!");
	StaticActivity *sa = loc->parent();

	double b = 0; Operator op = LESS_EQUAL;
	SparseMatrix<double>::Row row = {
		{getValue(d, sa->aid(), caller()), loc->inputPower(pwrm)}, {getValue(W, sa->aid(), caller()), -1.0}
	};

	addConstraint(row, op, b, "(1@"+to_string(mConstraintsCounter++)+")");
}

// a_{i,k}^{t}*d_i+b_{i,k}^t \leq W_i (2)
void ConstraintsGenerator::addEnergyFunction(const map1to1& d, const map1to1& W, Movement* mv)	{
	assert(mMapping != nullptr && mv->parent() != nullptr && "It should be initialized!");

	uint32_t aid = mv->parent()->aid();
	const auto& efit = mMapping->mvToEnergyFunc.find(mv);

	if (efit != mMapping->mvToEnergyFunc.cend())	{

		const tuple<vector<double>, vector<double>>& ef = efit->second;

		const vector<double>& t = get<0>(ef), & e = get<1>(ef);
		for (size_t s = 0; s+1 < min(t.size(), e.size()); ++s)	{
			double k = (e[s+1]-e[s])/(t[s+1]-t[s]), q = e[s]-k*t[s];
			double b = -q; Operator op = LESS_EQUAL;

			const auto& dit = d.find(aid), &Wit = W.find(aid);
			if (dit != d.cend() && Wit != W.cend())	{
				SparseMatrix<double>::Row row = {{dit->second, k}, {Wit->second, -1.0}};
				addConstraint(row, op, b, "(2@"+to_string(mConstraintsCounter++)+")");
			} else {
				throw SolverException(caller(), "Invalid mapping of variables d or W!");
			}
		}

	} else	{
		throw SolverException(caller(), "Cannot find the energy function for movement "+to_string(mv->mid())+"!");
	}
}

// s_j-s_i = d_i - CT*h_{i,j} (7)
void ConstraintsGenerator::addSelectedPrecedences(const map1to1& s, const map1to1& d, const std::vector<Movement*>& mvs)	{
	for (Movement* mv : mvs)	{
		assert(mv->parent() != nullptr && "Movement should be a part of a dynamic activity!");
		DynamicActivity *da = mv->parent();
		StaticActivity* from = da->predecessor(), *to = da->successor();
		vector<pair<Activity*, Activity*>> prec = {{from, da}, {da, to}};
		for (const pair<Activity*, Activity*>& p : prec)	{
			Operator op = EQUAL;
			double b = (p.first->lastInCycle() == false) ? 0.0 : -mCycleTime;

			const auto &sit1 = s.find(p.second->aid()), &sit2 = s.find(p.first->aid()), &dit = d.find(p.first->aid());
			if (sit1 != s.cend() && sit2 != s.cend() && dit != d.cend())	{
				uint32_t idx1 = sit1->second, idx2 = sit2->second, idx3 = dit->second;
				SparseMatrix<double>::Row row = {
					{idx1, 1.0}, {idx2, -1.0}, {idx3, -1.0}
				};

				addConstraint(row, op, b, "(7@"+to_string(mConstraintsCounter++)+")");
			} else {
				throw SolverException(caller(), "Invalid variable mapping!");
			}
		}
	}
}

// s_j \geq s_i+d_i+CT*K (16|17)
void ConstraintsGenerator::addCollisionResolution(const map1to1& s, const map1to1& d, uint32_t i, uint32_t j, int32_t multipleOfCycleTime)	{
	Operator op = GREATER_EQUAL;
	double b = multipleOfCycleTime*mCycleTime;
	SparseMatrix<double>::Row row = {
		{getValue(s, j, caller()), 1.0}, {getValue(s, i, caller()), -1.0}, {getValue(d, i, caller()), -1.0}
	};

	addConstraint(row, op, b, "(16|17@"+to_string(mConstraintsCounter++)+")");
}

void ConstraintsGenerator::initializeDataStructures()	{
	reset(); mStaticActivities.clear(); mDynamicActivities.clear();
	assert(!mRobots.empty() && "At least one robot must be added by the reset method!");
	assert(mRobots[0]->parent() != nullptr && "Even one robot must be encapsulated by the RoboticLine class!");
	mCycleTime = (mRobots[0]->parent())->productionCycleTime();
	for (Robot* r : mRobots)	{
		for (Activity* a : r->activities())	{
			StaticActivity* sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr)
				mStaticActivities.push_back(sa);

			DynamicActivity* da = dynamic_cast<DynamicActivity*>(a);
			if (da != nullptr)
				mDynamicActivities.push_back(da);
		}
	}
}

void ConstraintsGenerator::addConstraint(SparseMatrix<double>::Row& row, Operator op, const double& b, string conDesc)	{
	this->mA.addRow(row); this->mOp.push_back(op); this->mB.push_back(b);
	this->mConDesc.push_back(conDesc);
}


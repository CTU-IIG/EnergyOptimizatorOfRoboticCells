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

#include <tuple>
#include <string>
#include <stdexcept>
#include <vector>
#include "gurobi_c++.h"
#include "ILPModel/SolverInterface.h"

using namespace std;

vector<GRBEnv> threadEnv;

vector<GRBVar> generateProblem(const ILPModel& m, GRBModel& model);

string solverIdentification()	{
	string identification = "Gurobi";
	int major = -1, minor = -1, release = -1;
	GRBversion(&major, &minor, &release);
	if (major != -1 && minor != -1 && release != -1)
		identification += " "+to_string(major)+"."+to_string(minor)+"."+to_string(release);
	return identification;
}

void initializeLocalEnvironments(int numberOfThreads)	{
	threadEnv.resize(numberOfThreads);
}

SolutionILP solveILP(const ILPModel& m, bool verbose, double gap, double timeLimit, int numberOfThreads, int threadId)	{

	SolutionILP s;

	try {
		if (threadEnv.size() <= (uint32_t) threadId)	{
			string msg = "Either the Gurobi environments were not initialized or the number of concurrent threads\n";
			msg += "was specified incorrectly in the initializeLocalEnvironments method!";
			throw ILPSolverException(caller(), msg);
		}

		threadEnv[threadId].set(GRB_IntParam_Threads, numberOfThreads);
		if (timeLimit != 0.0)
			threadEnv[threadId].set(GRB_DoubleParam_TimeLimit, timeLimit);
		if (gap != 0.0)
			threadEnv[threadId].set(GRB_DoubleParam_MIPGap, gap);

		if (verbose)	{
			threadEnv[threadId].set(GRB_IntParam_OutputFlag, 1);
			threadEnv[threadId].set(GRB_IntParam_LogToConsole, 1);
		} else {
			threadEnv[threadId].set(GRB_IntParam_OutputFlag, 0);
			threadEnv[threadId].set(GRB_IntParam_LogToConsole, 0);
		}

		GRBModel model(threadEnv[threadId]);
		const vector<GRBVar>& vars = generateProblem(m, model);

		model.optimize();
		switch (model.get(GRB_IntAttr_Status))	{
			case GRB_OPTIMAL:
				if (gap == 0.0)
					s.status = ILP_OPTIMAL;
				else
					s.status = ILP_FEASIBLE;
				break;
			case GRB_TIME_LIMIT:
				if (model.get(GRB_IntAttr_SolCount) > 0)
					s.status = ILP_FEASIBLE;
				else
					s.status = ILP_UNKNOWN;
				break;
			case GRB_INFEASIBLE:
				s.status = ILP_INFEASIBLE;
				break;
			case GRB_UNBOUNDED:
				s.status = ILP_UNBOUNDED;
				break;
			default:
				s.status = ILP_UNKNOWN;
		}

		if (s.status == ILP_OPTIMAL || s.status == ILP_FEASIBLE)	{
			s.criterion = model.get(GRB_DoubleAttr_ObjVal);
			for (long v = 0; v < model.get(GRB_IntAttr_NumVars); ++v)
				s.solution.push_back(vars[v].get(GRB_DoubleAttr_X));
		}

		if (model.get(GRB_IntAttr_IsMIP))
			s.bound = model.get(GRB_DoubleAttr_ObjBound);
		else if (s.status == ILP_OPTIMAL)
			s.bound = s.criterion;

	} catch (GRBException& e)	{
		throw ILPSolverException(caller(), e.getMessage());
	} catch (...) {
		throw_with_nested(ILPSolverException(caller(), "Error during solving the ILP problem!"));
	}

	return s;
}

vector<GRBVar> generateProblem(const ILPModel& m, GRBModel& model)	{
	vector<GRBVar> vars;
	model.set(GRB_IntAttr_ModelSense, (m.obj == MINIMIZE ? GRB_MINIMIZE : GRB_MAXIMIZE));
	for (unsigned long v = 0; v < m.numberOfVariables(); ++v)	{
		GRBVar var;
		switch (m.x[v].type)	{
			case FLT:
				var = model.addVar(m.x[v].lowerBound, m.x[v].upperBound, m.c[v], GRB_CONTINUOUS, m.varDesc[v]);
				break;
			case BIN:
				var = model.addVar(m.x[v].lowerBound, m.x[v].upperBound, m.c[v], GRB_BINARY, m.varDesc[v]);
				break;
			default:
				var = model.addVar(m.x[v].lowerBound, m.x[v].upperBound, m.c[v], GRB_INTEGER, m.varDesc[v]);
		}
		vars.push_back(var);
	}

	model.update();

	if (!m.gurobiC.empty())	{
		assert(m.numberOfVariables() == m.gurobiC.size() && "Invalid initialization of gurobi criterion functions!");
		for (unsigned long v = 0; v < m.numberOfVariables(); ++v)	{
			const tuple<vector<double>, vector<double>>* cf = m.gurobiC[v];
			if (cf != nullptr)	{
				const vector<double>& x = get<0>(*cf), & y = get<1>(*cf);
				assert(x.size() == y.size() && "Invalid initialization of Gurobi functions!");
				if (!x.empty() && !y.empty())	{
					// Add a discretized function, a ,,dirty" casting is due to non-const Gurobi call.
					model.setPWLObj(vars[v], x.size(), (double*) x.data(), (double*) y.data());
				}
			}
		}
	}

	model.update();

	for (unsigned long c = 0; c < m.numberOfConstraints(); ++c)	{
		GRBLinExpr expr;
		for (const pair<uint32_t, double>& p : m.A[c])
			expr += p.second*vars[p.first];

		switch (m.ops[c])	{
			case LESS_EQUAL:
				model.addConstr(expr <= m.b[c], m.conDesc[c]);
				break;
			case EQUAL:
				model.addConstr(expr == m.b[c], m.conDesc[c]);
				break;
			case GREATER_EQUAL:
				model.addConstr(expr >= m.b[c], m.conDesc[c]);
		}
	}

	return vars;
}


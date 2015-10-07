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

#include <string>
#include <stdexcept>
#include <vector>
#include <ilcplex/ilocplex.h>
#include "ILPModel/SolverInterface.h"

using namespace std;

ILOSTLBEGIN

void generateProblem(const ILPModel& m, IloModel& model, IloNumVarArray& x, IloRangeArray& con);

string solverIdentification()	{
	IloEnv env;
	IloCplex cplex(env);
	string identification = "Cplex "+string(cplex.getVersion());
	env.end();

        return identification;
}

SolutionILP solveILP(const ILPModel& m, bool verbose, double gap, double timeLimit, int numberOfThreads, int)	{

	IloEnv env;
	SolutionILP solution;

	try {
		IloModel model(env);
		IloObjective obj(env);
		IloNumVarArray var(env);
		IloRangeArray con(env);
		generateProblem(m, model, var, con);

		IloCplex cplex(model);
		cplex.setParam(IloCplex::Threads, numberOfThreads);
		if (!verbose)
			cplex.setOut(env.getNullStream());
		if (gap != 0.0)
			cplex.setParam(IloCplex::EpGap, gap);
		if (timeLimit != 0.0)
			cplex.setParam(IloCplex::TiLim, timeLimit);

		cplex.solve();
		switch (cplex.getStatus())	{
			case  IloAlgorithm::Optimal:
				solution.status = ILP_OPTIMAL;
				break;
			case IloAlgorithm::Feasible:
				solution.status = ILP_FEASIBLE;
				break;
			case IloAlgorithm::Infeasible:
				solution.status = ILP_INFEASIBLE;
				break;
			case IloAlgorithm::Unbounded:
				solution.status = ILP_UNBOUNDED;
				break;
			default:
				solution.status = ILP_UNKNOWN;
		}

		if (solution.status == ILP_OPTIMAL || solution.status == ILP_FEASIBLE)	{
			IloNumArray sol(env);
			cplex.getValues(sol, var);
			solution.criterion = cplex.getObjValue();
			for (long v = 0; v < sol.getSize(); ++v)
				solution.solution.push_back(sol[v]);
		}

		solution.bound = cplex.getBestObjValue();	// Can throw IloException!
		env.end();

	} catch (IloException& e)	{
		env.end();
		throw ILPSolverException(caller(), e.getMessage());
	} catch (...) {
		env.end();
		throw_with_nested(ILPSolverException(caller(), "Error during solving the ILP problem!"));
	}

	return solution;
}

void generateProblem(const ILPModel& m, IloModel& model, IloNumVarArray& x, IloRangeArray& con)	{
	IloEnv env = model.getEnv();
	IloObjective obj = (m.obj == MINIMIZE ? IloMinimize(env) : IloMaximize(env));
	for (unsigned long v = 0; v < m.numberOfVariables(); ++v)	{
		switch (m.x[v].type)	{
			case FLT:
				x.add(IloNumVar(env, m.x[v].lowerBound, m.x[v].upperBound, IloNumVar::Float));
				break;
			case BIN:
				x.add(IloNumVar(env, m.x[v].lowerBound, m.x[v].upperBound, IloNumVar::Bool));
				break;
			default:
				x.add(IloNumVar(env, m.x[v].lowerBound, m.x[v].upperBound, IloNumVar::Int));
		}

		obj.setLinearCoef(x[v], m.c[v]);
		x[v].setName(m.varDesc[v].c_str());
	}

	for (unsigned long c = 0; c < m.numberOfConstraints(); ++c)	{
		switch (m.ops[c])	{
			case LESS_EQUAL:
				con.add(IloRange(env, -IloInfinity, m.b[c]));
				break;
			case EQUAL:
				con.add(IloRange(env, m.b[c], m.b[c]));
				break;
			case GREATER_EQUAL:
				con.add(IloRange(env, m.b[c], IloInfinity));
		}

		for (const pair<uint32_t, double>& p : m.A[c])
			con[c].setLinearCoef(x[p.first], p.second);

		con[c].setName(m.conDesc[c].c_str());
	}

	model.add(obj);
	model.add(con);
}


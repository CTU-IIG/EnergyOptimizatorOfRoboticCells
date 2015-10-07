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
#include <string>
#include <stdexcept>
#include <vector>
#include <lpsolve/lp_lib.h>
#include "ILPModel/SolverInterface.h"

using namespace std;

void throwExceptionIfError(const unsigned char& status, string errorMsg);
void generateProblem(const ILPModel& m, lprec* ilp);

string solverIdentification()	{
	string identification = "LPSolve";
	int major = -1, minor = -1, release = -1, build = -1;
	lp_solve_version(&major, &minor, &release, &build);
	if (major != -1 && minor != -1 && release != -1 && build != -1)
		identification += " "+to_string(major)+"."+to_string(minor)+"."+to_string(release)+"."+to_string(build);
	return identification;
}

SolutionILP solveILP(const ILPModel& m, bool verbose, double gap, double timeLimit, int, int)	{

	SolutionILP s;
	lprec *model = nullptr;

	try {
		model = make_lp(m.numberOfConstraints(), m.numberOfVariables());
		if (model == nullptr)
			throw ILPSolverException(caller(), "LPSolve - cannot initialize the ilp problem!");

		generateProblem(m, model);

		if (!verbose)
			set_verbose(model, NEUTRAL);
		if (gap != 0.0)
			set_mip_gap(model, FALSE, gap);
		if (timeLimit != 0.0)
			set_timeout(model, static_cast<long>(ceil(timeLimit)));

		set_presolve(model, PRESOLVE_ROWS | PRESOLVE_BOUNDS, get_presolveloops(model));
		switch (solve(model))	{
			case OPTIMAL:
				s.status = ILP_OPTIMAL;
				break;
			case SUBOPTIMAL:
				s.status = ILP_FEASIBLE;
				break;
			case INFEASIBLE:
				s.status = ILP_INFEASIBLE;
				break;
			case UNBOUNDED:
				s.status = ILP_UNBOUNDED;
				break;
			default:
				s.status = ILP_UNKNOWN;
		}

		if (s.status == ILP_OPTIMAL || s.status == ILP_FEASIBLE)	{
			double *vars = nullptr;
			get_ptr_variables(model, &vars);
			if (vars != nullptr)	{
				s.criterion = get_objective(model);
				for (unsigned long v = 0; v < m.numberOfVariables(); ++v)
					s.solution.push_back(vars[v]);
			} else {
				throw ILPSolverException(caller(), "LPSolve - cannot obtain the variables!");
			}

		}

		if (s.status == ILP_OPTIMAL)	{
			s.bound = s.criterion;
		} else {
			s.bound = 0.0;
			for (unsigned long v = 0; v < m.numberOfVariables(); ++v)	{
				if (m.obj == MINIMIZE && m.c[v] > 0.0)
					s.bound += m.c[v]*m.x[v].lowerBound;
				else if (m.obj == MINIMIZE && m.c[v] < 0.0)
					s.bound += m.c[v]*m.x[v].upperBound;
				else if (m.obj == MAXIMIZE && m.c[v] > 0.0)
					s.bound += m.c[v]*m.x[v].upperBound;
				else if (m.obj == MAXIMIZE && m.c[v] < 0.0)
					s.bound += m.c[v]*m.x[v].lowerBound;
			}
		}

		delete_lp(model);

	} catch (...) {
		delete_lp(model);
		throw_with_nested(ILPSolverException(caller(), "Error during solving the ILP problem!"));
	}

	return s;
}

void throwExceptionIfError(const unsigned char& status, string errorMsg)	{
	if (status != TRUE)
		throw ILPSolverException(caller(), errorMsg);
}

void generateProblem(const ILPModel& m, lprec* ilp)	{

	if (m.obj == MINIMIZE)
		set_minim(ilp);
	else
		set_maxim(ilp);

	vector<double> crit(1, 0.0);
	crit.insert(crit.end(), m.c.cbegin(), m.c.cend());
	throwExceptionIfError(set_obj_fn(ilp, crit.data()), "LPSolve - cannot set the objective function!");

	for (unsigned long v = 0; v < m.numberOfVariables(); ++v)	{
		if (m.x[v].type == BIN || m.x[v].type == INT)
			throwExceptionIfError(set_int(ilp, v+1, TRUE), "LPSolve - cannot set the integer type of the variable!");

		throwExceptionIfError(set_bounds(ilp, v+1, m.x[v].lowerBound, m.x[v].upperBound), "LPSolve - cannot set variable bounds!");
		throwExceptionIfError(set_col_name(ilp, v+1, (char*) m.varDesc[v].c_str()), "LPSolve - cannot set the variable name!");
	}

	throwExceptionIfError(set_add_rowmode(ilp, TRUE), "LPSolve - 'rowmode' should be changed!");
	for (unsigned long c = 0; c < m.numberOfConstraints(); ++c)	{
		vector<int> conIdx;
		vector<double> conCoeff;
		for (const pair<uint32_t, double>& p : m.A[c])	{
			conIdx.push_back(p.first+1);
			conCoeff.push_back(p.second);
		}

		switch (m.ops[c])	{
			case LESS_EQUAL:
				throwExceptionIfError(add_constraintex(ilp, conIdx.size(), conCoeff.data(), conIdx.data(), LE, m.b[c]), "LPSolve - cannot add the constraint!");
				break;
			case EQUAL:
				throwExceptionIfError(add_constraintex(ilp, conIdx.size(), conCoeff.data(), conIdx.data(), EQ, m.b[c]), "LPSolve - cannot add the constraint!");
				break;
			case GREATER_EQUAL:
				throwExceptionIfError(add_constraintex(ilp, conIdx.size(), conCoeff.data(), conIdx.data(), GE, m.b[c]), "LPSolve - cannot add the constraint!");
		}

		throwExceptionIfError(set_row_name(ilp, c+1, (char*) m.conDesc[c].c_str()), "LPSolve - cannot set the constraint name!");
	}
	throwExceptionIfError(set_add_rowmode(ilp, FALSE), "LPSolve - 'rowmode' should be turned off!");
}


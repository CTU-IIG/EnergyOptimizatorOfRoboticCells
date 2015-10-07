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

#ifndef HLIDAC_PES_ILP_MODEL_H
#define HLIDAC_PES_ILP_MODEL_H

/*!
 * \file ILPModel.h
 * \author Libor Bukata
 * \brief General model for Integer Linear Programming problem.
 */

#include <string>
#include <vector>
#include "SparseMatrix.h"
#include "Shared/NumericConstants.h"

//! Specifies the sense of the objective function.
enum Objective {
	MINIMIZE, MAXIMIZE
};

//! Constants for floats, binary variables, and integers, respectively.
enum VariableType {
	FLT, BIN, INT
};

//! Constants for operators '<=', '=', and '>=', respectively.
enum Operator {
	LESS_EQUAL, EQUAL, GREATER_EQUAL
};

//! Structure containing information about the variable.
struct Variable {
	Variable(VariableType t = FLT, double lb = F64_MIN, double ub = F64_MAX) : type(t), lowerBound(lb), upperBound(ub) { }

	//! Type of the variable, see \ref VariableType.
	VariableType type;
	//! The minimal value of the variable.
	double lowerBound;
	//! The maximal value of the variable.
	double upperBound;
};

/*!
 * Data structure corresponding to the Integer Linear Programming problem in the form:<br>
 * \f$
 * \begin{alignat}{1}
 * 	& \text{min}/\text{max} \; c'x \notag \\
 *      & \qquad Ax \; \mathrm{op} \; b \notag \\
 *      & \qquad x_1 \in \mathcal{R},\; x_2 \in \mathbb{B},\; x_3 \in \mathbb{Z} \notag \\
 *      & \qquad x = x_1 \cup x_2 \cup x_3 \notag
 * \end{alignat}
 * \f$
 * \brief Integer Linear Programming problem is stored in this data structure.
 */
struct ILPModel	{
	ILPModel(Objective obj = MINIMIZE) : obj(obj) { }
	size_t numberOfConstraints() const { return A.numberOfRows(); }
	size_t numberOfVariables() const { return x.size(); }
	//! It checks that sizes of vectors and the matrix are valid.
	void checkFormulation() const;
	//! It prints various statistics like density of A matrix and number of constraints to the standard output.
	void printStatistics() const;

	//! Sense of the objective function (minimization/maximization).
	Objective obj;
	//! Constraint matrix of the problem.
	SparseMatrix<double> A;
	//! Vector of the criterion coefficients.
	std::vector<double> c;
	//! Convex functions in the criterion in the format suitable for Gurobi solver.
	std::vector<const std::tuple<std::vector<double>, std::vector<double>>*> gurobiC;
	//! Constants in the constraints, i.e. the right-hand side vector of \f$Ax \; \mathrm{op} \; b\f$.
	std::vector<double> b;
	//! Operators of the constraints, see \ref Operator enum.
	std::vector<Operator> ops;
	//! Variables of the problem.
	std::vector<Variable> x;
	//! Optional description of the variables.
	std::vector<std::string> varDesc;
	//! Optional description of the constraints.
	std::vector<std::string> conDesc;
};

#endif

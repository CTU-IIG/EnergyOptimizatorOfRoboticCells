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
#include <iostream>
#include <iterator>
#include "SolverConfig.h"
#include "Shared/Exceptions.h"
#include "ILPModel/ILPModel.h"

using namespace std;

void ILPModel::checkFormulation() const	{
	uint64_t numberOfVariables = A.numberOfColumns(), numberOfConstraints = A.numberOfRows();
	if (numberOfVariables == 0 || numberOfConstraints == 0)
		throw SolverException(caller(), "Empty matrix 'A' of the ILP formulation!");

	uint32_t cSize = max(c.size(), gurobiC.size());
	if (numberOfVariables != x.size() || numberOfVariables != cSize)
		throw SolverException(caller(), "Invalid horizontal size of 'A', 'c', 'x'!");

	if (numberOfConstraints != ops.size() || numberOfConstraints != b.size())
		throw SolverException(caller(), "Invalid vertical size of 'A', 'ops', 'b'!");

	if (!varDesc.empty() && varDesc.size() != numberOfVariables)
		throw SolverException(caller(), "Incomplete description of variables!");

	if (!conDesc.empty() && conDesc.size() != numberOfConstraints)
		throw SolverException(caller(), "Incomplete description of constraints!");
}

void ILPModel::printStatistics() const	{
	try {
		checkFormulation();
		double densityOfMatrixA = A.densityOfMatrix();
		uint64_t numberOfBinaryVariables = 0, numberOfFloatVariables = 0, numberOfIntegerVariables = 0;
		for (uint64_t v = 0; v < numberOfVariables(); ++v)	{
			switch (x[v].type)	{
				case FLT:
					++numberOfFloatVariables;
					break;
				case BIN:
					++numberOfBinaryVariables;
					break;
				case INT:
					++numberOfIntegerVariables;
					break;
				default:
					throw SolverException(caller(), "Unknown type of variable!");
			}
		}

		cout<<"Number of variables: "<<numberOfVariables()<<endl;
		cout<<"Number of float variables: "<<numberOfFloatVariables<<endl;
		cout<<"Number of binary variables: "<<numberOfBinaryVariables<<endl;
		cout<<"Number of integer variables: "<<numberOfIntegerVariables<<endl;
		cout<<"Number of constraints: "<<numberOfConstraints()<<endl;
		cout<<"Density of matrix A: "<<densityOfMatrixA*100.0<<" %"<<endl;

	} catch (...)	{
		throw_with_nested(InvalidArgument(caller(), "Invalid dimensions of ILP formulation!"));
	}
}


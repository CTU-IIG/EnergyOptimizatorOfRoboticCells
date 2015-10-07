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

#ifndef HLIDAC_PES_ALGORITHMS_H
#define HLIDAC_PES_ALGORITHMS_H

/*!
 * \file Algorithms.h
 * \author Libor Bukata
 * \brief Universal algorithms like Floyd-Warshall, Golden Search, etc.
 */

#include <algorithm>
#include <cmath>
#include <cassert>
#include <functional>
#include <vector>
#include <utility>

/*!
 * \tparam T A numerical type.
 * \brief Definition of the matrix data type.
 */
template <class T>
using DistanceMatrix = std::vector<std::vector<T>>;

/*!
 * \param m Distance matrix of the graph without negative cycles.
 * \param cmp Operator of the comparison, typically '>'.
 * \tparam T A numerical type.
 * \tparam C A comparison function of distances.
 * \return Updated distance matrix containing the lengths of all-to-all shortest paths.
 * \brief It calculates all-to-all shortest paths and returns the matrix with their lengths.
 */
template <class T, class C = std::greater<T>>
DistanceMatrix<T> floyd(DistanceMatrix<T> m, const C& cmp = std::greater<T>())	{
	size_t matrixSize = m.size();
	for (size_t k = 0; k < matrixSize; ++k)	{
		for (size_t i = 0; i < matrixSize; ++i)	{
			for (size_t j = 0; j < matrixSize; ++j)	{
				assert(i < m.size() && j < m[i].size() && k < m[i].size() && k < m.size() && j < m[k].size() && "Floyd needs square matrix!");
				if (cmp(m[i][j], m[i][k]+m[k][j]))
					m[i][j] = m[i][k]+m[k][j];
			}
		}
	}

	return m;
}

/*!
 * \param unimodalFce A convex unimodal function depending on one variable.
 * \tparam T A class encapsulating the convex function, T::tolerance(), T::functionValue() methods have to be defined.
 * \return The optimal value of the variable and the related minimal function value.
 * \brief It finds the minimal function value of a unimodal convex function.
 */
template <class T>
std::pair<double, double> goldenSearch(const T& unimodalFce)	{
	const double tol = unimodalFce.tolerance();
	const static double phi = (std::sqrt(5.0)-1.0)/2.0;

	double a = 0.0, b = 1.0, c = 1.0-phi, d;
	double fxC = unimodalFce.functionValue(c), fxD;
	while (b-a > tol)	{
		if (c-a < b-c)	{
			d = phi*(b-a)+a;
			fxD = unimodalFce.functionValue(d);
			if (fxC <= fxD)	{
				b = d;
			} else {
				a = c; c = d;
				fxC = fxD;
			}
		} else {
			d = (1-phi)*(b-a)+a;
			fxD = unimodalFce.functionValue(d);
			if (fxD <= fxC)	{
				b = c; c = d;
				fxC = fxD;
			} else {
				a = d;
			}
		}
	}

	return { c, fxC };
}

#endif

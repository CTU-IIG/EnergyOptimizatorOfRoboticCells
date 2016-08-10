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

#ifndef HLIDAC_PES_NUMERIC_CONSTANTS_H
#define HLIDAC_PES_NUMERIC_CONSTANTS_H

/*!
 * \file NumericConstants.h
 * \author Libor Bukata
 * \brief The file defines allowed inaccuracies in a solution and constants for floats.
 */

#include <cmath>
#include <limits>

//! A minimal time difference that is considered significant for a solution.
constexpr double TIME_TOL = 0.01;
//! A maximal relative tolerance of the criterion error imposed by the piece-wise linearization of energy functions.
constexpr double CRITERION_RTOL = 0.005;
//! Minimal recognizable difference in the time.
constexpr double TIME_ERR = 0.001*TIME_TOL;

constexpr float F32_INF = std::numeric_limits<float>::infinity();
constexpr float F32_MIN = std::numeric_limits<float>::lowest();
constexpr float F32_MAX = std::numeric_limits<float>::max();
constexpr float F32_EPS = std::numeric_limits<float>::epsilon();

constexpr double F64_INF = std::numeric_limits<double>::infinity();
constexpr double F64_MIN = std::numeric_limits<double>::lowest();
constexpr double F64_MAX = std::numeric_limits<double>::max();
constexpr double F64_EPS = std::numeric_limits<double>::epsilon();

#endif

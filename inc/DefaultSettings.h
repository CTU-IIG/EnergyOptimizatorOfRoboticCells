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

#ifndef HLIDAC_PES_DEFAULT_SETTINGS_H
#define HLIDAC_PES_DEFAULT_SETTINGS_H

/*!
 * \file DefaultSettings.h
 * \author Libor Bukata
 * \brief Default configuration of the solver.
 */

/* GENERAL OPTIONS */

//! Dataset to be loaded if not specified.
#define DEFAULT_DATASET_FILE ""
//! Verbosity is not turn on by default.
#define DEFAULT_VERBOSE false
//! The number of linear functions that approximate each energy function of the movement.
#define DEFAULT_NUMBER_OF_SEGMENTS 10
//! Default time limit.
#define DEFAULT_MAX_RUNTIME 30.0
//! It specifies whether the heuristic algorithm should be preferred by default.
#define DEFAULT_USE_HEURISTICS true
//! It specifies whether the exact algorithm should be preferred by default.
#define DEFAULT_USE_EXACT_ALGORITHM false
//! Path to a directory where the results could be written.
#define DEFAULT_RESULTS_DIRECTORY ""


/* ILP SOLVER OPTIONS */

//! ILP solver stops if a current best solution is not worse than gap*100 % from optimality.
#define DEFAULT_ILP_RELATIVE_GAP 0.0
//! It specifies whether a tight lower bound should be calculated.
#define DEFAULT_CALCULATE_LOWER_BOUND false
//! Upper bound on time limit for the tight lower bound.
#define DEFAULT_RUNTIME_OF_LOWER_BOUND 15.0


/* HEURISTIC OPTIONS */

//! Default number of elite solutions stored by the heuristic.
#define DEFAULT_MAX_ELITE_SOLUTIONS 20
//! It bounds the maximal number of generated alternatives for each robot, only some of them are selected.
#define DEFAULT_MAX_ALTERNATIVES 1000
//! The minimal number of optimization iterations per partial solution in the heuristic.
#define DEFAULT_MIN_ITERS_PER_TUPLE 100

#endif


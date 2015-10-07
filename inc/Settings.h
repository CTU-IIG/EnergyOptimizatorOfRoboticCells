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

#ifndef HLIDAC_PES_SETTINGS_H
#define HLIDAC_PES_SETTINGS_H

/*!
 * \file Settings.h
 * \author Libor Bukata
 * \brief It declares the namespace for program settings.
 */

#include <string>
#include <stdint.h>
#include "DefaultSettings.h"

//! This namespace encapsulates various external variables related to the program settings.
namespace Settings {

	/* GENERAL OPTIONS */

	//! Dataset with problems to be solved.
	extern std::string DATASET_FILE;
	//! Boolean flag determining verbosity of the program.
	extern bool VERBOSE;
	//! By how many segments (linear pieces) the energy function of the movement is approximated.
	extern uint32_t NUMBER_OF_SEGMENTS;
	//! Maximal number of threads to be used.
	extern uint32_t NUMBER_OF_THREADS;
	//! Maximal run time of the solver.
	extern double MAX_RUNTIME;
	//! The variable indicates whether the heuristic should be used.
	extern bool USE_HEURISTICS;
	//! The variable indicates whether the exact algorithm should be used.
	extern bool USE_EXACT_ALGORITHM;
	//! If not empty, then the optimization results will be written to this directory.
	extern std::string RESULTS_DIRECTORY;


	/* ILP SOLVER OPTIONS */

	//! If a given relative gap from the best known lower bound is achieved, then the solver stops.
	extern double ILP_RELATIVE_GAP;
	//! Indicates whether a tight lower bound should be calculated.
	extern bool CALCULATE_LOWER_BOUND;
	//! Time limit for the tight lower bound.
	extern double RUNTIME_OF_LOWER_BOUND;


	/* HEURISTIC OPTIONS */

	//! The number of top solutions maintained by the heuristic.
	extern uint32_t MAX_ELITE_SOLUTIONS;
	//! The maximal number of alternative orders generated for each robot.
	extern uint32_t MAX_ALTERNATIVES;
	//! The minimal number of optimization iterations per each tuple.
	extern uint32_t MIN_ITERS_PER_TUPLE;
}

#endif


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
#include <thread>
#include "Settings.h"
#include "DefaultSettings.h"

using namespace std;

namespace Settings {
	/* GENERAL OPTIONS */
	string DATASET_FILE = DEFAULT_DATASET_FILE;
	bool VERBOSE = DEFAULT_VERBOSE;
	uint32_t NUMBER_OF_SEGMENTS = DEFAULT_NUMBER_OF_SEGMENTS;
	uint32_t NUMBER_OF_THREADS = max(1u, thread::hardware_concurrency());
	double MAX_RUNTIME = DEFAULT_MAX_RUNTIME;
	bool USE_HEURISTICS = false;
	bool USE_EXACT_ALGORITHM = false;
	string RESULTS_DIRECTORY = DEFAULT_RESULTS_DIRECTORY;

	/* ILP SOLVER OPTIONS */
	double ILP_RELATIVE_GAP = DEFAULT_ILP_RELATIVE_GAP;
	bool CALCULATE_LOWER_BOUND = DEFAULT_CALCULATE_LOWER_BOUND;
	double RUNTIME_OF_LOWER_BOUND = DEFAULT_RUNTIME_OF_LOWER_BOUND;

	/* HEURISTIC OPTIONS */
	uint32_t MAX_ELITE_SOLUTIONS = DEFAULT_MAX_ELITE_SOLUTIONS;
	uint32_t MAX_ALTERNATIVES = DEFAULT_MAX_ALTERNATIVES;
	uint32_t MIN_ITERS_PER_TUPLE = DEFAULT_MIN_ITERS_PER_TUPLE;
}


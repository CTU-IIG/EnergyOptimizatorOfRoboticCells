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

#ifndef HLIDAC_PES_SOLUTION_H
#define HLIDAC_PES_SOLUTION_H

/*!
 * \file Solution.h
 * \author Libor Bukata
 * \brief A representation of the solution that is algorithm independent.
 */

#include <iostream>
#include <map>
#include <utility>
#include "ILPModel/SolverInterface.h"

struct PrecalculatedMapping;

/*!
 * \brief Enum declares constants specifying whether a found solution
 * is optimal, feasible, infeasible, or unknown (i.e. not found), respectively.
 */
enum State {
	OPTIMAL, FEASIBLE, INFEASIBLE, UNKNOWN
};

//! Conversion between the ILP solver status and the solution state.
State convert(const Status& ilpStatus);

/*!
 * The structure uniquely describes the solution found by an algorithm.
 * Besides the solution, runtime, and memory usage can be also recorded.
 * \brief The structure representing a solution found by an algorithm.
 */
struct Solution {
	//! It constructs an empty solution.
	Solution() : status(UNKNOWN), totalEnergy(0.0), lowerBound(0.0), runTime(-1.0), memUsage(-1.0) { }

	//! Enum specifying whether the solution is optimal, feasible, or infeasible... \see State
	State status;
	//! The amount of energy required per robot cycle.
	double totalEnergy;
	//! Lower bound on energy, i.e.  there is not a solution consuming less energy than this number.
	double lowerBound;
	//! Run time (seconds) of the optimization algorithm required for obtaining this solution.
	double runTime;
	//! Memory usage (bytes) of the optimization algorithm, currently not used due to implementation issues.
	double memUsage;
	//! Mapping of the activity identification to its start time and duration.
	std::map<uint32_t, std::pair<double, double> > startTimeAndDuration;
	//! It maps the identification of a static activity to its assigned coordinate and used power saving mode.
	std::map<uint32_t, std::pair<uint32_t, uint32_t> > pointAndPowerMode;

};

/*!
 * \defgroup sol_print solution printing or writing
 * The module contains the output stream operators for Solution class,
 * and functions dedicated to writing brief or detailed results to *.csv files.
 * \brief Output operators and other related functions for solution printing or writing.
 * @{
 */

//! It prints the solution to the output stream.
std::ostream& operator<<(std::ostream& out, const Solution& s);
//! It prints the state of a solution to the output stream.
std::ostream& operator<<(std::ostream& out, const State& st);

/*!
 * \param OUT An output stream to which a brief result will be written.
 * \param instanceName A short name of the instance.
 * \param s %Solution to be printed.
 * \brief It prints the solution as follows: 'instance name (state, runtime s): energy'.
 */
void writeBriefResultToStream(std::ostream& OUT, const std::string& instanceName, const Solution& s);
/*!
 * \param instanceName A short name of the instance.
 * \param pathToFile A path to the file to which a brief result will be written.
 * \param s %Solution to be written.
 * \brief It writes the solution to the file in the same format as \ref writeBriefResultToStream function.
 */
void writeBriefResultToCsvFile(const std::string& instanceName, const std::string& pathToFile, const Solution& s);
/*!
 * \param file A file to which the solution will be written in the form of csv file.
 * \param s %Solution to be written.
 * \param mapper Fast access to the data structure of the related robotic cell.
 * \brief It writes the solution to a csv file, the header is 'activity id,
 * start time, duration, type, point, power saving mode, movement, energy'.
 */
void writeSolutionToCsvFile(const std::string& file, const Solution& s, const PrecalculatedMapping& mapper);

//! @}

#endif


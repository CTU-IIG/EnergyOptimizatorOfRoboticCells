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

#ifndef HLIDAC_PES_VARIABLE_MAPPING_LP_H
#define HLIDAC_PES_VARIABLE_MAPPING_LP_H

/*!
 * \file VariableMappingLP.h
 * \author Libor Bukata
 * \brief Mapping of continuous variables of the energy optimization problem.
 */

#include <map>
#include <string>
#include <vector>
#include "RoboticLine.h"
#include "ILPModel/ILPModel.h"

//! Identification of the activity is mapped to the index of the variable.
using map1to1 = std::map<uint32_t, uint32_t>;

/*!
 * An instance of this class creates a mapping of the continuous variables
 * occurring in the energy optimization problem.
 * \brief Mapping of continuous variables occurring in the energy optimization problem.
 * \see \ref math_form
 */
struct VariableMappingLP {
	/*!
	 * \param r Only the mandatory activities of this robot are taken into account.
	 * \brief Constructs the mapping of the continuous variables.
	 */
	VariableMappingLP(Robot* r);
	/*!
	 * \param l Only the mandatory activities of this robotic cell are taken into account.
	 * \brief Constructs the mapping of the continuous variables.
	 */
	VariableMappingLP(const RoboticLine& l);

	uint32_t numberOfVariables() const { return W.size()+s.size()+d.size(); }
	/*!
	 * \param activities Additional (e.g. optional) activities.
	 * \param mandatory True if only mandatory activities should be considered, otherwise false.
	 * \param mapW Whether continuous variables W (activity energy consumption) should be also mapped.
	 * \brief It adds the mapping for extra activities.
	 * \note This method is usually called after the constructor to add optional activities.
	 */
	void addActivities(const std::vector<Activity*>& activities, bool mandatory, bool mapW = true);

	//! Maps activity identification to the index of variable corresponding to the energy consumption of this activity.
	map1to1 W;
	//! Maps activity identification to the index of variable corresponding to the start time of this activity.
	map1to1 s;
	//! Maps activity identification to the index of variable corresponding to the duration of this activity.
	map1to1 d;
	//! Continuous variables of the problem with the well-specified domains.
	std::vector<Variable> variables;
	//! Description of the continuous variables.
	std::vector<std::string> varDesc;
};

/*!
 * \param solution A solution of an LP/ILP problem.
 * \param mapping A mapping of some continuous variable.
 * \return Extracted values from the solution in the form of map, e.g. 'activity identification -> duration'.
 * \brief It extracts the values of some continuous variables from the solution.
 */
std::map<uint32_t, double> inverseMapping(const std::vector<double>& solution, const map1to1& mapping);

#endif

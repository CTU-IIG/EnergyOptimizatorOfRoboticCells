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

#ifndef HLIDAC_PES_VARIABLE_MAPPING_ILP_H
#define HLIDAC_PES_VARIABLE_MAPPING_ILP_H

/*!
 * \file VariableMappingILP.h
 * \author Libor Bukata
 * \brief Mapping of integer, to be more precise binary, variables of the energy optimization problem.
 */

#include <vector>
#include <utility>
#include "ILPSolver/VariableMappingLP.h"

//! Mapping of variables indexed by two numbers.
using map2to1 = std::map<std::pair<uint32_t, uint32_t>, uint32_t>;
//! Mapping of variables indexed by four numbers.
using map4toN = std::map<std::pair<uint64_t, uint64_t>, std::vector<uint32_t> >;

/*!
 * The mapping of both the continuous and binary variables is created by an object of this class.
 * \brief Mapping of continuous and binary variables.
 * \see \ref math_form
 */
struct VariableMappingILP : public VariableMappingLP {
	public:
		//! Constructs the mapping of all the variables related to robot \a r.
		VariableMappingILP(Robot* r);
		//! Constructs the mapping of all the variables related to robotic cell \a l.
		VariableMappingILP(const RoboticLine& l);

		//! Total number of variables including both continuous and binary ones.
		uint32_t numberOfVariables() const;

		/*!
		 * Maps static activity identification and coordinate/location to the index of the binary variable
		 * that indicates whether the coordinate/location is selected for this activity or not.
		 */
		map2to1 x;
		/*!
		 * Maps the identifications of the static activity and power saving mode to the index of
		 * the binary variable that indicates whether the power saving mode is selected for this activity or not.
		 */
		map2to1 z;
		/*!
		 * Maps the identifications of the dynamic activity and movement to the index of
		 * the binary variable that indicates whether the movement is selected for this activity or not.
		 */
		map2to1 y;
		/*!
		 * Maps the identifications of the dynamic activity and its successor to the index of
		 * the binary variable that indicates whether this optional dynamic activity is performed or not.
		 */
		map2to1 w;
		//! Binary variables used for collisions resolution.
		map4toN c;
	private:
		//! It creates the mapping for \a x, \a z, and \a y variables.
		void addActivityBinaryVariables(Robot* r);
		//! It constructs the mapping for \a w variables.
		void addActivityOrderBinaryVariables(Robot* r);
		//! Mapping of \a c variables is created.
		void addCollisionBinaryVariables(const RoboticLine& l);
};

/*!
 * \param solution A solution of the ILP problem.
 * \param mapping A mapping of some binary variables.
 * \return A map containing the indices (keys of mapping) of binary variables set to true.
 * \brief It extracts the variables set to true and returns the ,,selected" indices.
 * \code
 * // Solution of an ILP problem.
 * vector<double> sol = { 0.0, 0.0, 1.0, 0.0, 1.0, 0.0 };
 * // Mapping of variables, e.g. {{activity id, location id}, index}*.
 * map2to1 mapping = { {{0,0},0}, {{0,1},1}, {{0,2},2}, {{1,3},3}, {{1,4},4}, {{1,5},5} };
 * // It returns {{0,2}, {1,4}}, for our example above it means that locations
 * // with id 2 and 4 were selected for activities 0 and 1, respectively.
 * const auto& ret = inverseMapping(sol, mapping);
 * \endcode
 */
std::map<uint32_t, uint32_t> inverseMapping(const std::vector<double>& solution, const map2to1& mapping);

#endif

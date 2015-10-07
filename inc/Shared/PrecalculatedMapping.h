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

#ifndef HLIDAC_PES_PRECALCULATED_MAPPING_H
#define HLIDAC_PES_PRECALCULATED_MAPPING_H

/*!
 * \file PrecalculatedMapping.h
 * \author Libor Bukata
 * \brief The structures and methods suitable for fast searching in the data structure of the robotic cell.
 */

#include <map>
#include <set>
#include <vector>
#include <utility>
#include <unordered_map>
#include "RoboticLine.h"
#include "Shared/Utils.h"
#include "Solution/Solution.h"

/*!
 * \addtogroup hash_def calculation of hashes
 * As there is not default implementation of the hash for some data structures,
 * we implemented the missing hashes by using the template specializations.
 * \brief Methods and structures related to the calculation of hashes.
 * \see pack, unpack
 * @{
 */
namespace std {
	//! Template specialization of the hash for a pair of uint32_t.
	template <>
	struct hash<pair<uint32_t, uint32_t>> {
		size_t operator() (const pair<uint32_t, uint32_t>& p) const {
			hash<uint64_t> h;
			uint64_t v64 = pack(p.first, p.second);
			return h(v64);
		}
	};

	//! Template specialization of the hash for a pair of Location pointers.
	template <>
	struct hash<pair<Location*, Location*>> {
		size_t operator() (const pair<Location*, Location*>& p) const {
			hash<uint64_t> h;
			uint64_t v64 = pack(p.first->point(), p.second->point());
			return h(v64);
		}
	};
}
//! @}

//! Defines the data type for an element of the spatial compatibility, i.e. (rob1 idx, rob2 idx, act1, act2, {{loc1, loc2}*}).
using SpatialCmpTuple = std::tuple<uint32_t, uint32_t, StaticActivity*, StaticActivity*, std::vector<std::pair<Location*, Location*>>>;

/*!
 * The structure consists of various (unordered) maps and the vector for fast access to elements of the spatial compatibility.
 * The main goal of the maps is to provide fast access based on keys, i.e. unique identifiers like e.g. location ID, point, pointer values, etc.
 * As the mapped values are mostly pointers, it allows to immediately explore the related data-structure and close parts of the robotic cell.
 * \brief The structure contains the maps for fast searching in the robotic cell.
 */
struct PrecalculatedMapping {
	//! %Activity identification to its pointer.
	std::map<uint32_t, Activity*> aidToActivity;
	//! Unique coordinate, i.e. a point, to the related location.
	std::map<uint32_t, Location*> pointToLocation;
	//! Two coordinates are mapped to the movement starting/ending at the first/second coordinate, respectively.
	std::unordered_map<std::pair<uint32_t, uint32_t>, Movement*> pointsToMovement;
	//! It finds the movement between two locations if exists.
	std::unordered_map<std::pair<Location*, Location*>, Movement*> locationsToMovement;
	//! It finds all the starting coordinates of the movements entering the given coordinate, i.e. the point.
	std::map<uint32_t, std::vector<uint32_t> > pointToPredecessorPoints;
	//! It finds all the ending coordinates of the movements leaving the given coordinate, i.e. the point.
	std::map<uint32_t, std::vector<uint32_t> > pointToSuccessorPoints;
	//! Two-step mapping taking two static activities and returning a list of compatible pairs of locations.
	std::map<StaticActivity*, std::map<StaticActivity*, std::vector<std::pair<Location*, Location*>>>> sptComp;
	//! Enables to fast iterate through spatial compatibility elements. \see SpatialCmpTuple
	std::vector<SpatialCmpTuple> sptCompVec;
	//! It takes a pointer to a movement and returns the vector of precalculated coordinates corresponding to a piece-wise linearized energy function.
	std::map<Movement*, std::tuple<std::vector<double>, std::vector<double>>> mvToEnergyFunc;
	//! %Location and the selected power saving mode are mapped to the linear energy function, i.e. the size of each vector in the tuple is 2.
	std::map<std::pair<Location*, RobotPowerMode*>, std::tuple<std::vector<double>, std::vector<double>>> locToEnergyFunc;
	//! It finds colliding movements and locations for a given location or movement.
	std::map<ActivityMode*, std::set<ActivityMode*>> collisionSearch;
	//! It searches for all the time lags starting from the given activity.
	std::map<Activity*, std::vector<TimeLag>> timeLagsFromAct;
	//! It searches for all the time lags ending in the given activity.
	std::map<Activity*, std::vector<TimeLag>> timeLagsToAct;
};

/*!
 * \param s %Solution of the energy optimization problem.
 * \param m Mapping calculated for the related robotic cell.
 * \param da A dynamic activity located in the robotic cell.
 * \return Movement of the dynamic activity that was selected in the solution.
 * \brief It extracts the selected movement of the given dynamic activity from the solution.
 */
Movement* getSelectedMovement(const Solution& s, const PrecalculatedMapping& m, DynamicActivity* da);

/*!
 * \param from Specifies starting location/activity.
 * \param to Specifies ending location/activity.
 * \tparam X,Y Each of them is either a pointer to location or static activity.
 * \return Movements between \a from and \a to.
 * \brief It finds all the movements located between \a from and \a to and returns them in the vector.
 */
template <class X, class Y>
extern std::vector<Movement*> findMovements(X from, Y to);

#endif

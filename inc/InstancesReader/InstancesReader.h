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

#ifndef HLIDAC_PES_INSTANCES_READER_H
#define HLIDAC_PES_INSTANCES_READER_H

/*!
 * \file InstancesReader.h
 * \author Libor Bukata
 * \brief Checking and post-processing of parsed datasets.
 */

#include <stdint.h>
#include <string>
#include <vector>
#include "RoboticLine.h"
#include "Shared/PrecalculatedMapping.h"

/*!
 * An instance of the InstancesReader class is responsible for the full process of the dataset processing,
 * i.e. parsing, filling the data structures of robotic cells, checking the correctness of instances,
 * and finally creating the mappings for fast searching by a key.
 * However, a part of the work is carried out by XmlReader and InstanceChecker classes that
 * help with XML parsing and instance checking, respectively.
 * \brief The class is intended to be used by users for the dataset parsing and checking.
 * \see XmlReader, InstanceChecker
 */
class InstancesReader {
	public:
		InstancesReader() { }
		//! It processes the dataset specified in \a Settings::DATASET_FILE.
		void readInstances();
		std::vector<RoboticLine> dataset() const { return mLines; }
		std::vector<PrecalculatedMapping> precalculatedMappings() const { return mMapping; }
		std::string datasetName() const { return mDatasetName; }
		std::string datasetDescription() const { return mDatasetDescription; }
	private:
		//! A wrapper method calling all the related filling methods.
		void completeFillingOfMapping();
		/*!
		 * \param mapping A reference to the structure with precalculated mappings and data.
		 * \param line Robotic cell for which the mapping will be updated.
		 * \brief Maps related to activities relations (precedences) are filled.
		 */
		void fillActivitiesRelations(PrecalculatedMapping& mapping, const RoboticLine& line);
		/*!
		 * \param mapping A reference to the structure with precalculated mappings and data.
		 * \param line Robotic cell for which the mapping will be updated.
		 * \brief Energy functions are piece-wise linearized, and vectors of coordinates are written to the mapping.
		 * \remark The functions are piece-wise linearized in a smart way, it means that the function segments
		 * are not equidistant but their lengths are carefully selected with respect to the second derivations and function errors.
		 */
		void fillDiscretizedEnergyFunctions(PrecalculatedMapping& mapping, const RoboticLine& line);
		/*!
		 * \param mapping A reference to the structure with precalculated mappings and data.
		 * \param line Robotic cell for which the mapping will be updated.
		 * \brief It fills the searching maps related to the robot synchronizations and spatial compatibility.
		 */
		void fillInterRobotOperations(PrecalculatedMapping& mapping, const RoboticLine& line);
		/*!
		 * \param mapping A reference to the structure with precalculated mappings and data.
		 * \param line Robotic cell for which the mapping will be updated.
		 * \brief It fills the maps for fast searching of colliding movements or locations.
		 */
		void fillCollisionZones(PrecalculatedMapping& mapping, const RoboticLine& line);

		//! Robotic cells read from the dataset.
		std::vector<RoboticLine> mLines;
		//! Precalculated mappings suitable for fast searching in the data structures of robotic cells.
		std::vector<PrecalculatedMapping> mMapping;
		//! Name of the dataset.
		std::string mDatasetName;
		//! Description of the dataset.
		std::string mDatasetDescription;
};

/*!
 * The purpose of this class is to carry out various checks of the instance,
 * e.g. check durations, energy functions, precedence graphs, operations, and collisions.
 * \brief An instance of this class checks a dataset instance.
 */
class InstanceChecker {
	public:
		/*!
		 * \param line A robotic cell to be checked.
		 * \brief Constructs an instance checker.
		 */
		InstanceChecker(const RoboticLine& line) : mLine(line) { }
		//! The method verifies the correctness of the instance. If incorrect, then an exception is thrown.
		void checkInstance() const;
	private:
		/*!
		 * \param sa Static activity to be checked.
		 * \brief It checks the activity duration and whether the input power of power saving modes is defined.
		 */
		void checkStaticActivity(StaticActivity* sa) const;
		/*!
		 * \param da Dynamic activity to be verified.
		 * \brief Verifies the activity duration and convexity of energy functions.
		 */
		void checkDynamicActivity(DynamicActivity* da) const;
		/*!
		 * \param r %Robot to be checked.
		 * \brief It checks that there is a hamiltonian circuit in the robot graph.
		 */
		void checkRobotGraph(Robot* r) const;
		/*!
		 * \param op Inter-robot operation to be verified.
		 * \brief Checks that each time lag or spatial compatibility pair is defined for two different robots.
		 */
		void checkOperation(InterRobotOperation* op) const;
		/*!
		 * \param col Collision to be checked.
		 * \brief Checks that the collision is defined for two different robots.
		 */
		void checkCollision(const std::pair<ActivityMode*, ActivityMode*>& col) const;

		//! A reference to a robotic cell that will be tested for inconsistencies.
		const RoboticLine& mLine;
};

#endif

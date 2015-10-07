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

#ifndef HLIDAC_PES_XML_READER_H
#define HLIDAC_PES_XML_READER_H

/*!
 * \file XmlReader.h
 * \author Libor Bukata
 * \brief The file declares XmlReader class which purpose is to parse and check XML datasets.
 * \note Libxml2 and libxmlpp libraries are employed to parse XML files.
 */

#include <cassert>
#include <map>
#include <string>
#include <vector>
#include <libxml++/libxml++.h>
#include "Settings.h"
#include "RoboticLine.h"
#include "Shared/PrecalculatedMapping.h"

/*!
 * Instance of XmlReader class parses XML datasets with problem instances,
 * and fills data structures connected with the robotic cell.
 * During the reading some elemental checks are performed, however,
 * more complex checking and post-processing is accomplished by \a InstancesReader class.
 * \brief Parser of XML datasets.
 * \see RoboticLine, InstancesReader
 */
class XmlReader {
	private:
		XmlReader() { }
		//! It parses the XML dataset (specified in \a Settings::DATASET_FILE) and checks it against schema.
		void readDatasetFromXmlFile();

		/*!
		 * \param rootNode The node of a document tree corresponding to '\<dataset>...\</dataset>' elements.
		 * \brief It parses all the dataset and stores the data to member variables.
		 */
		void processRootNode(const xmlpp::Node *rootNode);
		/*!
		 * \param node The node of a document tree corresponding to '\<instance>...\</instance>' elements.
		 * \brief It parses the problem instance specified by the node.
		 */
		void processInstanceNode(const xmlpp::Node *node);
		/*!
		 * \param robotNode The node of a document tree corresponding to '\<robot>...\</robot>' elements.
		 * \return Pointer to a newly created robot.
		 * \brief It parses the data related to the robot, and creates the instance of Robot class.
		 */
		Robot* processRobotNode(const xmlpp::Node *robotNode);
		/*!
		 * \param node The node of a document tree corresponding to '\<static-activity>...\</static-activity>' elements.
		 * \param robotModes The vector containing the power saving modes of the robot that performs the activity specified by the node.
		 * \return Pointer to a newly created static activity.
		 * \brief An instance of the static activity is created from the parsed data and power saving modes of the robot.
		 */
		StaticActivity* processStaticActivityNode(const xmlpp::Node *node, const std::vector<RobotPowerMode*>& robotModes);
		/*!
		 * \param node The node of a document tree corresponding to '\<dynamic-activity>...\</dynamic-activity>' elements.
		 * \return Pointer to a newly created dynamic activity.
		 * \brief An instance of the dynamic activity is created from the parsed data.
		 */
		DynamicActivity* processDynamicActivityNode(const xmlpp::Node *node);
		/*!
		 * \param interRobotOperationNode The node of a document tree corresponding to '\<operation>...\</operation>' elements.
		 * \return Pointer to a newly created inter-robot operation.
		 * \brief It creates a new inter-robot operation from parsed data.
		 */
		InterRobotOperation* processInterRobotOperationNode(const xmlpp::Node *interRobotOperationNode) const;
		/*!
		 * \param collisionPairNode The node of a document tree corresponding to '\<collision-pair>...\</collision-pair>' elements.
		 * \return A pair containing the parsed collision pair.
		 * \brief It parses an element containing time disjunctive pair, and returns the pointers to colliding locations/movements.
		 */
		std::pair<ActivityMode*, ActivityMode*> processCollisionPairNode(const xmlpp::Node *collisionPairNode) const;

		/*!
		 * \param node A node of a document tree.
		 * \return A pointer to an element, i.e. '\<x>...\</x>'.
		 * \brief The method casts xmlpp::Node to xmlpp::Element, and in case of success the result is returned, otherwise an exception is thrown.
		 */
		const xmlpp::Element* castToElement(const xmlpp::Node *node) const;
		/*!
		 * \param node An element from which the attribute is extracted.
		 * \param attributeName The name of the attribute.
		 * \param required Specifies whether the attribute is mandatory.
		 * \return String containing the attribute value, e.g. "hello" if element is '\<x attr="hello"/>'.
		 * \brief It parses the value assigned to the specified attribute and returns it in the form of string.
		 * \note If the attribute is not mandatory and also not present, then an empty string is returned.
		 */
		std::string getAttribute(const xmlpp::Element* node, const std::string& attributeName, bool required = true) const;
		/*!
		 * \param element An element that may contain an inner element called childName.
		 * \param childName An element containing the text.
		 * \param required Indicates whether the child element is mandatory.
		 * \return A text extracted from the child element or an empty string.
		 * \brief It tries to extract the text from the child element, e.g. getTextFromElement(instanceElement, "name", false)
		 * returns the name of the instance or an empty string if the name is not specified.
		 * \note If the child element is optional, then an empty string can be returned.
		 */
		std::string getTextFromElement(const xmlpp::Element* element, const std::string& childName, bool required = true) const;

		//! Vector of problem instances, i.e. robotic cells.
		std::vector<RoboticLine> mLines;
		//! Vector of precalculated mappings for fast access by a key.
		std::vector<PrecalculatedMapping> mMapping;
		//! Name of the dataset.
		std::string mDatasetName;
		//! Description of the dataset.
		std::string mDatasetDescription;

		//! A map mapping the activity identification to the pointer to the related activity.
		std::map<uint32_t, Activity*> mAidToActivity;
		//! Unique mapping of coordinate identifications (called points) to the related location.
		std::map<uint32_t, Location*> mPointToLocation;
		//! It maps the movement to start and end coordinates of the movement.
		std::map<Movement*, std::pair<uint32_t, uint32_t>> mMovementToPoints;

		//! Only %InstancesReader is allowed to use the data and methods of this class.
		friend class InstancesReader;
};

#endif


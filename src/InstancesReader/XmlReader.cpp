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

#include <algorithm>
#include <map>
#include <iostream>
#include <string>
#include <typeinfo>
#include <utility>

#if VALIDATE_INPUT_FILE_AGAINST_SCHEMA == 1
#include <libxml++/validators/schemavalidator.h>
#endif

#include "SolverConfig.h"
#include "RoboticLine.h"
#include "Shared/Utils.h"
#include "Shared/Exceptions.h"
#include "Shared/NumericConstants.h"
#include "InstancesReader/XmlReader.h"

using namespace std;
using namespace xmlpp;

void XmlReader::readDatasetFromXmlFile()	{
	DomParser parser(Settings::DATASET_FILE);
	if (parser)	{
		#ifdef VALIDATE_INPUT_FILE_AGAINST_SCHEMA
		SchemaValidator validator(string(INSTALL_PATH)+"/share/"+string(PROJECT_NAME)+"-"+string(PROGRAM_VERSION)+"/dataset.xsd");
		validator.validate(parser.get_document());
		#endif
		const Node *rootNode = parser.get_document()->get_root_node();
		processRootNode(rootNode);
	} else {
		throw InvalidDatasetFile(caller(), "Cannot parse input xml file!");
	}
}

void XmlReader::processRootNode(const Node *rootNode)	{
	const Element *dataset = castToElement(rootNode);
	mDatasetName = getTextFromElement(dataset, "name", false);
	mDatasetDescription = getTextFromElement(dataset, "desc", false);
	mLines.clear();

	const Node::NodeList instances = dataset->get_children("instance");
	for (Node *instance : instances)
		processInstanceNode(instance);
}

void XmlReader::processInstanceNode(const Node *node)	{
	RoboticLine line;
	mAidToActivity.clear();
	mPointToLocation.clear();
	const Element *instance = castToElement(node);
	line.name(getTextFromElement(instance, "name", false));
	line.description(getTextFromElement(instance, "desc", false));

	const NodeSet robots = instance->find("robots/robot");
	for (const Node* robotNode : robots)
		line.addRobot(processRobotNode(robotNode));

	for (Robot* r : line.robots())	{
		for (Activity *a : r->activities())
			setValue(mAidToActivity, a->aid(), a, caller());
	}

	const NodeSet interRobotOperations = instance->find("inter-robot-operations/operation");
	for (const Node* op : interRobotOperations)
		line.addInterRobotOperation(processInterRobotOperationNode(op));

	const NodeSet collisionZones = instance->find("collision-zones/collision-pair");
	for (const Node* cz : collisionZones)
		line.addCollision(processCollisionPairNode(cz));

	line.productionCycleTime(stod(getTextFromElement(instance, "production-cycle-time")));
	line.initialiseDataStructures(mPointToLocation, mMovementToPoints);

	PrecalculatedMapping mapping;
	mapping.aidToActivity = move(mAidToActivity);
	mapping.pointToLocation = move(mPointToLocation);

	mLines.push_back(line);
	mMapping.push_back(move(mapping));
}

Robot* XmlReader::processRobotNode(const Node *robotNode) {

	Robot* robot = new Robot();
	const Element *robotElement = castToElement(robotNode);

	robot->name(getTextFromElement(robotElement, "name", false));
	robot->description(getTextFromElement(robotElement, "desc", false));

	// In reality it is vector<Node*> powerModes!
	const NodeSet powerModes = robotElement->find("power-saving-modes/power-mode");
	for (const Node* node : powerModes)	{
		const Element *powerMode = castToElement(node);
		RobotPowerMode *mode = new RobotPowerMode(stoul(getAttribute(powerMode, "pid")));
		mode->name(getTextFromElement(powerMode, "name", false));
		mode->description(getTextFromElement(powerMode, "desc", false));
		mode->minimalDelay(stod(getTextFromElement(powerMode, "minimal-idle-time")));
		string inputPower = getTextFromElement(powerMode, "expected-input-power", false);
		if (!inputPower.empty())
			mode->expectedInputPower(stod(inputPower));

		robot->addPowerMode(mode);
	}

	const NodeSet staticActivities = robotElement->find("activities/static-activity");
	for (const Node* node : staticActivities)
		robot->addActivity(processStaticActivityNode(node, robot->powerModes()));

	const NodeSet dynamicActivities = robotElement->find("activities/dynamic-activity");
	for (const Node* node : dynamicActivities)
		robot->addActivity(processDynamicActivityNode(node));

	return robot;
}

StaticActivity* XmlReader::processStaticActivityNode(const Node *node, const vector<RobotPowerMode*>& robotModes) {

	const Element *activityElement = castToElement(node);
	StaticActivity *staticActivity = new StaticActivity(stoul(getAttribute(activityElement, "aid")));

	if (getAttribute(activityElement, "last_in_cycle", false) == "true")
		staticActivity->lastInCycle(true);

	staticActivity->name(getTextFromElement(activityElement, "name", false));
	staticActivity->description(getTextFromElement(activityElement, "desc", false));

	const NodeSet locations = activityElement->find("locations/location");
	for (const Node *loc : locations)	{

		const Element *locElement = castToElement(loc);
		Location *location = new Location(stoul(getAttribute(locElement, "lid")));
		location->point(stoul(getTextFromElement(locElement, "point")));

		const NodeSet locDepPowerInput = locElement->find("location-dependent-power-consumption/consumption");
		for (const Node *ldpcNode : locDepPowerInput)	{
			const Element *ldpcElement = castToElement(ldpcNode);
			uint32_t pid = stoul(getAttribute(ldpcElement, "pid"));
			auto spmIt = find_if(robotModes.cbegin(), robotModes.cend(), [pid](RobotPowerMode* m) { return pid == m->pid(); });
			if (spmIt != robotModes.cend())	{
				LocationDependentPowerConsumption ldpc(*spmIt);
				ldpc.inputPower(stod(getAttribute(ldpcElement, "input_power")));
				location->addLocationDependentPowerConsumption(ldpc);
			} else	{
				throw InvalidDatasetFile(caller(), "Invalid value of 'pid' attribute!", ldpcNode->get_line());
			}
		}

		location->parent(staticActivity);
		setValue(mPointToLocation, location->point(), location, caller());

		staticActivity->addLocation(location);
	}

	double minimalDelay = F64_MAX;
	for (RobotPowerMode* pwrm : robotModes)
		minimalDelay = min(minimalDelay, pwrm->minimalDelay());

	staticActivity->minAbsDuration(max(minimalDelay, stod(getTextFromElement(activityElement, "min-duration"))));
	staticActivity->maxAbsDuration(stod(getTextFromElement(activityElement, "max-duration")));
	// Workaround the incapability of ILP solvers to solve the problem with the fixed variables.
	if (abs(staticActivity->maxAbsDuration()-staticActivity->minAbsDuration()) < TIME_ERR)
		staticActivity->maxAbsDuration(staticActivity->minAbsDuration()+TIME_ERR);

	return staticActivity;
}

DynamicActivity* XmlReader::processDynamicActivityNode(const Node *node) {

	const Element *activityElement = castToElement(node);
	DynamicActivity *dynamicActivity = new DynamicActivity(stoul(getAttribute(activityElement, "aid")));

	dynamicActivity->name(getTextFromElement(activityElement, "name", false));
	dynamicActivity->description(getTextFromElement(activityElement, "desc", false));

	double minDuration = F64_MAX, maxDuration = F64_MIN;
	const Element *movements = castToElement(activityElement->get_first_child("movements"));
	string a1 = getAttribute(movements, "from_aid", false), a2 = getAttribute(movements, "to_aid", false);
	int64_t fromActivity = (a1.empty() ? -1 : stol(a1)), toActivity = (a2.empty() ? -1 : stol(a2));

	const NodeSet movementSet = movements->find("movement");
	for (const Node *movementNode : movementSet)	{
		const Element *movementElement = castToElement(movementNode);

		Movement *mv = new Movement(stoul(getAttribute(movementElement, "mid")));
		mv->minDuration(stod(getTextFromElement(movementElement, "min-duration")));
		mv->maxDuration(stod(getTextFromElement(movementElement, "max-duration")));
		if (mv->minDuration() < TIME_ERR)	{
			string mid = to_string(mv->mid());
			delete dynamicActivity; delete mv;
			throw InvalidDatasetFile(caller(), "Dynamic activity's movement "+mid+" must have positive duration!", movementNode->get_line());
		}

		// Workaround the incapability of ILP solvers to solve the problem with the fixed variables.
		if (abs(mv->maxDuration()-mv->minDuration()) < TIME_ERR)
			mv->maxDuration(mv->minDuration()+TIME_ERR);

		const NodeSet monomials = movementElement->find("energy-function/monomial");
		for (const Node *monomialNode : monomials)	{
			const Element *monomialElement = castToElement(monomialNode);
			int32_t degree = stoi(getAttribute(monomialElement, "degree"));
			double coeff = stod(getAttribute(monomialElement, "coeff"));
			mv->addMonomial({degree, coeff});
		}

		uint32_t fromPoint = stoul(getTextFromElement(movementElement, "from-point"));
		uint32_t toPoint = stoul(getTextFromElement(movementElement, "to-point"));

		Location *locFrom = nullptr, *locTo = nullptr;
		StaticActivity *actFrom = nullptr, *actTo = nullptr;
		auto fSit = mPointToLocation.find(fromPoint), tSit = mPointToLocation.find(toPoint);
		if (fSit == mPointToLocation.cend() || tSit == mPointToLocation.cend())	{
			delete dynamicActivity; delete mv;
			throw InvalidDatasetFile(caller(), "Dynamic activity's point is not linked with any static activity!", movementNode->get_line());
		} else {
			locFrom = fSit->second; locTo = tSit->second;
			assert(locFrom != nullptr && locTo != nullptr && "Unexpected null pointers!");
			actFrom = locFrom->parent(); actTo = locTo->parent();
			assert(actFrom != nullptr && actTo != nullptr && "Should not be nullptr as it has been set in processRobotNode method!");
		}

		if ((actFrom->aid() != fromActivity && fromActivity != -1) || (actTo->aid() != toActivity && toActivity != -1))	{
			delete dynamicActivity; delete mv;
			throw InvalidDatasetFile(caller(), "Optional attributes 'from_aid' and/or 'to_aid' are invalid!", movementNode->get_line());
		}

		minDuration = min(minDuration, mv->minDuration());
		maxDuration = max(maxDuration, mv->maxDuration());
		setValue(mMovementToPoints, mv, {fromPoint, toPoint}, caller());

		dynamicActivity->addMovement(mv);
	}

	dynamicActivity->minAbsDuration(minDuration);
	dynamicActivity->maxAbsDuration(maxDuration);

	return dynamicActivity;
}

InterRobotOperation* XmlReader::processInterRobotOperationNode(const Node *interRobotOperationNode) const {

	const Element *operationElement = castToElement(interRobotOperationNode);
	InterRobotOperation* operation = new InterRobotOperation(stoul(getAttribute(operationElement, "oid")));
	operation->name(getTextFromElement(operationElement, "name", false));
	operation->description(getTextFromElement(operationElement, "desc", false));

	const NodeSet timeLags = operationElement->find("time-compatibility/time-lag");
	for (const Node *timeLagNode : timeLags)	{
		const Element *timeLagElement = castToElement(timeLagNode);

		uint32_t fromId = stoul(getTextFromElement(timeLagElement, "from-activity"));
		uint32_t toId = stoul(getTextFromElement(timeLagElement, "to-activity"));
		double length = stod(getTextFromElement(timeLagElement, "length"));
		int32_t height = stoi(getTextFromElement(timeLagElement, "height"));

		auto sit1 = mAidToActivity.find(fromId), sit2 = mAidToActivity.find(toId);
		if (sit1 != mAidToActivity.cend() && sit2 != mAidToActivity.cend())	{
			operation->addTimeLag({sit1->second, sit2->second, length, height});
		} else {
			string msg = "Time lag '"+to_string(fromId)+" -> "+to_string(toId)+"' between non-existing activities!\n";
			throw InvalidDatasetFile(caller(), msg+"Invalid inter-robot time-lag!", timeLagNode->get_line());
		}
	}

	const NodeSet spaceComp = operationElement->find("spatial-compatibility/compatible-pair");
	for (const Node *cmpPairNode : spaceComp)	{
		const Element *cmpPairElement = castToElement(cmpPairNode);
		const NodeSet locations = cmpPairElement->find("location");

		try {
			vector<Location*> cmpPair;
			for (uint32_t i = 0; i < min(locations.size(), 2ul); ++i)	{
				const Element *locElement = castToElement(locations[i]);
				auto sit = mAidToActivity.find(stoul(getAttribute(locElement, "aid")));
				if (sit != mAidToActivity.cend())	{
					StaticActivity *sa = dynamic_cast<StaticActivity*>(sit->second);
					if (sa != nullptr)	{
						Location *l = sa->findLocation(stoul(getAttribute(locElement, "lid")));
						cmpPair.push_back(l);
					} else {
						string msg = "Spatial compatibility can only be resolved between static activities!";
						throw InvalidDatasetFile(caller(), msg, locElement->get_line());
					}
				} else {
					throw InvalidDatasetFile(caller(), "Invalid 'aid' attribute in '<location>' element!", locElement->get_line());
				}
			}

			if (cmpPair.size() == 2)
				operation->addSpatialCompatibilityPair(cmpPair.front(), cmpPair.back());
			else
				throw InvalidDatasetFile(caller(), "Invalid number of elements in the '<compatible-pair>' element!");

		} catch (...)	{
			throw_with_nested(InvalidDatasetFile(caller(), "Cannot process the spatial compatibility pair!", cmpPairElement->get_line()));
		}
	}

	return operation;
}

pair<ActivityMode*, ActivityMode*> XmlReader::processCollisionPairNode(const xmlpp::Node *collisionPairNode) const	{

	const Element *collisionPairElement = castToElement(collisionPairNode);

	vector<ActivityMode*> collisionPair;
	NodeSet locations = collisionPairElement->find("location");
	NodeSet movements = collisionPairElement->find("movement");

	for (const Node *locNode : locations)	{
		const Element *locElement = castToElement(locNode);
		auto sit = mAidToActivity.find(stoul(getAttribute(locElement, "aid")));
		if (sit != mAidToActivity.cend())	{
			StaticActivity* sa = dynamic_cast<StaticActivity*>(sit->second);
			if (sa != nullptr)	{
				collisionPair.emplace_back(sa->findLocation(stoul(getAttribute(locElement, "lid"))));
			} else {
				string msg = "Referenced 'aid' does not correspond to the static activity for 'location' element!";
				throw InvalidDatasetFile(caller(), msg, locNode->get_line());
			}
		} else {
			throw InvalidDatasetFile(caller(), "Invalid 'aid' attribute in 'location' element!", locNode->get_line());
		}
	}

	for (const Node *mvNode : movements)	{
		const Element *mvElement = castToElement(mvNode);
		auto sit = mAidToActivity.find(stoul(getAttribute(mvElement, "aid")));
		if (sit != mAidToActivity.cend())	{
			DynamicActivity* da = dynamic_cast<DynamicActivity*>(sit->second);
			if (da != nullptr)	{
				collisionPair.emplace_back(da->findMovement(stoul(getAttribute(mvElement, "mid"))));
			} else {
				string msg = "Referenced 'aid' does not correspond to the dynamic activity for 'movement' element!";
				throw InvalidDatasetFile(caller(), msg, mvNode->get_line());
			}
		} else {
			throw InvalidDatasetFile(caller(), "Invalid 'aid' attribute in 'movement' element!", mvNode->get_line());
		}
	}

	if (collisionPair.size() != 2)
		throw InvalidDatasetFile(caller(), "Invalid number of child elements in 'collision-pair' element!");

	return { collisionPair.front(), collisionPair.back() };
}

const Element* XmlReader::castToElement(const Node *node) const	{
	const Element *element = dynamic_cast<const Element*>(node);
	if (element == nullptr)
		throw InvalidDatasetFile(caller(), "Invalid cast to the 'Element*' type!", (node != nullptr ? node->get_line() : -1));
	return element;
}

string XmlReader::getAttribute(const Element* node, const string& attributeName, bool required) const {
	Attribute *attr = node->get_attribute(attributeName);
	if (attr == nullptr && required)	{
		string msg = "Cannot get '"+attributeName+"' from the '"+string(node->get_name())+"' element!";
		throw InvalidDatasetFile(caller(), "Failed to get the attribute from the element!", node->get_line());
	}

	if (attr != nullptr)
		return attr->get_value();
	else
		return "";
}

string XmlReader::getTextFromElement(const Element* elementNode, const string& childName, bool required) const	{
	string text;
	const Node::NodeList children = elementNode->get_children(childName);
	if (children.size() == 1)	{
		const Element *elementChild = castToElement(children.front());
		for (const Node *child : elementChild->get_children())	{
			const TextNode* textNode = dynamic_cast<const TextNode*>(child);
			if (textNode != nullptr)	{
				if (!textNode->is_white_space())
					text += textNode->get_content();
			} else {
				throw InvalidDatasetFile(caller(), "Invalid cast to 'TextNode*' type!", (child != nullptr ? child->get_line() : -1));
			}
		}
	} else if (children.size() > 1)	{
		throw InvalidDatasetFile(caller(), "Only from one child the text can be retrieved!", elementNode->get_line());
	} else if (children.empty() && required)	{
		string msg = "Cannot find the '"+string(childName)+"' element!\n";
		throw InvalidDatasetFile(caller(), msg+"Invalid input xml file!", elementNode->get_line());
	}

	return text;
}


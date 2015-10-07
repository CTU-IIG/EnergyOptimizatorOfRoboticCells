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

#ifndef HLIDAC_PES_ROBOTIC_LINE_H
#define HLIDAC_PES_ROBOTIC_LINE_H

/*!
 * \file RoboticLine.h
 * \author Libor Bukata
 * \brief The file contains various classes devoted to abstract representation of the robotic cell.
 */

#include <cassert>
#include <algorithm>
#include <iostream>
#include <stdint.h>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <unordered_set>
#include <utility>
#include "SolverConfig.h"
#include "Shared/Exceptions.h"

class Activity;
class Location;
class DynamicActivity;
class StaticActivity;
class RobotPowerMode;
class LocationDependentPowerConsumption;
class Robot;
class RoboticLine;

/*!
 * A base class (wrapper) for the robot movement or robot location.
 * It is particularly useful for easier data representation of robot collisions.
 * \brief Either a movement or location.
 */
class ActivityMode {
	public:
		ActivityMode() { };
		virtual uint32_t id() const = 0;
		virtual Activity* baseParent() const = 0;
		virtual ~ActivityMode() { };
};

/*!
 * The abstract class incorporating the common properties of both types of the activity, i.e. StaticActivity and DynamicActivity.
 * A robot operation, e.g. assembling or welding, can be considered either as a StaticActivity (robot is stationary), or it can be further
 * decomposed into more dynamic and static activities, e.g. movements during welding, spot welding operations, to achieve higher accuracy of the model.
 * \brief The base class incorporating common properties of robot operations and movements.
 */
class Activity {
	public:
		Activity(const uint32_t& aid) : mAid(aid), mLastInCycle(false), mParent(nullptr) { }
		uint32_t aid() const { return mAid; }
		std::string name() const { return mName; }
		std::string description() const { return mDescription; }
		bool lastInCycle() const { return mLastInCycle; }
		std::vector<Activity*> successors() const { return mSuccessors; }
		std::vector<Activity*> predecessors() const { return mPredecessors; }
		double minAbsDuration() const { return mMinAbsDuration; }
		double maxAbsDuration() const { return mMaxAbsDuration; }
		virtual bool mandatory() const = 0;
		virtual bool optional() const = 0;
		virtual size_t numberOfModes() const = 0;
		virtual Robot* parent() const = 0;
		virtual ~Activity() { }
	protected:
		void name(const std::string& name) { mName = name; }
		void description(const std::string& description) { mDescription = description; }
		void lastInCycle(const bool& lastInCycle) { mLastInCycle = lastInCycle; }
		void addSuccessor(Activity* successor) { mSuccessors.push_back(successor); }
		void addPredecessor(Activity* predecessor) { mPredecessors.push_back(predecessor); }
		void minAbsDuration(const double& minAbsDuration) { mMinAbsDuration = minAbsDuration; }
		void maxAbsDuration(const double& maxAbsDuration) { mMaxAbsDuration = maxAbsDuration; }

		virtual void parent(Robot* parent) = 0;
		virtual void freeAllocatedMemory() = 0;

		//! Identification of the activity.
		uint32_t mAid;
		//! It determines whether the activity closes the robot cycle.
		bool mLastInCycle;
		//! Name of the activity.
		std::string mName;
		//! Description of the activity.
		std::string mDescription;
		//! Successors of the activity.
		std::vector<Activity*> mSuccessors;
		//! Predecessors of the activity.
		std::vector<Activity*> mPredecessors;
		//! Minimal possible duration of the activity.
		double mMinAbsDuration;
		//! Maximal possible duration of the activity.
		double mMaxAbsDuration;

		//! Pointer to the parent, i.e. a robot performing this activity.
		Robot *mParent;

		//! Allow the robot to set itself as a parent and free the memory of the activity.
		friend class Robot;
};

/*!
 * It corresponds to a part of the energy function, i.e.
 * \f$\mathrm{coeff}*d^{\mathrm{degree}}\f$ where \f$d\f$ is the duration of the movement.
 * \brief A part of the energy function of a movement.
 */
struct Monomial {
	int32_t degree;
	double coeff;
};

/*!
 * \brief The instance of the class corresponds to a robot movement between two coordinates.
 */
class Movement : public ActivityMode {
	public:
		Movement(const uint32_t& mid) : mMid(mid), mFrom(nullptr), mTo(nullptr), mParent(nullptr) { }
		virtual uint32_t id() const { return mMid; }
		uint32_t mid() const { return mMid; }
		Location* from() const { return mFrom; }
		Location* to() const { return mTo; }
		double minDuration() const { return mMinDuration; }
		double maxDuration() const { return mMaxDuration; }
		std::vector<Monomial> energyFunction() const { return mEnergyFunction; }
		/*!
		 * \param duration Duration of the movement.
		 * \return Energy consumption of the movement for the given duration.
		 */
		double energyConsumption(const double& duration) const;
		DynamicActivity* parent() const { return mParent; }
		virtual Activity* baseParent() const { return (Activity*) mParent; }
		virtual ~Movement() = default;
	private:
		void from(Location* from) { mFrom = from; }
		void to(Location* to) { mTo = to; }
		void minDuration(const double& minDuration) { mMinDuration = minDuration; }
		void maxDuration(const double& maxDuration) { mMaxDuration = maxDuration; }
		void addMonomial(const Monomial& m) { mEnergyFunction.push_back(m); }
		void parent(DynamicActivity* parent) { mParent = parent; }

		//! %Movement identification.
		uint32_t mMid;
		//! %Location representing the start coordinate of the movement.
		Location *mFrom;
		//! %Location representing the end coordinate of the movement.
		Location *mTo;
		//! The minimal possible duration of the robot movement.
		double mMinDuration;
		//! The maximal duration of the movement.
		double mMaxDuration;
		/*!
		 * Energy function, the relation between the duration and required energy for the movement,
		 * is modeled as a sum of monomials stored in \a mEnergyFunction vector.
		 */
		std::vector<Monomial> mEnergyFunction;

		//! Parent of the movement, i.e. a dynamic activity.
		DynamicActivity* mParent;

		//! Enable an instance of %XmlReader class to set member variables.
		friend class XmlReader;
		//! Enable the parent to set a pointer to him.
		friend class DynamicActivity;
		//! Enable a robot to establish relations between activities, i.e. to set \a mFrom and \a mTo members.
		friend class Robot;
};

/*!
 * The instance of the class corresponds to a robot configuration, in which the robot
 * can perform an operation on the workpiece/weldment. Even though the class is called Location,
 * it represents not only the absolute coordinates of the operation but also orientation of the gun/gripper of the robot.
 * \brief %Location of the robot used either during work (welding) or waiting.
 */
class Location : public ActivityMode {
	public:
		Location(const uint32_t& lid = 0) : mLid(lid), mParent(nullptr) { }
		virtual uint32_t id() const { return mLid; }
		uint32_t lid() const { return mLid; }
		uint32_t point() const { return mPoint; }
		std::vector<LocationDependentPowerConsumption> locationDependentPowerConsumption() const {
			return mLocationDependentPowerConsumption;
		}
		std::vector<Movement*> enteringMovements() const { return mEnteringMovements; }
		std::vector<Movement*> leavingMovements() const { return mLeavingMovements; }
		/*!
		 * \param m Power saving mode of the robot.
		 * \return Input power of the robot.
		 */
		double inputPower(RobotPowerMode* m) const;
		/*!
		 * \param duration How long the robot is stationary.
		 * \param m Power saving mode of the robot.
		 * \return Energy consumption of the stationary robot for the given duration.
		 */
		double energyConsumption(const double& duration, RobotPowerMode* m) const;
		StaticActivity* parent() const { return mParent; }
		virtual Activity* baseParent() const { return (Activity*) mParent; }
		virtual ~Location() = default;
	private:
		void lid(const uint32_t& lid) { mLid = lid; }
		void point(const uint32_t& point) { mPoint = point; }
		void addLocationDependentPowerConsumption(const LocationDependentPowerConsumption& ldpc) {
			mLocationDependentPowerConsumption.push_back(ldpc);
		}
		void enteringMovements(const std::vector<Movement*>& enteringMovements) { mEnteringMovements = enteringMovements; }
		void leavingMovements(const std::vector<Movement*>& leavingMovements) { mLeavingMovements = leavingMovements; }
		void parent(StaticActivity *parent);

		//! Identification of this location.
		uint32_t mLid;
		/*!
		 * A unique number representing a coordinate of the location.
		 * The optimization problem does not need exact coordinates
		 * since they are already incorporated in the energy functions.
		 */
		uint32_t mPoint;
		/*!
		 * The power consumption of a stationary robot may also depend on its position,
		 * e.g. if the robot is held by its motors, and therefore, the vector \a mLocationDependentPowerConsumption
		 * defines the input power of the robot for these specific cases.
		 */
		std::vector<LocationDependentPowerConsumption> mLocationDependentPowerConsumption;
		//! The movements entering to this location.
		std::vector<Movement*> mEnteringMovements;
		//! The movements leaving from this location.
		std::vector<Movement*> mLeavingMovements;

		//! A parent of the location, i.e. static activity.
		StaticActivity *mParent;

		//! Enable an instance of %XmlReader class to set member variables.
		friend class XmlReader;
		//! Enable a static activity to declare its relation to the location.
		friend class StaticActivity;
};

/*!
 * The dynamic activity represents a collection of possible movements between two static activities.
 * In the final solution, at most one movement can be selected for each dynamic activity due to the alternatives,
 * i.e. the possibility to select from multiple process plans - activity orders.
 * \brief Collection of movements between two static activities.
 */
class DynamicActivity: public Activity {
	public:
		DynamicActivity(const uint32_t& aid) : Activity(aid) { }
		std::vector<Movement*> movements() const { return mMovements; }
		/*!
		 * \param mid Identification of a movement.
		 * \return A pointer to the movement with \a mid identification.
		 * \brief It finds a movement according to its identification.
		 * \see StaticActivity::findLocation
		 */
		Movement* findMovement(const uint32_t& mid) const;
		StaticActivity* predecessor() const { return cessor("prede"); }
		StaticActivity* successor() const { return cessor("suc"); }
		//! It returns whether this dynamic activity has to be performed.
		virtual bool mandatory() const;
		//! It returns whether this dynamic activity is optional, i.e. may or may not be performed.
		virtual bool optional() const { return !mandatory(); }
		virtual size_t numberOfModes() const { return mMovements.size(); }
		virtual Robot* parent() const { return mParent; }
		virtual ~DynamicActivity() = default;
	private:
		void addMovement(Movement* mv) { mMovements.push_back(mv); }
		virtual void parent(Robot* parent);
		/*!
		 * \param prefix Either "prede" or "suc".
		 * \return Successor or predecessor of this activity depending on \a prefix parameter.
		 */
		StaticActivity* cessor(const std::string& prefix) const;
		virtual void freeAllocatedMemory();

		//! The movements of this dynamic activity.
		std::vector<Movement*> mMovements;

		//! Enable an instance of %XmlReader class to add movements.
		friend class XmlReader;
};

/*!
 * The static activity represents a collection of locations, i.e. robot configurations,
 * in which some operation (or waiting) such as e.g. welding, cutting, or assembling can be performed.
 * In the final solution, only one location is selected for each static activity.
 * \brief Collection of locations in which a robot operation (or waiting) can be performed.
 */
class StaticActivity : public Activity {
	public:
		StaticActivity(const uint32_t& aid) : Activity(aid) { }
		std::vector<Location*> locations() const { return mLocations; }
		/*!
		 * \param lid Identification of a location.
		 * \return A pointer to the location with \a lid identification.
		 * \brief It finds a location according to its identification.
		 * \see DynamicActivity::findMovement
		 */
		Location* findLocation(const uint32_t& lid) const;
		/*!
		 * \param lid Identification of a location belonging to this activity.
		 * \param pid Identification of the power saving mode of the robot \a Activity::mParent.
		 * \return Input power of the robot for the given location and power saving mode.
		 */
		double inputPower(const uint32_t& lid, const uint32_t& pid) const;
		/*!
		 * \param duration Requested duration of this static activity.
		 * \param lid Identification of a location belonging to this activity.
		 * \param pid Identification of the power saving mode of the robot \a Activity::mParent.
		 * \return Calculated energy consumption of this activity for given parameters.
		 */
		double energyConsumption(double duration, const uint32_t& lid, const uint32_t& pid) const;
		//! Each static activity is mandatory since it has to be performed.
		virtual bool mandatory() const { return true; }
		//! None of static activities is optional since all of them have to be performed.
		virtual bool optional() const { return false; }
		virtual size_t numberOfModes() const { return mLocations.size(); }
		virtual Robot* parent() const { return mParent; }
		virtual ~StaticActivity() = default;
	private:
		void addLocation(Location* p) { mLocations.push_back(p); }
		virtual void parent(Robot* parent);
		//! Method assigns entering and leaving movements to each location in \a mLocations.
		void findMovementsForLocations();
		virtual void freeAllocatedMemory();

		//! Each static activity consists of possible robot positions, i.e. locations.
		std::vector<Location*> mLocations;

		//! Enable an instance of %XmlReader class to add locations.
		friend class XmlReader;
		//! Enable the parent robot to call \ref findMovementsForLocations method.
		friend class Robot;
		//! It allows the parallel heuristic to directly access \ref mLocations array to avoid copying.
		friend class ParallelHeuristicSolver;
};

/*!
 * The class represents the power saving mode of the robot, which can be applied if the robot is in a stationary position.
 * For example, the stationary robot can usually be held by electric motors (dummy power saving mode) or brakes.
 * However, even deeper power saving modes such as bus-power-off and hibernate are possible for e.g. KUKA robots.
 * \brief It represents the power saving mode of the robot.
 */
class RobotPowerMode {
	public:
		RobotPowerMode(const uint32_t& pid) : mPid(pid), mExpectedInputPower(-1), mParent(nullptr) { }
		uint32_t pid() const { return mPid; }
		std::string name() const { return mName; }
		std::string description() const { return mDescription; }
		double minimalDelay() const { return mMinimalDelay; }
		double expectedInputPower() const { return mExpectedInputPower; }
		Robot* parent() const { return mParent; }
	private:
		void pid(const uint32_t& pid) { mPid = pid; }
		void name(const std::string& name) { mName = name; }
		void description(const std::string& description) { mDescription = description; }
		void minimalDelay(const double& delay) { mMinimalDelay = delay; }
		void expectedInputPower(const double& expectedInputPower) { mExpectedInputPower = expectedInputPower; }
		void parent(Robot *parent) { mParent = parent; }

		//! Identification of the power saving mode.
		uint32_t mPid;
		//! Name of this power saving mode, e.g. 'brakes', 'motors', 'bus-power-off', ...
		std::string mName;
		//! Description of this power saving mode.
		std::string mDescription;
		//! Minimal time needed for an application of this power saving mode.
		double mMinimalDelay;
		//! Expected input power of the robot (see \a mParent) for this power saving mode.
		double mExpectedInputPower;

		//! A robot that is equipped with this power saving mode.
		Robot *mParent;

		//! Enable an instance of %XmlReader to fill the data.
		friend class XmlReader;
		//! It allows the robot to set itself as a parent.
		friend class Robot;
};

/*!
 * Input power of the robot is also dependent on the robot configuration for some power saving modes, for example,
 * if the robot is held by motors then the power consumption is different for the robot being stretched out or robot at home position.
 * This class can be perceived as an extension to \a RobotPowerMode class as it adds the information
 * about input power for a particular robot configuration, i.e. location (see also \a Location class).
 * \brief The instance of this class specifies input power of the robot for a particular robot configuration and power saving mode.
 */
class LocationDependentPowerConsumption {
	public:
		LocationDependentPowerConsumption(const RobotPowerMode * rpm) : mPowerMode(rpm), mParent(nullptr) { }
		const RobotPowerMode* robotPowerMode() const { return mPowerMode; }
		double inputPower() const { return mInputPower; }
		Location *parent() const { return mParent; }
	private:
		void inputPower(const double& inputPower) { mInputPower = inputPower; }
		void parent(Location *parent) { mParent = parent; }

		//! The power saving mode which input power is location dependent.
		const RobotPowerMode* mPowerMode;
		//! %Location dependent input power.
		double mInputPower;

		//! Parent of this class is the related location for which the input power is calculated.
		Location *mParent;

		//! Enable an instance of %XmlReader to call setters.
		friend class XmlReader;
		//! It allows the location to set itself as a parent.
		friend class Location;
};

/*!
 * An instance of this class corresponds to a robot with its operations and movements.
 * The order of activities stems from the precedences between activities.
 * \brief Instance of this class includes all the data structures and methods related to a robot.
 */
class Robot {
	public:
		Robot() : mParent(nullptr) { }
		std::string name() const { return mName; }
		std::string description() const { return mDescription; }
		std::vector<Activity*> activities() const { return mActivities; }
		std::vector<RobotPowerMode*> powerModes() const { return mRobotModes; }
		//! The power saving mode with the minimal time for an application is returned.
		RobotPowerMode* fastestPowerSavingMode() const;
		RoboticLine* parent() const { return mParent; }
	private:
		void name(const std::string& name) { mName = name; }
		void description(const std::string& description) { mDescription = description; }
		void addActivity(Activity *a) { mActivities.push_back(a); }
		void addPowerMode(RobotPowerMode* m) { mRobotModes.push_back(m); }
		void parent(RoboticLine* parent);
		/*!
		 * \param pointToLocation Mapping between coordinate identifications and locations.
		 * \param movementToPoints Mapping of movements to their start and end coordinate identifications.
		 * \brief It initializes precedences between activities (successors, predecessors).
		 */
		void setActivitiesRelations(const std::map<uint32_t, Location*>& pointToLocation,
				const std::map<Movement*, std::pair<uint32_t, uint32_t>>& movementToPoints);
		void freeAllocatedMemory();

		//! Name of the robot.
		std::string mName;
		//! Description of the robot.
		std::string mDescription;
		//! Static (operations, waiting) and dynamic (movements) activities of the robot.
		std::vector<Activity*> mActivities;
		//! Applicable power saving modes for this robot.
		std::vector<RobotPowerMode*> mRobotModes;

		//! Robotic cell in which this robot is located.
		RoboticLine* mParent;

		//! Enable an instance of %XmlReader to call private methods and setters.
		friend class XmlReader;
		//! Full access of the related robotic cell is required for initialization.
		friend class RoboticLine;
};

/*!
 * The time synchronization of robots is accomplished by using time lags.
 * Multiple instances of this class can model e.g. a workpiece/weldment
 * passing, which can be carried out either directly, i.e. gripper-to-gripper, or by using the bench.
 * \brief Instance of TimeLag class defines a time relation between two different robots.
 */
class TimeLag {
	public:
		/*!
		 * \param f The activity from which the arc is leaving.
		 * \param t The activity to which the arc is entering.
		 * \param l The time offset of the inter-robot arc.
		 * \param h It indexes the previous or current cycles of a related robot.
		 * \brief It constructs an instance corresponding to \f$s_t \geq s_f+l-\mathrm{CT} h\f$ equation
		 * where \f$s_t\f$ and \f$s_f\f$ are start times of activity \a t and \a f respectively, and \f$\mathrm{CT}\f$ is the production cycle time.
		 */
		TimeLag(Activity* f, Activity* t, double l, int32_t h) : mFrom(f), mTo(t), mLength(l), mHeight(h) { }
		Activity* from() const { return mFrom; }
		Activity* to() const { return mTo; }
		double length() const { return mLength; }
		int32_t height() const { return mHeight; }
	private:
		void from(Activity* from) { mFrom = from; }
		void to(Activity* to) { mTo = to; }
		void length(const double& length) { mLength = length; }
		void height(const int32_t& height) { mHeight = height; }

		//! The activity from which the arc is leaving.
		Activity *mFrom;
		//! The activity to which the arc is entering.
		Activity *mTo;
		//! The time offset of the inter-robot arc.
		double mLength;
		//! The time offset in the number of cycles.
		int32_t mHeight;
};

/*!
 * The class corresponds to the inter-robot operation, e.g. a workpiece/weldment passing.
 * It incorporates time lags and spatial compatibility pairs to guarantee that
 * a cooperation takes place in the right location at the right time.
 * \brief The inter-robot operation corresponding to the workpiece/weldment handling.
 */
class InterRobotOperation {
	public:
		InterRobotOperation(const uint32_t& oid) : mOid(oid), mParent(nullptr) { }
		uint32_t oid() const { return mOid; }
		std::string name() const { return mName; }
		std::string description() const { return mDescription; }
		std::vector<TimeLag> timeLags() const { return mTimeLags; }
		std::vector<std::pair<Location*, Location*>> spatialCompatibility() const { return mSpatialCompatibility; }
		RoboticLine* parent() const { return mParent; }
	private:
		void name(const std::string& name) { mName = name; }
		void description(const std::string& description) { mDescription = description; }
		void addTimeLag(const TimeLag& lag) { mTimeLags.push_back(lag); }
		void addSpatialCompatibilityPair(Location *l1, Location *l2) { mSpatialCompatibility.emplace_back(l1, l2); }
		void parent(RoboticLine* parent) { mParent = parent; }

		//! Identification of this inter-robot operation.
		uint32_t mOid;
		//! Name of this inter-robot operation.
		std::string mName;
		//! Description of this inter-robot operation.
		std::string mDescription;
		//! Time lags ensuring desired time synchronizations.
		std::vector<TimeLag> mTimeLags;
		/*!
		 * \brief List of compatible location pairs for the handover(s).
		 * \note If the vector is empty then all the possible pairs are allowed.
		 */
		std::vector<std::pair<Location*, Location*>> mSpatialCompatibility;

		//! A parent of the inter-robot operation is the robotic cell.
		RoboticLine* mParent;

		//! An instance of the %XmlReader class calls the private setters.
		friend class XmlReader;
		//! The robotic cell is allowed to set itself as a parent.
		friend class RoboticLine;
};

/*!
 * The instance of this class corresponds to the whole robotic cell including
 * its robots, operations, synchronizations, and collisions.
 * \brief The robotic cell corresponds to an instance of this class.
 * \note The robotic cell is immutable (read-only) after it has been filled.
 */
class RoboticLine {
	public:
		RoboticLine();
		RoboticLine(const RoboticLine& l);
		RoboticLine& operator=(const RoboticLine& l);
		std::string name() const { return mName; }
		std::string description() const { return mDescription; }
		std::vector<Robot*> robots() const { return mRobots; }
		std::vector<InterRobotOperation*> interRobotOperations() const { return mInterRobotOperations; }
		std::vector<std::pair<ActivityMode*, ActivityMode*>> collisions() const { return mCollisions; }
		double productionCycleTime() const { return mProductionCycleTime; }
		~RoboticLine();
	private:
		void name(const std::string& name) { mName = name; }
		void description(const std::string& description) { mDescription = description; }
		void addRobot(Robot* r) { mRobots.push_back(r); }
		void addInterRobotOperation(InterRobotOperation* op) { mInterRobotOperations.push_back(op); }
		void addCollision(const std::pair<ActivityMode*, ActivityMode*>& collision) { mCollisions.push_back(collision); }
		void productionCycleTime(const double& productionCycleTime) { mProductionCycleTime = productionCycleTime; }

		/*!
		 * \param rl Instance of this class.
		 * \brief This method is propagated through all the parts of the robotic cell in order to set parents.
		 */
		void setParentOfChildren(RoboticLine *rl);
		/*!
		 * \param pointToLocation Mapping between coordinate identifications and locations.
		 * \param movementToPoints Mapping of movements to their start and end coordinate identifications.
		 * \brief It recursively sets parents and initializes relations between activities.
		 */
		void initialiseDataStructures(const std::map<uint32_t, Location*>& pointToLocation,
				const std::map<Movement*, std::pair<uint32_t, uint32_t>>& movementToPoints);
		void copyDataStructures(const RoboticLine& r);
		void freeAllocatedMemory();

		//! Counter of references, the number of shallow copies sharing the same data.
		int64_t *mRefCounter;
		//! Name of the robotic cell.
		std::string mName;
		//! Description of the robotic cell.
		std::string mDescription;
		//! Robots located in this cell.
		std::vector<Robot*> mRobots;
		//! Inter-robot operations between robots.
		std::vector<InterRobotOperation*> mInterRobotOperations;
		//! Collisions between robots defined as time disjunctive pairs.
		std::vector<std::pair<ActivityMode*, ActivityMode*>> mCollisions;
		//! Production cycle time, also called robot cycle time.
		double mProductionCycleTime;

		//! Track of dynamically created instances, needed for managing the memory.
		static std::map<int64_t*, std::set<RoboticLine*>> liveInstances;

		//! %XmlReader should be allowed to fill the data-structures of the robotic cell.
		friend class XmlReader;
};

#endif

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

#include <fstream>
#include <string>
#include <vector>
#include "SolverConfig.h"
#include "Solution/Solution.h"
#include "Shared/Exceptions.h"
#include "Shared/PrecalculatedMapping.h"
#include "Shared/Utils.h"

using namespace std;

State convert(const Status& ilpStatus)	{
	State solStatus;
	switch (ilpStatus)	{
		case ILP_OPTIMAL:
			solStatus = OPTIMAL;
			break;
		case ILP_FEASIBLE:
			solStatus = FEASIBLE;
			break;
		case ILP_INFEASIBLE:
			solStatus = INFEASIBLE;
			break;
		default:
			solStatus = UNKNOWN;
	}

	return solStatus;
}

ostream& operator<<(ostream& out, const Solution& s)	{
	vector<string> toPrint;
	if (s.status == OPTIMAL || s.status == FEASIBLE)	{
		toPrint.emplace_back("Solution is "+string(s.status == OPTIMAL ? "optimal" : "feasible")+".");
		toPrint.emplace_back("Total energy consumed: "+to_string(s.totalEnergy)+" J");
		if (s.lowerBound > 0.0)
			toPrint.emplace_back("Lower bound on energy: "+to_string(s.lowerBound)+" J\n");
		else
			toPrint.emplace_back("");

		for (const auto& mit : s.startTimeAndDuration)	{
			string line = "activity "+to_string(mit.first)+": ";
			line += to_string(mit.second.first)+"+"+to_string(mit.second.second);
			const auto& sit = s.pointAndPowerMode.find(mit.first);
			if (sit != s.pointAndPowerMode.cend())
				line += " (point="+to_string(sit->second.first)+", mode="+to_string(sit->second.second)+")";
			toPrint.push_back(move(line));
		}

		if (s.runTime >= 0.0 || s.memUsage >= 0.0)	{
			toPrint.push_back("");
			if (s.runTime >= 0.0)
				toPrint.push_back("Run time: "+to_string(s.runTime)+" s");
			if (s.memUsage >= 0.0)
				toPrint.push_back("Memory usage: "+to_string(s.memUsage)+" Bytes");
		}
	} else {
		if (s.status == INFEASIBLE)
			toPrint.push_back("No feasible solution exists!");
		else
			toPrint.push_back("Solution not found!");
	}

	for (uint64_t l = 0; l < toPrint.size(); ++l)	{
		if (l+1 < toPrint.size())
			out<<toPrint[l]<<endl;
		else
			out<<toPrint[l];
	}

	return out;
}

ostream& operator<<(ostream& out, const State& st)	{
	switch (st)	{
		case OPTIMAL:
			out<<"optimal";
			break;
		case FEASIBLE:
			out<<"feasible";
			break;
		case INFEASIBLE:
			out<<"infeasible";
			break;
		default:
			out<<"unknown";
	}

	return out;
}

void writeSolutionToCsvFile(const string& file, const Solution& s, const PrecalculatedMapping& mapper)	{
	ofstream out(file.c_str(), ofstream::out | ofstream::trunc);
	if (!out)
		throw SolverException(caller(), "Cannot write to "+file+" file! Please check permissions.");

	if (s.status == OPTIMAL || s.status == FEASIBLE)	{
		out<<"activity id, start time, duration, type, point, power saving mode, movement, energy"<<endl;
		for (const auto& mit : s.startTimeAndDuration)	{
			uint32_t aid = mit.first;
			Activity* a = getValue(mapper.aidToActivity, aid, caller());
			double start = mit.second.first, duration = mit.second.second;

			out<<aid<<", "<<start<<", "<<duration<<", ";

			StaticActivity* sa = dynamic_cast<StaticActivity*>(a);
			if (sa != nullptr)	{
				Location *loc = nullptr;
				uint32_t point, powerMode;
				try {
					const auto p = getValue(s.pointAndPowerMode, aid, caller());
					point = p.first; powerMode = p.second;
					loc = getValue(mapper.pointToLocation, point, caller());
				} catch (...)	{
					string msg = "Incomplete solution - cannot find selected point and power saving mode for activity "+to_string(aid)+"!";
					throw_with_nested(SolverException(caller(), msg));
				}

				out<<"STATIC, "<<point<<", "<<powerMode<<", NA, "<<sa->energyConsumption(duration, loc->lid(), powerMode)<<endl;
			}

			DynamicActivity* da = dynamic_cast<DynamicActivity*>(a);
			if (da != nullptr)	{
				Movement *mv = getSelectedMovement(s, mapper, da);
				out<<"DYNAMIC, NA, NA, "<<mv->mid()<<", "<<mv->energyConsumption(duration)<<endl;
			}
		}
	} else {
		out<<(s.status == INFEASIBLE ? "No feasible solution exists!" : "Solution not found!")<<endl;
	}

	out.close();
}

void writeBriefResultToStream(std::ostream& OUT, const string& instanceName, const Solution& s)	{
	OUT<<instanceName<<" ("<<s.status<<", "<<s.runTime<<" s): ";
	if (s.status == FEASIBLE || s.status == OPTIMAL)	{
		if (s.lowerBound > 0.0)
			OUT<<"from "<<s.lowerBound<<" to "<<s.totalEnergy<<endl;
		else
			OUT<<s.totalEnergy<<endl;
	} else {
		OUT<<"?"<<endl;
	}
}

void writeBriefResultToCsvFile(const std::string& instanceName, const std::string& pathToFile, const Solution& s)	{
	bool writeHeader = !fileExists(pathToFile);
	ofstream resFile(pathToFile.c_str(), ofstream::app);
	if (resFile.good())	{
		if (writeHeader)
			resFile<<"instance name, status, lower bound on energy, total energy, runtime [s]"<<endl;

		resFile<<instanceName<<", "<<s.status<<", ";
		if (s.status == FEASIBLE || s.status == OPTIMAL)
			resFile<<s.lowerBound<<", "<<s.totalEnergy<<", ";
		else
			resFile<<"NA, NA, ";

		resFile<<s.runTime<<endl;
	} else {
		cerr<<"Warning: Cannot open '"<<pathToFile<<"' file for writing of results!"<<endl;
	}

	resFile.close();
}


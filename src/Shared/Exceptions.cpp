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

#include <iostream>
#include <sstream>
#include "Shared/Exceptions.h"

using namespace std;

vector<string> split(const std::string& toSplit, const char& delim)	{
	string r;
	vector<string> splitted;
	stringstream ss(toSplit);
	while (getline(ss, r, delim))
		splitted.push_back(r);

	return splitted;
}

string concat(const vector<string>& toConcat, const std::string& beforeItem, const std::string& afterItem)	{
	string result;
	for (const string& item : toConcat)	{
		result += beforeItem;
		result += item;
		result += afterItem;
	}

	size_t pos = result.find_last_of(afterItem);
	if (pos != string::npos)
		result = result.substr(0, pos);	// remove last afterItem

	return result;
}

string exceptionToString(const std::exception& e, uint32_t level)	{
	string msg;
	if (level == 0)
		msg += string(100, '!')+'\n';

	try {
		try {
			const SolverException& se = dynamic_cast<const SolverException&>(e);
			msg += se.whatIndented(level);
			msg += '\n';
		} catch (...)	{
			vector<string> lines = split(e.what(), '\n');
			for (const string& line : lines)
				msg += string(level, '\t')+line+'\n';
		}

		rethrow_if_nested(e);
	} catch (const exception& e)	{
		msg += '\n';
		msg += exceptionToString(e, level+1);
	}

	if (level == 0)
		msg += string(100, '!');

	return msg;
}


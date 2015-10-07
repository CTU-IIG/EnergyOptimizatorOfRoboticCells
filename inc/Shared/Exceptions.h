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

#ifndef HLIDAC_PES_EXCEPTIONS_H
#define HLIDAC_PES_EXCEPTIONS_H

/*!
 * \file Exceptions.h
 * \author Libor Bukata
 * \brief The file defines extended exceptions for the better error handling in the program.
 */

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>
#include <stdint.h>

/*!
 * \param toSplit String to be split.
 * \param delim Delimiter used for splitting.
 * \return Vector of strings split by the delimiter.
 * \brief It splits the input string, e.g. split("abc ab c", ' ') -> {"abc", "ab", "c"}.
 */
std::vector<std::string> split(const std::string& toSplit, const char& delim);

/*!
 * \param toConcat Vector of strings to concatenate.
 * \param beforeItem String prepended to each string in \a toConcat.
 * \param afterItem String appended to each string in \a toConcat with exception of the last one.
 * \return Concatenation of the strings including additional parts.
 * \brief Method concatenates the given strings, e.g. concat({"abc", "ab"}, "\t", "\n") -> "\tabc\n\tab".
 */
std::string concat(const std::vector<std::string>& toConcat, const std::string& beforeItem, const std::string& afterItem);


/*!
 * A general exception of the program supporting formatted output.
 * More specialized exceptions are derived from this class.
 * \brief A general exception of the program.
 * \see InvalidDatasetFile, InvalidArgument, ILPSolverException, NoFeasibleSolutionExists, EmptySolutionPool
 */
class SolverException : public std::exception {
	public:
		/*!
		 * \param caller String representation of the method in which this exception was thrown.
		 * \param msg Error message. Messages with multiple lines are supported.
		 * \brief Constructs the exception, it comprises building of the error string and vector of lines.
		 * \note To ease the filling of the caller, an ugly but incredibly useful macro \a caller() is available.
		 */
		SolverException(const std::string& caller, const std::string& msg) {
			std::vector<std::string> splitCaller = split(caller, '\n'), splitMsg = split(msg, '\n');
			std::copy(splitCaller.cbegin(), splitCaller.cend(), std::back_inserter(mWhat));
			std::copy(splitMsg.cbegin(), splitMsg.cend(), std::back_inserter(mWhat));
			mWhatStr = concat(mWhat, "", "\n");
		}

		//! Virtual method returns the error message without an indentation.
		virtual const char* what() const throw() { return mWhatStr.c_str(); }
		/*!
		 * \param level The number of prepended tabulators.
		 * \return Formated string with the error message.
		 * \brief Virtual function returning the formatted error message.
		 */
		virtual std::string whatIndented(uint32_t level = 0) const throw() {
			return concat(mWhat, std::string(level, '\t'), "\n");
		}

		//! Virtual destructor is mandatory due to the polymorphism.
		virtual ~SolverException() throw() { }
	protected:
		//! The formated error message without an indentation.
		std::string mWhatStr;
		//! Individual lines of the error message.
		std::vector<std::string> mWhat;
};

/*!
 * This exception is typically thrown if a dataset file is invalid, i.e.
 * invalid format of the XML file or incorrectly specified robotic cells.
 * The exception adds the possibility to state the line number in the XML file where the error/inconsistency occurs.
 * \brief Thrown if the dataset file contains ill-specified robotic cells.
 */
class InvalidDatasetFile : public SolverException {
	public:
		/*!
		 * \param caller String representation of the method in which this exception was thrown.
		 * \param msg Explanation of a problem in the dataset.
		 * \param lineNumber At which line of XML file the problem occurs.
		 * \brief Constructs a specialized exception for error handling of ill-specified datasets.
		 * \note To ease the filling of the caller, an ugly but incredibly useful macro \a caller() is available.
		 */
		InvalidDatasetFile(const std::string& caller, const std::string& msg, int64_t lineNumber = -1) : SolverException(caller, msg), mLineNumber(lineNumber) {
			if (mLineNumber != -1)
				mWhat.push_back("The error occured around the line "+std::to_string(mLineNumber)+" in the input dataset file!");
		}

		virtual ~InvalidDatasetFile() throw() { }
	protected:
		//! Error occurred at this line in the XML file.
		int64_t mLineNumber;
};

//! Exception is thrown if a method is given invalid parameters or a user provides invalid program arguments.
class InvalidArgument : public SolverException {
	public:
		InvalidArgument(const std::string& caller, const std::string& msg) : SolverException(caller, msg) { }
		virtual ~InvalidArgument() throw() { }
};

//! Exception dedicated to problems with Integer Linear Programming solvers.
class ILPSolverException : public SolverException {
	public:
		ILPSolverException(const std::string& caller, const std::string& msg) : SolverException(caller, msg) { }
		virtual ~ILPSolverException() throw() { }
};

//! Thrown if no feasible solution is found by the heuristic. \see ParallelHeuristicSolver
class NoFeasibleSolutionExists : public SolverException {
	public:
		NoFeasibleSolutionExists(const std::string& caller, const std::string& msg) : SolverException(caller, msg) { }
		virtual ~NoFeasibleSolutionExists() throw() { }
};

//! Thrown if the best solution of the heuristic cannot be returned since the solution pool is empty. \see KnowledgeBase
class EmptySolutionPool : public SolverException {
	public:
		EmptySolutionPool(const std::string& caller, const std::string& msg) : SolverException(caller, msg) { }
		virtual ~EmptySolutionPool() throw() { }
};

/*!
 * \param e Exception to be printed to string.
 * \param level Current level of indentation.
 * \return A fully formatted error message for the given exception, including nested exceptions.
 * \brief The recursive method creates the formatted error message for the given exception and their nested sub-exceptions.
 */
std::string exceptionToString(const std::exception& e, uint32_t level = 0);

#endif

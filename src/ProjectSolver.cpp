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

/*!
 * \file ProjectSolver.cpp
 * \author Libor Bukata
 * \brief Entry point of the program. Auxiliary functions for program help and argument processing.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "Settings.h"
#include "SolverConfig.h"
#include "Shared/Exceptions.h"
#include "InstancesReader/InstancesReader.h"
#include "Solution/Solution.h"
#include "Solution/SolutionChecker.h"
#include "ILPModel/SolverInterface.h"
#include "ILPSolver/RoboticLineSolverILP.h"
#include "HeuristicSolver/ParallelHeuristicSolver.h"

using namespace std;

//! Return codes of the program with obvious meanings.
enum ProgramReturnedCodes {
	EXIT_WITH_SUCCESS = 0,
	INVALID_PARAMETER = 1,
	INPUT_OUTPUT_ERROR = 2,
	RUNTIME_ERROR = 4,
	UNKNOWN_ERROR = 8
};

/*!
 * \defgroup prog_help program help
 * \brief Functions informing the user about the program and its arguments.
 * \details \verbinclude Programhelp.txt
 * @{
 */

//! It prints a brief description of the program including the authors, license, ILP solver, and version.
void printProgramHeader() {
	cout<<"Energy optimizator of robotic cells."<<endl;
	cout<<"Authors: Libor Bukata and Premysl Sucha"<<endl;
	cout<<"Licence: GNU General Public License"<<endl;
	cout<<"Program version: "<<PROGRAM_VERSION<<endl;
	cout<<"ILP solver: "<<solverIdentification()<<endl;
	cout<<"Build type: "<<BUILD_TYPE<<endl;
}

/*!
 * \param progName The name of this program.
 * \brief It prints the program header and brief help.
 * \see printProgramHeader
 */
void printProgramHelp(const string& progName)	{
	printProgramHeader();
	cout<<endl<<"Usage:"<<endl;
	cout<<"\t"<<progName<<" [options] --dataset FILE"<<endl<<endl;
	cout<<"General options:"<<endl;
	cout<<"\t--dataset ARG, -d ARG, ARG: FILE"<<endl;
	cout<<"\t\tThe input dataset to solve (an xml file)."<<endl;
	cout<<"\t--verbose, -v"<<endl;
	cout<<"\t\tAn additional information is printed (solver progress, runtime, etc.)."<<endl;
	cout<<"\t--number-of-segments ARG, -nos ARG, ARG: INTEGER"<<endl;
	cout<<"\t\tThe number of segments of each discretized energy function."<<endl;
	cout<<"\t--number-of-threads ARG, -not ARG, ARG: INTEGER"<<endl;
	cout<<"\t\tThe number of concurrent threads (default is autodetect)."<<endl;
	cout<<"\t--help, -h"<<endl;
	cout<<"\t\tIt prints this help."<<endl;
	cout<<"\t--max-runtime ARG, -mr ARG, ARG: FLOAT"<<endl;
	cout<<"\t\t"<<"It sets the time limit per instance for a selected algorithm (in seconds)."<<endl;
	cout<<"\t--use-heuristic-algorithm, -uha"<<endl;
	cout<<"\t\t"<<"A heuristic algorithm is employed to solve instances."<<endl;
	cout<<"\t--use-exact-algorithm, -uea"<<endl;
	cout<<"\t\t"<<"An exact algorithm is preferred as a problem solver."<<endl;
	cout<<"\t--write-results ARG, -wr ARG, ARG: DIRECTORY"<<endl;
	cout<<"\t\tIt specifies where the solutions, error logs, and performance logs will be written."<<endl<<endl;
	cout<<"ILP solver options:"<<endl;
	cout<<"\t--ilp-solver-relative-gap ARG, -isrg ARG, ARG: DECIMAL"<<endl;
	cout<<"\t\tIt stops the solver after achieving the relative gap between the best integer solution and lower bound."<<endl;
	cout<<"\t\tSetting the gap to 0.05 means that solver stops after proving 5 % maximal gap from the optimal solution."<<endl;
	cout<<"\t--lower-bound-calculation, -lbc"<<endl;
	cout<<"\t\tIt turns on the calculation of a tighter lower bound by using a problem decomposition and an ILP solver."<<endl;
	cout<<"\t--lower-bound-runtime ARG, -lbr ARG, ARG: FLOAT"<<endl;
	cout<<"\t\tIt sets a time limit for the tight lower bound calculation."<<endl<<endl;
	cout<<"Heuristic options:"<<endl;
	cout<<"\t--number-of-elite-solutions ARG, -noes ARG, ARG: INTEGER"<<endl;
	cout<<"\t\tThe maximal number of elite solutions in the pool."<<endl;
	cout<<"\t--max-number-of-alternatives ARG, -mnoa ARG, ARG: INTEGER"<<endl;
	cout<<"\t\tThe maximal number of alternatives to consider for each robot."<<endl;
	cout<<"\t--minimal-number-of-iters-per-tuple ARG, -mnoipt ARG, ARG: INTEGER"<<endl;
	cout<<"\t\tThe minimal number of runs of sub-heuristics for each feasible solution. Sub-heuristics, i.e."<<endl;
	cout<<"\t\t(de)select power mode, change locations, and change path, are executed in the round robin order."<<endl<<endl;
	cout<<"Default settings can be modified at \"DefaultSettings.h\" file."<<endl;
}

//! @}

/*!
 * \defgroup arg_proc processing of program arguments
 * Various functions related to the processing of program arguments.
 * Passed arguments are reflected in the setting of the program.
 * \brief Functions related to the parsing of program arguments.
 * \see Settings
 * @{
 */

/*!
 * \param number A string containing only a positive number.
 * \tparam T Either unsigned integer or double.
 * \return A positive number parsed from the string.
 * \throw InvalidArgument String cannot be converted to a positive number.
 */
template <class T>
T parsePositiveNumber(const string& number)	{
	if (number.empty() || number[0] == '-')
		throw InvalidArgument(caller(), "Argument must be a positive number!");

	T readNumber = T();
	istringstream istr(number, istringstream::in);
	istr>>readNumber;
        if (!istr.eof())
		throw InvalidArgument(caller(), "Cannot parse the number!");

	return readNumber;
}

/*!
 * \param[in,out] i A reference to the index pointing to the current argument to be processed. The index may be updated.
 * \param argc The number of program arguments including the program name.
 * \param argv Arguments of the program.
 * \param fullName The full name of the argument, e.g. "--use-heuristic-algorithm".
 * \param shortName Abbreviated version of the full argument, e.g. "-uha".
 * \param[out] writeArgument String to which the parameter of the argument is written, e.g. for "--max-runtime 10.5" the string contains "10.5".
 * \return Whether the argument with the given name (short or full) was processed.
 * \brief It processes the arguments with one parameter.
 */
bool getArgument(int& i, int argc, char* argv[], const string& fullName, const string& shortName, string& writeArgument)	{
	string arg = argv[i];
	if (arg == fullName || arg == shortName)	{
		if (i+1 < argc)	{
			writeArgument = argv[++i];
			return true;
		} else	{
			throw InvalidArgument(caller(), "'"+fullName+"' requires the argument!");
		}
	}

	return false;
}

/*!
 * \param[in,out] i A reference to the index pointing to the current argument to be processed. The index may be updated.
 * \param argc The number of program arguments including the program name.
 * \param argv Arguments of the program.
 * \param fullName The full name of the argument, e.g. "--use-heuristic-algorithm".
 * \param shortName Abbreviated version of the full argument, e.g. "-uha".
 * \param[out] toWrite A pointer to a boolean variable that is set to true if the argument, e.g. "--verbose", is passed to the program.
 * \return Whether the argument with the given name (short or full) was processed.
 * \brief It processes the arguments without parameters.
 */
bool processArg(int& i, int argc, char* argv[], const string& fullName, const string& shortName, bool* toWrite = nullptr)	{
	string arg = argv[i];
	if (arg == fullName || arg == shortName)	{
		if (toWrite != nullptr)
			*toWrite = true;
		return true;
	} else {
		return false;
	}
}

/*!
 * \param[in,out] i A reference to the index pointing to the current argument to be processed. The index may be updated.
 * \param argc The number of program arguments including the program name.
 * \param argv Arguments of the program.
 * \param fullName The full name of the argument, e.g. "--use-heuristic-algorithm".
 * \param shortName Abbreviated version of the full argument, e.g. "-uha".
 * \param[out] toWrite String to which the parameter of the given argument is written.
 * \return Whether the argument with the given name (short or full) was processed.
 * \brief It processes the arguments with one string parameter.
 */
bool processArg(int& i, int argc, char* argv[], const string& fullName, const string& shortName, string& toWrite)	{
	return getArgument(i, argc, argv, fullName, shortName, toWrite);
}

/*!
 * \param[in,out] i A reference to the index pointing to the current argument to be processed. The index may be updated.
 * \param argc The number of program arguments including the program name.
 * \param argv Arguments of the program.
 * \param fullName The full name of the argument, e.g. "--use-heuristic-algorithm".
 * \param shortName Abbreviated version of the full argument, e.g. "-uha".
 * \tparam T Either positive integer or double.
 * \param[out] toWrite A reference to a variable to which the parameter of the given argument is written.
 * \param minValue The minimal allowed value of the parameter.
 * \return Whether the argument with the given name (short or full) was processed.
 * \brief It processes the arguments with one numerical parameter.
 */
template <class T>
bool processArg(int& i, int argc, char* argv[], const string& fullName, const string& shortName, T& toWrite, T minValue = 1)	{
	string strNumber;
	if (getArgument(i, argc, argv, fullName, shortName, strNumber))	{
		T number;
		try {
			number = parsePositiveNumber<T>(strNumber);
		} catch (...) {
			 throw_with_nested(InvalidArgument(caller(), "Cannot read the parameter of '"+fullName+"' option!"));
		}

		if (number >= minValue)	{
			toWrite = number;
			return true;
		} else	{
			throw InvalidArgument(caller(), "The '"+fullName+"' parameter needs to be a positive number!");
		}
	} else {
		return false;
	}
}

/*!
 * \param argc The number of program arguments (+parameters) passed to the program is argc-1.
 * \param argv Program name and arguments (+parameters) of the program.
 * \return Returns true if the program should exit, otherwise false.
 * \brief It parses the program arguments and sets the desired parameters for optimization.
 */
bool processCommandLineArguments(int argc, char* argv[])	{

	for (int i = 1; i < argc; ++i)	{
		/* GENERAL OPTIONS */
		if (processArg(i, argc, argv, "--dataset", "-d", Settings::DATASET_FILE)) continue;
		if (processArg(i, argc, argv, "--verbose", "-v", &Settings::VERBOSE)) continue;
		if (processArg(i, argc, argv, "--number-of-segments", "-nos", Settings::NUMBER_OF_SEGMENTS)) continue;
		if (processArg(i, argc, argv, "--number-of-threads", "-not", Settings::NUMBER_OF_THREADS)) continue;
		if (processArg(i, argc, argv, "--help", "-h") == true)	{
			printProgramHelp(argv[0]);
			return true;
		}
		if (processArg(i, argc, argv, "--max-runtime", "-mr", Settings::MAX_RUNTIME, 0.0)) continue;
		if (processArg(i, argc, argv, "--use-heuristic-algorithm", "-uha", &Settings::USE_HEURISTICS)) continue;
		if (processArg(i, argc, argv, "--use-exact-algorithm", "-uea", &Settings::USE_EXACT_ALGORITHM)) continue;
		if (processArg(i, argc, argv, "--write-results", "-wr", Settings::RESULTS_DIRECTORY)) continue;

		/* ILP SOLVER OPTIONS */
		if (processArg(i, argc, argv, "--ilp-solver-relative-gap", "-isrg", Settings::ILP_RELATIVE_GAP, 0.0)) continue;
		if (processArg(i, argc, argv, "--lower-bound-calculation", "-lbc", &Settings::CALCULATE_LOWER_BOUND)) continue;
		if (processArg(i, argc, argv, "--lower-bound-runtime", "-lbr", Settings::RUNTIME_OF_LOWER_BOUND, 0.0)) continue;

		/* HEURISTIC OPTIONS */
		if (processArg(i, argc, argv, "--number-of-elite-solutions", "-noes", Settings::MAX_ELITE_SOLUTIONS)) continue;
		if (processArg(i, argc, argv, "--max-number-of-alternatives", "-mnoa", Settings::MAX_ALTERNATIVES)) continue;
		if (processArg(i, argc, argv, "--minimal-number-of-iters-per-tuple", "-mnoipt", Settings::MIN_ITERS_PER_TUPLE)) continue;

		throw InvalidArgument(caller(), "Unknown argument \""+string(argv[i])+"\"!");
	}

	if (Settings::DATASET_FILE.empty())
		throw InvalidArgument(caller(), "Insufficient number of parameters! At least a dataset must be specified!");

	if (Settings::USE_HEURISTICS == false && Settings::USE_EXACT_ALGORITHM == false)	{
		Settings::USE_HEURISTICS = DEFAULT_USE_HEURISTICS;
		Settings::USE_EXACT_ALGORITHM = DEFAULT_USE_EXACT_ALGORITHM;
	}

	if (!Settings::RESULTS_DIRECTORY.empty())
		Settings::RESULTS_DIRECTORY += "/";

	return false;
}

//! @}

/*!
 * \param argc The number of program arguments passed to the program is argc-1.
 * \param argv Program name and arguments (+parameters) of the program.
 * \return A return code of the program, see \ref ProgramReturnedCodes enum.
 * \brief An entry point of the program, arguments are processed,
 * dataset is parsed and solved, and results printed and optionally written to files.
 * \see InstancesReader, ParallelHeuristicSolver, RoboticLineSolverILP, SolutionChecker, SolverException
 * \callgraph
 */
int main(int argc, char* argv[])	{

	try {
		// It processes command line arguments.
		if (argc > 1)	{
			bool exit = processCommandLineArguments(argc, argv);
			if (exit == true)
				return EXIT_WITH_SUCCESS;
		} else {
			printProgramHelp(argv[0]);
			return EXIT_WITH_SUCCESS;
		}

		if (Settings::VERBOSE)
			printProgramHeader();

		// Dataset parsing, and filling data structures.
		InstancesReader reader;
		reader.readInstances();
		const vector<RoboticLine>& lines = reader.dataset();
		const vector<PrecalculatedMapping>& mappings = reader.precalculatedMappings();

		// Each robotic cell in the dataset is optimized.
		for (uint32_t lineId = 0; lineId < min(lines.size(), mappings.size()); ++lineId)	{

			const RoboticLine& line = lines[lineId];
			const PrecalculatedMapping& mapping = mappings[lineId];

			// Find a high quality solution.
			Solution solution;
			if (Settings::USE_HEURISTICS)	{
				ParallelHeuristicSolver solver(line, mapping);
				solution = solver.solve();
			} else {
				RoboticLineSolverILP solver(line, mapping);
				solution = solver.solve();
			}

			// Solution is checked for feasibility.
			SolutionChecker checker(line, mapping, solution);
			if (checker.checkAll())	{
				// Calculate the tight lower bound if requested.
				if (Settings::CALCULATE_LOWER_BOUND == true && (solution.status == OPTIMAL || solution.status == FEASIBLE))
					solution.lowerBound = RoboticLineSolverILP::lowerBoundOnEnergy(line.robots(), mapping);

				// Print all the solution or only criterion value depending on the verbosity level.
				if (Settings::VERBOSE)	{
					cout<<endl;
					cout<<solution<<endl;
				} else {
					writeBriefResultToStream(cout, "instance "+to_string(lineId), solution);
				}

				// Optionally, write optimization results to the specified directory.
				if (!Settings::RESULTS_DIRECTORY.empty())	{
					writeSolutionToCsvFile(Settings::RESULTS_DIRECTORY+"instance"+to_string(lineId)+".csv", solution, mapping);
					writeBriefResultToCsvFile("instance "+to_string(lineId), Settings::RESULTS_DIRECTORY+"dataset_results.csv", solution);
				}
			} else {
				if (Settings::VERBOSE)
					cerr<<endl;

				// Solution is infeasible even though the algorithm declared it as feasible, i.e. an error in the algorithm.
				cerr<<"Solution of instance "<<lineId<<" is invalid:"<<endl;
				for (const string& errMsg : checker.errorMessages())
					cerr<<errMsg<<endl;

				// Optionally, write error messages to the log file of the instance.
				if (!Settings::RESULTS_DIRECTORY.empty())	{
					string instanceErrorFile = Settings::RESULTS_DIRECTORY+"instance"+to_string(lineId)+".err";
					ofstream errorLog(instanceErrorFile.c_str());
					if (errorLog.good())	{
						for (const string& errMsg : checker.errorMessages())
							errorLog<<errMsg<<endl;
					} else {
						cerr<<"Cannot open an error log for instance "<<lineId<<"!"<<endl;
					}

					errorLog.close();
				}
			}

		}

	} catch (const InvalidDatasetFile& e)	{
		cerr<<exceptionToString(e)<<endl;
		cerr<<"\nProjectSolver: Cannot read the input dataset!"<<endl;
		return INPUT_OUTPUT_ERROR;
	} catch (const InvalidArgument& e)	{
		cerr<<exceptionToString(e)<<endl;
		return INVALID_PARAMETER;
	} catch (const SolverException& e)	{
		cerr<<exceptionToString(e)<<endl;
		cerr<<"\nProjectSolver: A runtime error occurred, terminating..."<<endl;
		return RUNTIME_ERROR;
	} catch (const exception& e)	{
		cerr<<exceptionToString(e)<<endl;
		return UNKNOWN_ERROR;
	}

	// We are happy if we get here...
	return EXIT_WITH_SUCCESS;
}


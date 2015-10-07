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

#ifndef HLIDAC_PES_UTILS_H
#define HLIDAC_PES_UTILS_H

/*!
 * \file Utils.h
 * \author Libor Bukata
 * \brief Various auxiliary functions used across the program.
 * \see safe_acc, hash_def
 */

#include <map>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <utility>
#include <vector>
#include <stdint.h>
#include "SolverConfig.h"
#include "Shared/Exceptions.h"

/*!
 * \defgroup safe_acc safe access to associative containers
 * Template methods provide safe access to associative containers,
 * i.e. it does not allow to store e.g. null pointers or rewrite already existing values in the map.
 * Errors are handled by using exceptions.
 * \brief Template methods provide safe access to associative containers.
 * \code
 * map<uint32_t, double> mymap;
 * setValue(mymap, 0, 5.0, caller()); 	// 0 -> 5.0
 * setValue(mymap, 1, 7.0, caller());	// 1 -> 7.0
 * double x = getValue(mymap, 0, caller());		// returns 5.0
 * double y = getValue(mymap, 1, caller());		// returns 7.0
 * cout<<"x + y = "<<x+y<<endl;		// prints "x + y = 12"
 * // Error, duplicit key, exception thrown.
 * // setValue(mymap, 1, 9.0, caller());
 * // Error, non-existing key, exception thrown.
 * // getValue(mymap, 4, caller());
 * \endcode
 * \note There is a useful macro caller() for filling the string of the caller method.
 * \warning Due to the construction of the caller string there is a significant overhead, and therefore
 * it is not recommended to use these functions inside the performance critical functions.
 * @{
 */
template <class C>
typename C::mapped_type getValue(const C& m, const typename C::key_type& key, std::string calledFrom)	{
	typename C::const_iterator it = m.find(key);
	if (it != m.cend())	{
		return it->second;
	} else {
		std::string msg = "Invalid key!\n\n";
		msg += "Called from:\n"+calledFrom;
		throw SolverException(caller(), msg);
	}
}

template <class C>
void setValueHelper(C& m, const typename C::key_type& key, const typename C::mapped_type& value, std::string calledFrom)    {
	auto p = m.insert({key, value});
	if (p.second == false)	{
		std::string msg = "Value with the same key already exists!\n\n";
		msg += "Called from:\n"+calledFrom;
		throw SolverException(caller(), msg);
	}
}

template <class C, class T = typename C::mapped_type>
void setValue(C& m, const typename C::key_type& key, const typename std::enable_if<std::is_pointer<T>::value, T>::type& value, std::string calledFrom)	{
	if (value == nullptr)	{
		std::string msg = "Attempt to store nullptr as a value!\n\n";
		msg += "Called from:\n"+calledFrom;
		throw SolverException(caller(), msg);
	}

	setValueHelper(m, key, value, calledFrom);
}

template <class C, class T = typename C::mapped_type>
void setValue(C& m, const typename C::key_type& key, const typename std::enable_if<!std::is_pointer<T>::value, T>::type& value, std::string calledFrom)	{
	setValueHelper(m, key, value, calledFrom);
}
// @}

/*!
 * \param v1, v2 The first and second number, respectively.
 * \return Both the numbers packed into the one with double width.
 * \brief It packs two uint32_t numbers to uint64_t data type.
 */
uint64_t pack(const uint32_t& v1, const uint32_t& v2);
/*!
 * \param v Unsigned integer (uint64_t) containing two packed uint32_t numbers.
 * \return A pair with the unpacked uint32_t numbers.
 * \brief It unpacks two uint32_t numbers from uint64_t data type.
 */
std::pair<uint32_t, uint32_t> unpack(const uint64_t& v);

/*!
 * \addtogroup hash_def
 * @{
 */

/*!
 * \param v Iterable container.
 * \tparam C Currently it is either vector<uint32_t> or vector<StaticActivity*>.
 * \return Calculated hash value of the container.
 * \brief It calculates a hash of the container,
 * the order of elements influences (Order Wise) the hash value.
 */
template <class C>
extern uint64_t hashOW(const C& v);

/*!
 * \param v Iterable container.
 * \tparam C Currently it is either vector<Location*> or set<Location*>.
 * \return Calculated hash value of the container.
 * \brief It calculates a hash of the container,
 * the order of elements does not influence (Element Wise) the hash value.
 */
template <class C>
extern uintptr_t hashEW(const C& v);

//! @}

/*!
 * \param pathToFile Path to a file.
 * \return True if the file exists, otherwise false.
 * \brief It checks the existence of the file.
 */
bool fileExists(const std::string& pathToFile);

#endif

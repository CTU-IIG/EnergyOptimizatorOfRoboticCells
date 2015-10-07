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
#include <type_traits>
#include <set>
#include "RoboticLine.h"
#include "Shared/Utils.h"

using namespace std;

uint64_t pack(const uint32_t& v1, const uint32_t& v2)	{
	uint64_t r = v1;
	r <<= 32; r |= v2;
	return r;
}

pair<uint32_t, uint32_t> unpack(const uint64_t& v)	{
	return { v>>32, v & 0x00000000ffffffff };
}

template <class T, typename enable_if<is_pointer<T>::value>::type* = nullptr>
uintptr_t cast(T v)	{
	return reinterpret_cast<uintptr_t>(v);
}

template <class T, typename enable_if<is_integral<T>::value>::type* = nullptr>
uintptr_t cast(T v)	{
	return static_cast<uintptr_t>(v);
}

template <class C>
uintptr_t elementWiseHash(const C& v)	{
	uintptr_t hash = 0;
	for (typename C::const_iterator it = v.cbegin(); it != v.cend(); ++it)
		hash ^= cast(*it);

	return hash;
}

template <class C>
uintptr_t orderWiseHash(const C& v)	{
	uintptr_t hash = 1ul, i = 1;
	constexpr uintptr_t R = 3144134277;
	for (typename C::const_iterator it = v.cbegin(); it != v.cend(); ++it, ++i)	{
		uintptr_t h = cast(*it);
		hash *= R+2*h*i;
		hash ^= h;
	}

	hash >>= 2;

	return hash;
}

template <class T>
inline uint64_t hashOW(const T& v)	{
	return orderWiseHash(v);
}

template <class T>
inline uintptr_t hashEW(const T& v)	{
	return elementWiseHash(v);
}

template uint64_t hashOW<vector<uint32_t>>(const vector<uint32_t>&);
template uint64_t hashOW<vector<StaticActivity*>>(const vector<StaticActivity*>&);
template uintptr_t hashEW<vector<Location*>>(const vector<Location*>&);
template uintptr_t hashEW<set<Location*>>(const set<Location*>&);

bool fileExists(const string& pathToFile)	{
	bool exists = false;
	ifstream in(pathToFile.c_str());
	if (in.good())
		exists = true;

	in.close();
	return exists;
}


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

#ifndef HLIDAC_PES_SPARSE_MATRIX_H
#define HLIDAC_PES_SPARSE_MATRIX_H

/*!
 * \file SparseMatrix.h
 * \author Libor Bukata
 * \brief Memory efficient storage for a constraint matrix of Integer Linear Programming problem.
 */

#include <algorithm>
#include <cassert>
#include <iostream>
#include <set>
#include <vector>
#include <utility>
#include <stdint.h>

/*!
 * Sparse matrix implementation suitable for the row-by-row consecutive access
 * that is very appropriate for building of Integer Linear Programming problems.
 * A random access by columns is not recommended since it causes huge latencies.
 * \tparam T Mostly either float or double.
 * \brief Memory efficient storage of the constraint matrix.
 */
template <class T>
class SparseMatrix {
	public:
		//! Creates empty matrix with a default value for unfilled elements.
		SparseMatrix(T defaultValue = T()) : mDefaultValue(defaultValue) { }

		/*!
		 * \param i,j Row and column index, respectively.
		 * \return Value of the matrix at (i,j) indices.
		 * \brief It returns a value of the matrix for the specified indices.
		 */
		T get(const uint32_t& i, const uint32_t& j) const {
			assert(i < mSparseMatrix.size());
			for (const auto& rowElement : mSparseMatrix[i])	{
				if (rowElement.first == j)
					return rowElement.second;
			}

			return mDefaultValue;
		}

		//! The matrix row is stored as a vector of pairs where each pair consists of column index and the related value.
		using Row = std::vector<std::pair<uint32_t, T> >;

		//! Adds precreated row to the matrix. Passed argument is destroyed.
		void addRow(Row& row)	{
			assert(!row.empty() && checkColumn(row) && "Invalid matrix row!");
			mSparseMatrix.push_back(std::move(row));
		}

		//! It returns i-th row of the matrix.
		Row& operator[](const uint32_t& i) {
		        assert(i < mSparseMatrix.size() && "Row index is out of range!");
			return mSparseMatrix[i];
		}

		//! It returns i-th row of the matrix.
		const Row& operator[](const uint32_t& i) const {
		        assert(i < mSparseMatrix.size() && "Row index is out of range!");
			return mSparseMatrix[i];
		}

		uint64_t numberOfRows() const { return mSparseMatrix.size(); }

		uint64_t numberOfColumns() const {
			int64_t numberOfColumns = -1;
			for (const auto& row : mSparseMatrix)	{
				for (const auto& element : row)
					numberOfColumns = std::max(numberOfColumns, (int64_t) element.first);
			}

			return (numberOfColumns != -1) ? (uint64_t) numberOfColumns+1 : 0ul;
		}

		//! Number of non-zero elements of the matrix.
		uint64_t numberOfElements() const {
			uint64_t numberOfElements = 0;
			for (const auto& row : mSparseMatrix)
				numberOfElements += row.size();
			return numberOfElements;
		}

		//! Percentage of filled elements.
		double densityOfMatrix() const {
			return ((double) numberOfElements())/((double) numberOfRows()*numberOfColumns());
		}

		//! Destroys the matrix.
		void clear() { mSparseMatrix.clear(); }

	private:

		/*!
		 * \param row Matrix row to be checked.
		 * \return True if the column does not contain redefined values, otherwise false.
		 * \brief It checks whether the column is uniquely defined.
		 */
		bool checkColumn(const Row& row) const {
			std::set<uint32_t> uniqueColumns;
			for (const auto& el : row)	{
				if (uniqueColumns.count(el.first) > 0)
					return false;
				else
					uniqueColumns.insert(el.first);
			}

			return true;
		}

		//! Default value for unfilled elements.
		T mDefaultValue;
		//! Sparse constraint matrix.
		std::vector<Row> mSparseMatrix;
};

//! Output stream operator for the constraint matrix, mostly used for debugging purposes.
template <class T>
std::ostream& operator<<(std::ostream& out, const SparseMatrix<T>& m)	{
	out<<"A = ["<<std::endl;
	for (uint32_t r = 0; r < m.numberOfRows(); ++r)	{
		out<<"\t";
		for (uint32_t c = 0; c < m.numberOfColumns(); ++c)
			out<<" "<<m.get(r,c);

		if (r+1 < m.numberOfRows())
			out<<";"<<std::endl;
		else
			out<<std::endl;
	}
	out<<"];";

	return out;
}

#endif

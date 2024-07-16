//
// MIT License
//
// Copyright (c) 2024 Stellacore Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


#ifndef OriNet_stat_INCL_
#define OriNet_stat_INCL_

/*! \file
\brief Classes for computing/tracking statistics from data streams.

Example:
\snippet test_stat.cpp DoxyExample01

*/


#include "align.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <array>
#include <vector>


namespace orinet
{

namespace stat
{

namespace track
{

	//! Track running statistics for individual data values.
	class Values
	{
		std::vector<double> theValues{};

	public:

		/*! \brief Allocate space to hold all data values
		 *
		 * This implementation holds a copy of all data values.
		 * Therefore, (for efficiency) construction should allocate
		 * at least enough space to hold all values. Otherwise
		 * inserting a values may cause a reallocation/copy
		 * operations. This should work okay, but will affect
		 * performance to some degree (depending on size).
		 */
		inline
		explicit
		Values
			( std::size_t const & reserveSize
			)
			: theValues{}
		{
			theValues.reserve(reserveSize);
		}

		//! \brief Number of values that have been inserted.
		inline
		std::size_t
		size
			() const
		{
			return theValues.size();
		}

		//! \brief Incorporate value into data collection.
		inline
		void
		insert
			( double const & value
			)
		{
			// insert in sorted order
			std::vector<double>::iterator const itFind
				{ std::lower_bound(theValues.begin(), theValues.end(), value) };
			theValues.insert(itFind, value);
		}

		/*! \brief Median value of all inserted items.
		 *
		 * Returns engabra::g3::null<double>() if empty. Otherwise
		 * returns the middle value (of sorted) list for odd number
		 * of elements, and the average of the two middle values
		 * for even number of elements.
		 */
		inline
		double
		median
			() const
		{
			double med{ engabra::g3::null<double>() };
			std::size_t const numElem{ theValues.size() };
			if (0u < numElem)
			{
				bool const isOdd{ 1u == (numElem % 2u) };
				std::size_t const ndxHalf{ numElem / 2u };
				if (isOdd)
				{
					med = theValues[ndxHalf];
				}
				else // isEven
				{
					std::size_t const ndxB4{ ndxHalf - 1u };
					med = .5 * (theValues[ndxB4] + theValues[ndxHalf]);
				}
			}
			return med;
		}

	}; // Values

	//! Track running statistics for individual data values.
	class Vectors
	{
		std::array<Values, 3u> theValues;

	public:

		/*! \brief Allocate space to hold all data values.
		 *
		 * This implementation holds a copy of all data values.
		 * Therefore, (for efficiency) construction should allocate
		 * at least enough space to hold all values. Otherwise
		 * inserting a values may cause a reallocation/copy
		 * operations. This should work okay, but will affect
		 * performance to some degree (depending on size).
		 */
		inline
		explicit
		Vectors
			( std::size_t const & reserveSize
			)
			: theValues
				{ Values(reserveSize)
				, Values(reserveSize)
				, Values(reserveSize)
				}
		{ }

		//! \brief Number of values that have been inserted.
		inline
		std::size_t
		size
			() const
		{
			return theValues[0].size();
		}

		//! \brief Incorporate value into data collection.
		inline
		void
		insert
			( engabra::g3::Vector const & value
			)
		{
			// add components to component values
			theValues[0].insert(value[0]);
			theValues[1].insert(value[1]);
			theValues[2].insert(value[2]);
		}

		/*! \brief Vector comprised of median of all coordinate values.
		 *
		 * Returns engabra::g3::null<Vector>() if empty. Otherwise
		 * returns the middle value (of sorted) list for odd number
		 * of elements, and the average of the two middle values
		 * for even number of elements.
		 */
		inline
		engabra::g3::Vector
		median
			() const
		{
			return engabra::g3::Vector
				{ theValues[0].median()
				, theValues[1].median()
				, theValues[2].median()
				};
		}

	}; // Vectors

	//! Track running statistics for individual Attitudes.
	class Attitudes
	{
		std::array<Vectors, 2u> theIntoVecs;

	public:

		/*! \brief Allocate space to hold all data values.
		 *
		 * This implementation holds a copy of all data values.
		 * Therefore, (for efficiency) construction should allocate
		 * at least enough space to hold all values. Otherwise
		 * inserting a values may cause a reallocation/copy
		 * operations. This should work okay, but will affect
		 * performance to some degree (depending on size).
		 */
		inline
		explicit
		Attitudes
			( std::size_t const & reserveSize
			)
			: theIntoVecs
				{ Vectors(reserveSize)
				, Vectors(reserveSize)
				}
		{ }

		//! \brief Number of values that have been inserted.
		inline
		std::size_t
		size
			() const
		{
			return theIntoVecs[0].size();
		}

		/*! \brief Incorporate attitude information into data collection.
		 *
		 * The attitude is used to transform basis vectors, e1 and e2
		 * into the transform range. Each of the results is individually
		 * tracked in a track::Vectors instance.
		 */
		inline
		void
		insert
			( rigibra::Attitude const & value
			)
		{
			using namespace engabra::g3;
			Vector const into0{ value(e1) };
			Vector const into1{ value(e2) };
			theIntoVecs[0].insert(into0);
			theIntoVecs[1].insert(into1);
		}

		/*! \brief Vector comprised of median of all coordinate values.
		 *
		 * Returns engabra::g3::null<Vector>() if empty. Otherwise
		 * returns the middle value (of sorted) list for odd number
		 * of elements, and the average of the two middle values
		 * for even number of elements.
		 */
		inline
		rigibra::Attitude
		median
			() const
		{
			// fetch median points
			using namespace engabra::g3;
			Vector const intoA{ theIntoVecs[0].median() };
			Vector const intoB{ theIntoVecs[1].median() };

			constexpr align::DirPair fromDirPair{ e1, e2 };
			align::DirPair const intoDirPair{ intoA, intoB };

			return align::attitudeFromDirPairs(fromDirPair, intoDirPair);
		}

	}; // Attitudes


} // [track]

} // [stat]

} // [orinet]


#endif // OriNet_stat_INCL_

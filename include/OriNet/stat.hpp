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
\brief Contains TODO

Example:
\snippet test_stat.cpp DoxyExample01

*/


#include <Engabra>

#include <algorithm>
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
					std::cout << "using middle element\n";
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

} // [track]

} // [stat]

} // [orinet]


#endif // OriNet_stat_INCL_

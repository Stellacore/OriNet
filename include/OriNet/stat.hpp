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
#include "compare.hpp"

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

		/*! \brief Value before the median value.
		 */
		inline
		double
		medianPrev
			() const
		{
			double next{ engabra::g3::null<double>() };
			std::size_t const numElem{ theValues.size() };
			if (1u < numElem)
			{
				std::size_t const ndxHalf{ numElem / 2u };
				next = theValues[ndxHalf - 1];
			}
			return next;
		}

		/*! \brief Value after the median value.
		 */
		inline
		double
		medianNext
			() const
		{
			double prev{ engabra::g3::null<double>() };
			std::size_t const numElem{ theValues.size() };
			if (1u < numElem)
			{
				bool const isOdd{ 1u == (numElem % 2u) };
				std::size_t const ndxHalf{ numElem / 2u };
				if (isOdd)
				{
					prev = theValues[ndxHalf + 1];
				}
				else // isEven
				{
					prev = theValues[ndxHalf];
				}
			}
			return prev;
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

		/*! \brief Vector composed of values immediately before median value.
		 */
		inline
		engabra::g3::Vector
		medianPrev
			() const
		{
			return engabra::g3::Vector
				{ theValues[0].medianPrev()
				, theValues[1].medianPrev()
				, theValues[2].medianPrev()
				};
		}

		/*! \brief Vector composed of values immediately after median value.
		 */
		inline
		engabra::g3::Vector
		medianNext
			() const
		{
			return engabra::g3::Vector
				{ theValues[0].medianNext()
				, theValues[1].medianNext()
				, theValues[2].medianNext()
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

		//! Attitude that 'best' transforms {e1,e2} to into_{e1,e2} pair.
		inline
		static
		rigibra::Attitude
		attitudeFrom_e1e2
			( engabra::g3::Vector const & into_e1
			, engabra::g3::Vector const & into_e2
			)
		{
			using namespace engabra::g3;
			static  align::DirPair fromDirPair{ e1, e2 };
			align::DirPair const intoDirPair{ into_e1, into_e2 };
			return align::attitudeFromDirPairs(fromDirPair, intoDirPair);
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
			return attitudeFrom_e1e2(intoA, intoB);
		}

		/*! \brief Attitude from values immediately before median value.
		 */
		inline
		rigibra::Attitude
		medianPrev
			() const
		{
			using namespace engabra::g3;
			Vector const intoA{ theIntoVecs[0].medianPrev() };
			Vector const intoB{ theIntoVecs[1].medianPrev() };
			return attitudeFrom_e1e2(intoA, intoB);
		}

		/*! \brief Attitude from values immediately after median value.
		 */
		inline
		rigibra::Attitude
		medianNext
			() const
		{
			using namespace engabra::g3;
			Vector const intoA{ theIntoVecs[0].medianNext() };
			Vector const intoB{ theIntoVecs[1].medianNext() };
			return attitudeFrom_e1e2(intoA, intoB);
		}

	}; // Attitudes

	//! Track running statistics for individual Transforms.
	class Transforms
	{
		Vectors theLocs;
		Attitudes theAtts;

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
		Transforms
			( std::size_t const & reserveSize
			)
			: theLocs(reserveSize)
			, theAtts(reserveSize)
		{ }

		//! \brief Number of values that have been inserted.
		inline
		std::size_t
		size
			() const
		{
			return theLocs.size();
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
			( rigibra::Transform const & value
			)
		{
			theLocs.insert(value.theLoc);
			theAtts.insert(value.theAtt);
		}

		/*! \brief Vector comprised of median of all coordinate values.
		 *
		 * Returns engabra::g3::null<Vector>() if empty. Otherwise
		 * returns the middle value (of sorted) list for odd number
		 * of elements, and the average of the two middle values
		 * for even number of elements.
		 */
		inline
		rigibra::Transform
		median
			() const
		{
			// result composed of median position and median attitude
			return rigibra::Transform{ theLocs.median(), theAtts.median() };
		}

		/*! \brief Transform from values immediately before median value.
		 */
		inline
		rigibra::Transform
		medianPrev
			() const
		{
			return rigibra::Transform
				{ theLocs.medianPrev(), theAtts.medianPrev() };
		}

		/*! \brief Transform from values immediately after median value.
		 */
		inline
		rigibra::Transform
		medianNext
			() const
		{
			return rigibra::Transform
				{ theLocs.medianNext(), theAtts.medianNext() };
		}

		/*! \brief Estimate the error in median transform.
		 *
		 * If three or more items have been inserted, then the error is
		 * estimated by comparing the items adjacent to the median value.
		 * For an even number of elements, the error is estimated by
		 * comparing the two transforms on either side of the median.
		 * For an odd number of elements, the error is estimated as
		 * one half the difference of the two transforms on either side
		 * of the median.
		 *
		 * The error estimate is computed using the function
		 * compare::maxMagResultDifference() which evaluates the maximum
		 * error of transformed basis vectors.
		 *
		 * If there is only one (or zero) transforms in the collection
		 * a null values (nan) is returned.
		 */
		inline
		double
		medianErrorEstimate
			( bool const & useNormalizedCompare
			) const
		{
			double err{ engabra::g3::null<double>() };
			using namespace rigibra;
			Transform const xPrevWrtX{ medianPrev() };
			Transform const xNextWrtX{ medianNext() };
			err = compare::maxMagResultDifference
				(xPrevWrtX, xNextWrtX, useNormalizedCompare);
			// For odd number of elements, the Prev/Next are "2 units"
			// apart, whereas for even number of elements, the Prev/Next
			// are adjacent. Therefore, if odd number of elements,
			// divide the estimated error by half to approximate the
			// average of compare(prev,median) and compare(median,next)
			// without having to compute the two independent comparisons.
			bool const isOdd{ 1u == (size() % 2u) };
			if (isOdd)
			{
				err = .5 * err;
			}
			return err;
		}

	}; // Transforms


} // [track]

} // [stat]

} // [orinet]


#endif // OriNet_stat_INCL_

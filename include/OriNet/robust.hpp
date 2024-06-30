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


#ifndef OriNet_robust_INCL_
#define OriNet_robust_INCL_

/*! \file
\brief Contains Function to determine robust transformation estimates

Example:
\snippet test_robust.cpp DoxyExample01

*/


#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <array>
#include <vector>


namespace orinet
{
	/*! Return the median value of array of \b NOT_CONSTANT values.
	 *
	 * For non-empty collection containing 'N' elements:
	 * \arg For empty sizes: returns engabra::g3::null<double>() value.
	 * \arg For odd sizes: returns the "N/2-th" element
	 * \arg For even sizes: returns average of "N/2-th" and next element
	 *
	 *
	 * \note All data values are assumed to be valid (sortable) - e.g.
	 * none of them are NaN or infinity or other than valid numeric values.
	 */
	inline
	double
	medianOf
		( std::vector<double> & values
		)
	{
	 	double median{ engabra::g3::null<double>() };

		if (! values.empty())
		{
			std::size_t const sizeN{ values.size() };

		//	bool const isEven{ 0u == (sizeN % 2u) };
			bool const isOdd{ 1u == (sizeN % 2u) };

			// (0)  - even none  (special case)
			// ><
			// 0 (1)  - odd use 0  (N/2-th element)
			// >m <
			// 0 1 (2)  - even average 0,1  (N/2-th element and Next)
			// > m  <
			// 0 1 2 (3)  -  odd use 1  (N/2-th element)
			// > m    <
			// 0 1 2 3 (4)  - even average 1,2  (N/2-th element and Next)
			// >   m    <
			// 0 1 2 3 4 (5)  - odd use 2  (N/2-th element)
			// >   m      <
			// 0 1 2 3 4 5 (6)  - even average 2,3  (N/2-th element and Next)
			// >     m      <

			std::size_t const halfN{ sizeN / 2u };

			std::vector<double>::iterator const itBeg{ values.begin() };
			std::vector<double>::iterator const itMid{ itBeg + halfN };
			std::vector<double>::iterator const itEnd{ values.end() };

			// The largest of the smallest "half" of values
			std::nth_element(itBeg, itMid, itEnd);

			if (isOdd)
			{
				median = *itMid;
			}
			else // if (isEven)
			{
				// average the found value (largest of the smallest half
				// of all values) with the next value which is
				// the smallest of the remaining values (which are
				// all larger then *itMid).
				std::vector<double>::const_iterator const itNext
					{ std::min_element(itMid + 1u, itEnd) };
				if (itEnd == itNext)
				{
					std::cerr << "robust.hpp - fatal error itEnd==itNext\n";
					exit(1);
				}
				median = .5 * ((*itMid) + (*itNext));
			}
		}

		return median;
	}

	/*! \brief A rosbustly computed transform consistent with collection.
	 *
	 * Special cases include collection of:
	 * \arg zero items (empty) - return null transform
	 * \arg one item - return same transform as the collection item
	 * \arg two item - return an "average" of the two
	 * \arg three or more items - return a 'median transform' described below
	 *
	 * For present purposes, a "Median Transform" is defined as described
	 * as follows. (There may or may not be something relevant in the
	 * literature - a quick search has provided nothing useful)
	 *
	 * The collection of 'N' transformations is decomposed into a collection
	 * of location vectors and physical angle bivectors, each of which has
	 * three component values. The 'N' values for each of the six components
	 * is used to compute a median value for that component. The median
	 * transformation result is a new synthesized transformation defined
	 * using the six computed median results.
	 *
	 * Dereferencing to (* FwdIter) must be a rigibra::Trasnformation.
	 */
	template <typename FwdIter>
	inline
	rigibra::Transform
	robustTransformFrom
		( FwdIter const & beg
		, FwdIter const & end
		)
	{
		rigibra::Transform median{ rigibra::null<rigibra::Transform>() };

		std::size_t const numXforms{ static_cast<std::size_t>(end - beg) };
		if (0u < numXforms)
		{
			//
			// Copy the available parameter components into mutable collections
			//
			std::array<std::vector<double>, 6u> compVecs;
			compVecs[0].reserve(numXforms);
			compVecs[1].reserve(numXforms);
			compVecs[2].reserve(numXforms);
			compVecs[3].reserve(numXforms);
			compVecs[4].reserve(numXforms);
			compVecs[5].reserve(numXforms);
			for (FwdIter iter{beg} ; end != iter ; ++iter)
			{
				if (rigibra::isValid(*iter))
				{
					engabra::g3::Vector const & loc = iter->theLoc;
					rigibra::PhysAngle
						const physAngle{ iter->theAtt.physAngle() };
					compVecs[0].emplace_back(loc[0]);
					compVecs[1].emplace_back(loc[1]);
					compVecs[2].emplace_back(loc[2]);
					compVecs[3].emplace_back(physAngle.theBiv[0]);
					compVecs[4].emplace_back(physAngle.theBiv[1]);
					compVecs[5].emplace_back(physAngle.theBiv[2]);
				}
			}

			if (! compVecs[0].empty()) // all six have same size
			{
				//
				// Form a new transformation from the component means
				//
				median = rigibra::Transform
					{ engabra::g3::Vector
						{ medianOf(compVecs[0])
						, medianOf(compVecs[1])
						, medianOf(compVecs[2])
						}
					, rigibra::Attitude
						{ rigibra::PhysAngle
							{ medianOf(compVecs[3])
							, medianOf(compVecs[4])
							, medianOf(compVecs[5])
							}
						}
					};
			}
		}

		return median;
	}

} // [orinet]


#endif // OriNet_robust_INCL_

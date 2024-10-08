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

*/


#include "align.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <array>
#include <vector>


namespace orinet
{

/*! \brief Functions for robust clustering of rigid body transformations.
 */
namespace robust
{

	/*! \brief Return the median value of array of \b NOT_CONSTANT values.
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

			// Even: (0)  - even none  (special case)
			//      >< 
			//  Odd: 0 (1)  - odd use 0  (N/2-th element)
			//      >m < 
			// Even: 0 1 (2)  - even average 0,1  (N/2-th and Prev)
			//      >  m < 
			//  Odd: 0 1 2 (3)  -  odd use 1  (N/2-th element)
			//      >  m   < 
			// Even: 0 1 2 3 (4)  - even average 1,2  (N/2-th and Prev)
			//      >    m   < 
			//  Odd: 0 1 2 3 4 (5)  - odd use 2  (N/2-th element)
			//      >    m     < 
			// Even: 0 1 2 3 4 5 (6)  - even average 2,3  (N/2-th and Prev)
			//      >      m     < 

			std::size_t const halfN{ sizeN / 2u };
			std::size_t midN;
			if (isOdd)
			{
				midN = halfN;
			}
			else
			{
				if (! (0u < halfN))
				{
					std::cerr << "FATAL ERROR\n";
					exit(8);
				}
				// is true that (0u < halfN) since here is (N=even && 0<N)
				midN = halfN - 1u;
			}

			std::vector<double>::iterator const itBeg{ values.begin() };
			std::vector<double>::iterator const itMid{ itBeg + midN };
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
					std::cerr << __FILE__ << " - fatal error itEnd==itNext\n";
				}
				else
				{
					median = .5 * ((*itMid) + (*itNext));
				}
			}
		}

		return median;
	}

	/*! \brief Rosbustly computed transform consistent with xform collection.
	 *
	 * This implementation evaluates similarity by comparing transformation
	 * parameter component values (three vector offset components, and three
	 * bivector angle components).
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
	 * \note Dereferencing to (* FwdIter) must be a rigibra::Trasnformation.
	 *
	 * Example:
	 * \snippet test_robust.cpp DoxyExample01
	 */
	template <typename FwdIter>
	inline
	rigibra::Transform
	transformViaParameters
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
				else
				{
					std::cerr << __FILE__ << " - bad transform\n";
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

	/*! \brief Rosbustly computed transform consistent with xform collection.
	 *
	 * This implementation evaluates similarity using the *effect* that
	 * the transform has on data vectors.
	 *
	 * Special cases include collection of:
	 * \arg zero items (empty) - return null transform
	 * \arg one item - return same transform as the collection item
	 * \arg two item - return an "average" of the two
	 * \arg three or more items - return a 'median transform' described below
	 *
	 * Algorithm involves:
	 * \arg use median of translation vectors for translation offset
	 * \arg transform two orthogonal vectors (e.g. e1,e2)
	 * \arg create a resultant point cloud of each
	 * \arg compute median of each vector location within point cloud
	 * \arg construct median attitude by rotation onto the two median vectors
	 *
	 * \note Dereferencing to (* FwdIter) must be a rigibra::Trasnformation.
	 *
	 * Example:
	 * \snippet test_robust.cpp DoxyExample02
	 */
	template <typename FwdIter>
	inline
	rigibra::Transform
	transformViaEffect
		( FwdIter const & beg
		, FwdIter const & end
		)
	{
		rigibra::Transform median{ rigibra::null<rigibra::Transform>() };

		std::size_t const numXforms{ static_cast<std::size_t>(end - beg) };
		if (0u < numXforms)
		{
			using namespace engabra::g3;
			// pair of vectors to track through different transforms
			static Vector const a0{ e1 };
			static Vector const b0{ e2 };
			static align::DirPair const refDirPair{ a0, b0 };

			//
			// Copy translation parameter components into mutable collections
			//
			std::array<std::vector<double>, 3u> compVecs;
			compVecs[0].reserve(numXforms);
			compVecs[1].reserve(numXforms);
			compVecs[2].reserve(numXforms);
			std::array<std::vector<double>, 3u> comp_a1s;
			comp_a1s[0].reserve(numXforms);
			comp_a1s[1].reserve(numXforms);
			comp_a1s[2].reserve(numXforms);
			std::array<std::vector<double>, 3u> comp_b1s;
			comp_b1s[0].reserve(numXforms);
			comp_b1s[1].reserve(numXforms);
			comp_b1s[2].reserve(numXforms);

			for (FwdIter iter{beg} ; end != iter ; ++iter)
			{
				if (rigibra::isValid(*iter))
				{
					// gather translation vector components
					Vector const & loc = iter->theLoc;
					compVecs[0].emplace_back(loc[0]);
					compVecs[1].emplace_back(loc[1]);
					compVecs[2].emplace_back(loc[2]);

					// gather transformed basis pair components
					using namespace rigibra;
					Attitude const & att = iter->theAtt;
					Vector const a1{ att(a0) };
					Vector const b1{ att(b0) };
					//
					comp_a1s[0].emplace_back(a1[0]);
					comp_a1s[1].emplace_back(a1[1]);
					comp_a1s[2].emplace_back(a1[2]);
					//
					comp_b1s[0].emplace_back(b1[0]);
					comp_b1s[1].emplace_back(b1[1]);
					comp_b1s[2].emplace_back(b1[2]);

				}
			}

			if (! compVecs[0].empty()) // all six have same size
			{
				Vector const medianLoc
					{ medianOf(compVecs[0])
					, medianOf(compVecs[1])
					, medianOf(compVecs[2])
					};

				// robust estimate for transformed direction pair
				Vector const median_a1
					{ medianOf(comp_a1s[0])
					, medianOf(comp_a1s[1])
					, medianOf(comp_a1s[2])
					};
				Vector const median_b1
					{ medianOf(comp_b1s[0])
					, medianOf(comp_b1s[1])
					, medianOf(comp_b1s[2])
					};
				align::DirPair const bodDirPair{ median_a1, median_b1 };

				// attitude transforming reference pair onto body pair
				rigibra::Attitude const medianAtt
					{ align::attitudeFromDirPairs(refDirPair, bodDirPair) };

				//
				// Form a new transformation from the component means
				//
				median = rigibra::Transform{ medianLoc, medianAtt };
			}
		}

		return median;
	}

} // [robust]

} // [orinet]


#endif // OriNet_robust_INCL_

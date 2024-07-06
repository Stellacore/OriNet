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


#ifndef OriNet_compare_INCL_
#define OriNet_compare_INCL_


#include "OriNet/robust.hpp" // for medianOf() - probably should factor out

#include <Rigibra>

#include <algorithm>
#include <array>
#include <limits>
#include <vector>


/*! \file
\brief Utilities for comparing transformation results

*/


namespace orinet
{

/*! \brief Functions for assessing similarity/difference between transforms.
 */
namespace compare
{

	/*! \brief Differences: all basis vectors transformed by each attitude.
	 *
	 * Each of the basis vectors {e1, e2, e3} are transformed by each
	 * attitude. The corresponding difference vectors are returned.
	 */
	inline
	std::array<engabra::g3::Vector, 3u>
	triadDeltaVectors
		( rigibra::Attitude const & att1
		, rigibra::Attitude const & att2
		)
	{
		using namespace engabra::g3;
		std::array<Vector, 3u> diffs;
		if (rigibra::isValid(att1) && rigibra::isValid(att2))
		{
			// transform attitude changes for each of the +/- each basis vector
			Vector const into_e1_1{ att1(e1) };
			Vector const into_e1_2{ att2(e1) };

			Vector const into_e2_1{ att1(e2) };
			Vector const into_e2_2{ att2(e2) };

			Vector const into_e3_1{ att1(e3) };
			Vector const into_e3_2{ att2(e3) };

			// return all three differences
			diffs = std::array<Vector, 3u>
				{ (into_e1_2 - into_e1_1)
				, (into_e2_2 - into_e2_1)
				, (into_e3_2 - into_e3_1)
				};
		}
		return diffs;
	}

	/*! \brief Max mag difference in basis vectors transformed by each xfm[12]
	 *
	 * Specifically, each data argument is used to transform the endpoints
	 * of a "hexad" comprising six basis vectors (i.e. +/- e_{1,2,3}).
	 * Vector differences are computed between corresponding hexad entities
	 * and returnedin the array.
	 */
	inline
	std::array<engabra::g3::Vector, 6u>
	hexadDeltaVectors
		( rigibra::Transform const & xfm1
		, rigibra::Transform const & xfm2
		, bool const & useNormalizedCompare
		)
	{
		using namespace engabra::g3;
		std::array<Vector, 6u> diffs
			{ null<Vector>()
			, null<Vector>()
			, null<Vector>()
			, null<Vector>()
			, null<Vector>()
			, null<Vector>()
			};

		if (isValid(xfm1) && isValid(xfm2))
		{
			/*
			//
			// Full computation (lots of redundant operations)
			//

			// transform attitude changes for each of the +/- each basis vector
			Vector const into_pe1_1{ xfm1( e1) };
			Vector const into_pe1_2{ xfm2( e1) };

			Vector const into_pe2_1{ xfm1( e2) };
			Vector const into_pe2_2{ xfm2( e2) };

			Vector const into_pe3_1{ xfm1( e3) };
			Vector const into_pe3_2{ xfm2( e3) };

			Vector const into_ne1_1{ xfm1(-e1) };
			Vector const into_ne1_2{ xfm2(-e1) };

			Vector const into_ne2_1{ xfm1(-e2) };
			Vector const into_ne2_2{ xfm2(-e2) };

			Vector const into_ne3_1{ xfm1(-e3) };
			Vector const into_ne3_2{ xfm2(-e3) };

			diffs = std::array<Vector, 6u>
				{ ( into_pe1_1 - into_pe1_2 )
				, ( into_pe2_1 - into_pe2_2 )
				, ( into_pe3_1 - into_pe3_2 )
				, ( into_ne1_1 - into_ne1_2 )
				, ( into_ne2_1 - into_ne2_2 )
				, ( into_ne3_1 - into_ne3_2 )
				};
			*/

			//
			// Reduced effort computation
			// Ref theory/compare.lyx
			//

			// compute translation normalizing scale factor
			double rho{ 1. };
			if (useNormalizedCompare)
			{
				double const aveMag
					{ .5 * (magnitude(xfm1.theLoc) + magnitude(xfm2.theLoc)) };
				rho = std::max(1., aveMag);
			}

			// apply rotation to the translation components of transformations
			Vector const into_t_1{ xfm1.theAtt(xfm1.theLoc) };
			Vector const into_t_2{ xfm2.theAtt(xfm2.theLoc) };
			Vector const delta_trans{ (into_t_1 - into_t_2) };

			// transform attitude changes for each of the +/- each basis vector
			std::array<Vector, 3u> const deltas
				{ triadDeltaVectors(xfm1.theAtt, xfm2.theAtt) };
			Vector const delta_e1{ rho * deltas[0] };
			Vector const delta_e2{ rho * deltas[1] };
			Vector const delta_e3{ rho * deltas[2] };

			// residual distances
			diffs = std::array<Vector, 6u>
				{ delta_trans + delta_e1
				, delta_trans - delta_e1
				, delta_trans + delta_e2
				, delta_trans - delta_e2
				, delta_trans + delta_e3
				, delta_trans - delta_e3
				};

			/*
			//std::cout << '\n';
			std::cout << " lib:diff: ";
			for (Vector const & diff : diffs)
			{
				std::cout << "  " << diff;
			}
			std::cout << '\n';
			*/

		}
		return diffs;
	}

	/*! \brief Expected mag difference in transform of basis hexad
	 *
	 * Specifically, each data argument is used to transform the endpoints
	 * of six basis vectors (i.e. +/- e_{1,2,3}). Vector differences are
	 * computed between corresponding entities, and the expected (mean)
	 * magnitude of the six difference vectors is returned.
	 */
	inline
	double
	aveMagResultDifference
		( rigibra::Transform const & xfm1
		, rigibra::Transform const & xfm2
		, bool const & useNormalizedCompare
		)
	{
		double aveMag{ engabra::g3::null<double>() };

		if (isValid(xfm1) && isValid(xfm2))
		{
			using namespace engabra::g3;

			// residual distances
			std::array<Vector, 6u> const diffs
				{ hexadDeltaVectors(xfm1, xfm2, useNormalizedCompare) };

			// compute max delta
			// first std::max() will change to positive magnitude
			double sumMag{ 0. };
			for (Vector const & diff : diffs)
			{
				sumMag += magnitude(diff);
			}
			aveMag = (1./6.) * sumMag;
		}

		return aveMag;
	}

	/*! \brief Max mag difference in basis vectors transformed by each xfm[12]
	 *
	 * Specifically, each data argument is used to transform the endpoints
	 * of six basis vectors (i.e. +/- e_{1,2,3}). Vector differences are
	 * computed between corresponding entities, and the maximum magnitude
	 * of the six difference vectors is returned.
	 */
	inline
	double
	maxMagResultDifference
		( rigibra::Transform const & xfm1
		, rigibra::Transform const & xfm2
		, bool const & useNormalizedCompare
		)
	{
		double maxMag{ engabra::g3::null<double>() };

		if (isValid(xfm1) && isValid(xfm2))
		{
			using namespace engabra::g3;

			// residual distances
			std::array<Vector, 6u> const diffs
				{ hexadDeltaVectors(xfm1, xfm2, useNormalizedCompare) };

			// compute max delta
			// first std::max() will change to positive magnitude
			maxMag = -1.;
			for (Vector const & diff : diffs)
			{
				maxMag = std::max(maxMag, magnitude(diff));
			}
		}

		return maxMag;
	}


	/*! \brief True if both attitudes produce similar effect on basis vectors.
	 *
	 * Example:
	 * \snippet test_nearness.cpp DoxyExample01
	 */
	inline
	bool
	similarResult
		( rigibra::Attitude const & att1
		, rigibra::Attitude const & att2
		, double const & tol = std::numeric_limits<double>::epsilon()
		, double * const & ptMaxMag = nullptr
		)
	{
		bool same{ false };
		double maxMag{ engabra::g3::null<double>() };
		if (isValid(att1) && isValid(att2))
		{
			using namespace engabra::g3;

			// find largest difference
			std::array<Vector, 3u> const deltas
				{ triadDeltaVectors(att1, att2) };
			std::array<double, 3u> const mags
				{ magnitude(deltas[0])
				, magnitude(deltas[1])
				, magnitude(deltas[2])
				};
			std::array<double, 3u>::const_iterator const itMax
				{ std::max_element(mags.cbegin(), mags.cend()) };

			// set return and check against tolerance
			maxMag = *itMax;
			same = (maxMag < tol);
		}
		if (ptMaxMag)
		{
			*ptMaxMag = maxMag;
		}
		return same;
	}

	/*! \brief True if both transforms produce similar output.
	 *
	 * Specifically, maxMagResultDifference() is used to compute the
	 * the maximum magnitude difference that occurs when six basis
	 * vectors are transformed with each argument. This maxmag
	 * difference value is used to determine similarity of
	 * operational results. Specifically, the two transforms are
	 * considered to produce similar results if the computed
	 * maximum magnitude is less than the provided tolerance value.
	 *
	 * Example:
	 * \snippet test_nearness.cpp DoxyExample01
	 */
	inline
	bool
	similarResult
		( rigibra::Transform const & xfm1
		, rigibra::Transform const & xfm2
		, bool const & useNormalizedCompare
		, double const & tol = std::numeric_limits<double>::epsilon()
		, double * const & ptMaxMag = nullptr
		)
	{
		bool same{ false };
		double maxMag{ engabra::g3::null<double>() };
		if (isValid(xfm1) && isValid(xfm2))
		{
			// compute max delta
			maxMag = maxMagResultDifference(xfm1, xfm2, useNormalizedCompare);
			// compare with specified tolerance
			same = (maxMag < tol);
		}
		if (ptMaxMag)
		{
			*ptMaxMag = maxMag;
		}
		return same;
	}

	//! \brief Statistics comparing collection of transforms to some other one.
	struct Stats
	{
		std::size_t theNumSamps{ 0u };
		double theMinMagDiff{ engabra::g3::null<double>() };
		double theMedMagDiff{ engabra::g3::null<double>() };
		double theAveMagDiff{ engabra::g3::null<double>() };
		double theMaxMagDiff{ engabra::g3::null<double>() };

	}; // Stats

	/*! \brief Compute statistics for collection of xforms relative to refXform
	 *
	 * \note (*FwdIter) must resolve to rigibra::Transform.
	 */
	template <typename FwdIter>
	inline
	Stats
	differenceStats
		( FwdIter const & itBeg
		, FwdIter const & itEnd
		, rigibra::Transform const & refXform
		, bool const & normalize = false
		)
	{
		Stats stats{};
		long int const itDiff{ (long int)std::distance(itBeg, itEnd) };
		if (0 < itDiff)
		{
			std::size_t const size{ static_cast<std::size_t>(itDiff) };
			std::vector<double> mags;
			mags.reserve(size);
			double min{ std::numeric_limits<double>::max() };
			double max{ -1. };
			double sum{ 0. };
			for (FwdIter iter{itBeg} ; itEnd != iter ; ++iter)
			{
				rigibra::Transform const & anXform = *iter;
				double const mag
					{ maxMagResultDifference(anXform, refXform, normalize) };
				min = std::min(min, mag);
				max = std::max(max, mag);
				sum = sum + mag;
				mags.emplace_back(mag);
			}
			double const ave{ (1./(double)size) * sum };
			double const med{ robust::medianOf(mags) };
			stats = Stats{ size, min, med, ave, max };
		}
		return stats;
	}

} // [compare]

} // [orinet]


#endif // OriNet_compare_INCL_

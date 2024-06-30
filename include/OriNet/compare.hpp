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


#include <Rigibra>

#include <algorithm>
#include <limits>


/*! \file
\brief Utilities for comparing transformation results

*/


namespace orinet
{

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
		)
	{
		double maxMag{ engabra::g3::null<double>() };

		if (isValid(xfm1) && isValid(xfm2))
		{
			using namespace engabra::g3;

			// R*(x-t)*Rr
			// R*x*Rr - R*t*Rr
			// +/- R*ek*Rr - R*t*Rr
			// (-R*t*Rr) +/- R*ek*Rr 

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

			std::array<Vector, 6u> const diffs
				{ ( into_pe1_1 - into_pe1_2 )
				, ( into_pe2_1 - into_pe2_2 )
				, ( into_pe3_1 - into_pe3_2 )
				, ( into_ne1_1 - into_ne1_2 )
				, ( into_ne2_1 - into_ne2_2 )
				, ( into_ne3_1 - into_ne3_2 )
				};

#if 0
			// transform attitude changes for each of the +/- each basis vector
			Vector const into_e1_1{ xfm1.theAtt(e1) };
			Vector const into_e1_2{ xfm2.theAtt(e1) };

			Vector const into_e2_1{ xfm1.theAtt(e2) };
			Vector const into_e2_2{ xfm2.theAtt(e2) };

			Vector const into_e3_1{ xfm1.theAtt(e3) };
			Vector const into_e3_2{ xfm2.theAtt(e3) };


			// apply rotation to the translation components of transformations
			/*
			Vector const into_rtr_1{ -xfm1.theAtt(xfm1.theLoc) };
			Vector const into_rtr_2{ -xfm2.theAtt(xfm2.theLoc) };
			Vector const into_rtr_delta{ into_rtr_2 - into_rtr_1 };
			*/
			Vector const into_rtr_1{ xfm1.theAtt(xfm1.theLoc) };
			Vector const into_rtr_2{ xfm2.theAtt(xfm2.theLoc) };
			Vector const into_rtr_delta{ into_rtr_1 - into_rtr_2 };

			// compute corresponding vector differences
			/*
			std::array<Vector, 6u> const 
				{ (into_rtr_2 + into_e1_2) - (into_rtr_1 + into_e1_1)
				, (into_rtr_2 - into_e1_2) - (into_rtr_1 - into_e1_1)
				, (into_rtr_2 + into_e2_2) - (into_rtr_1 + into_e2_1)
				, (into_rtr_2 - into_e2_2) - (into_rtr_1 - into_e2_1)
				, (into_rtr_2 + into_e3_2) - (into_rtr_1 + into_e3_1)
				, (into_rtr_2 - into_e3_2) - (into_rtr_1 - into_e3_1)
				};

			std::array<Vector, 6u> const 
				{ (into_rtr_2 - into_rtr_1) + into_e1_2 - into_e1_1
				, (into_rtr_2 - into_rtr_1) - into_e1_2 + into_e1_1
				, (into_rtr_2 - into_rtr_1) + into_e2_2 - into_e2_1
				, (into_rtr_2 - into_rtr_1) - into_e2_2 + into_e2_1
				, (into_rtr_2 - into_rtr_1) + into_e3_2 - into_e3_1
				, (into_rtr_2 - into_rtr_1) - into_e3_2 + into_e3_1
				};
			std::array<Vector, 6u> const diffs
				{ into_rtr_delta + into_e1_2 - into_e1_1
				, into_rtr_delta - into_e1_2 + into_e1_1
				, into_rtr_delta + into_e2_2 - into_e2_1
				, into_rtr_delta - into_e2_2 + into_e2_1
				, into_rtr_delta + into_e3_2 - into_e3_1
				, into_rtr_delta - into_e3_2 + into_e3_1
				};
			*/
			Vector const into_e1_delta{ into_e1_2 - into_e1_1 };
			Vector const into_e2_delta{ into_e2_2 - into_e2_1 };
			Vector const into_e3_delta{ into_e3_2 - into_e3_1 };

			std::array<Vector, 6u> const diffs
				{ into_rtr_delta + into_e1_delta
				, into_rtr_delta - into_e1_delta
				, into_rtr_delta + into_e2_delta
				, into_rtr_delta - into_e2_delta
				, into_rtr_delta + into_e3_delta
				, into_rtr_delta - into_e3_delta
				};
#endif

			/*
			//std::cout << '\n';
			std::cout << " lib:diff: ";
			for (Vector const & diff : diffs)
			{
				std::cout << "  " << diff;
			}
			std::cout << '\n';
			*/

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
		, double const & tol = std::numeric_limits<double>::epsilon()
		, double * const & ptMaxMag = nullptr
		)
	{
		bool same{ false };
		double maxMag{ engabra::g3::null<double>() };
		if (isValid(xfm1) && isValid(xfm2))
		{
			// compute max delta
			maxMag = maxMagResultDifference(xfm1, xfm2);
			// compare with specified tolerance
			same = (maxMag < tol);
		}
		if (ptMaxMag)
		{
			*ptMaxMag = maxMag;
		}
		return same;
	}

} // [orinet]


#endif // OriNet_compare_INCL_

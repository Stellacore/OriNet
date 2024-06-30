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


/*! \file
\brief Unit tests (and example) code for Rigibra::NS::CN
*/


#include "OriNet/compare.hpp"

#include "random.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <sstream>


namespace
{
	//! Check computation of transform result differences
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample00]

		// check maxMagResultDifference for many pseudo-random transforms
		constexpr std::size_t numXforms{ 32u*1024u };
		for (std::size_t numXform{0u} ; numXform < numXforms ; ++numXform)
		{
			// Generate two transforms (the x[12]w0) to compare
			rigibra::Transform const x1w0{ orinet::aRandomTransform() };
			rigibra::Transform const x2w1{ orinet::aRandomTransform() };
			rigibra::Transform const x2w0{ x2w1 * x1w0 };

			// get max mag result error between transforms
			using namespace engabra::g3;
			// Note the error associated with zero should be less than
			// the error associated with the +/- e_k values and therefore
			// not affect the test case. (error at the transformed zero
			// should be an interpolation of the error at the extrema)
			std::array<Vector, 7u> const locFroms
				{ -e1, -e2, -e3, zero<Vector>(), e1, e2, e3 };
			double expMaxMag{ -1 };
			// evaluate the difference with explicit application of
			// transformation operator()
			for (Vector const & locFrom : locFroms)
			{
				Vector const into1{ x1w0(locFrom) };
				Vector const into2{ x2w0(locFrom) };
				Vector const diff{ into2 - into1 };
				expMaxMag = std::max(expMaxMag, magnitude(diff));
			}

			// get computed values (which use abreviated formulae)
			double const gotMaxMag
				{ orinet::maxMagResultDifference(x1w0, x2w0) };

			// check if the two methods of computation agree
			// allow for computation noise (quadratic operations in attitude)
			constexpr double tol
				{ 128. * std::numeric_limits<double>::epsilon() };
			if (! nearlyEquals(gotMaxMag, expMaxMag, tol))
			{
				double const difMaxMag{ gotMaxMag - expMaxMag };
				oss << "Failure of maxMagResultDifference() test\n";
				oss << "exp: " << expMaxMag << '\n';
				oss << "got: " << gotMaxMag << '\n';
				oss << "dif: " << engabra::g3::io::enote(difMaxMag) << '\n';
				oss << "tol: " << engabra::g3::io::enote(tol) << '\n';
				break;
			}
		}

		// [DoxyExample00]
	}

	//! Examples for documentation
	void
	test1
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// create a first arbitrary rigid transform
		rigibra::Transform const xform1
			{ engabra::g3::Vector{ 1.1, 1.2, 1.3 }
			, rigibra::Attitude(rigibra::PhysAngle{ 2.1, 2.2, 2.3 })
			};

		// create a second arbitrary rigid transform - distinctly different
		rigibra::Transform const xform2
			{ engabra::g3::Vector{ 1.6, 1.5, 1.4 }
			, rigibra::Attitude(rigibra::PhysAngle{ 2.6, 2.5, 2.4 })
			};

		// transform collection of vectors using each of the two transforms
		using namespace engabra::g3;
		std::array<Vector, 7u> const locFroms
			{ -e1, -e2, -e3, zero<Vector>(), e1, e2, e3 };
		// compute maximum magnitude vector difference of transformed vectors
		// maxMag is a measure of maximum difference between transform results
		double maxMag{ -1 };
		for (Vector const & locFrom : locFroms)
		{
			Vector const into1{ xform1(locFrom) };
			Vector const into2{ xform2(locFrom) };
			Vector const diff{ into2 - into1 };
			maxMag = std::max(maxMag, magnitude(diff));
		}

		// compute the nearness of the transformations...

		// ... for case where tolerance is less than actual maxMax error
		double const tolA{ maxMag - 1./1024. };
		constexpr bool expNearA{ false };

		bool const gotNearA{ orinet::similarResult(xform1, xform2, tolA) };
		if (! (expNearA == gotNearA))
		{
			oss << "Failure of similarResult test 'A'\n";
			oss << "exp: " << std::boolalpha << expNearA << '\n';
			oss << "got: " << std::boolalpha << gotNearA << '\n';
			oss << "tol: " << io::fixed(tolA) << '\n';
			oss << "mag: " << io::fixed(maxMag) << '\n';
		}

		// ... for case where tolerance is more than actual maxMax error
		double const tolB{ maxMag + 1./1024. };
		constexpr bool expNearB{ true };
		bool const gotNearB{ orinet::similarResult(xform1, xform2, tolB) };
		if (! (expNearB == gotNearB))
		{
			oss << "Failure of similarResult test 'B'\n";
			oss << "exp: " << std::boolalpha << expNearB << '\n';
			oss << "got: " << std::boolalpha << gotNearB << '\n';
			oss << "tol: " << io::fixed(tolB) << '\n';
			oss << "mag: " << io::fixed(maxMag) << '\n';
		}


		// irrespective of similarity testing the two transforms
		// are not "nearly" the same in the sense of numeric representations
		if (  nearlyEquals(xform1, xform2))
		{
			oss << "Failure: of nearlyEquals() self test\n";
			oss << "xform1: " << xform1 << '\n';
		}

		// [DoxyExample01]
	}

}

//! Check behavior of NS
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);
	test1(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}


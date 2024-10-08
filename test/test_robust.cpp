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
\brief Unit tests (and example) code for OriNet::robust
*/


#include "OriNet/compare.hpp"
#include "OriNet/random.hpp" // for simulation support
#include "OriNet/robust.hpp"

#include <Rigibra>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <set> // for development info
#include <sstream>


namespace
{
	//! \brief Compute maxMagResult of xforms relative to expXform
	inline
	double
	maxMagDifferenceFor
		( std::vector<rigibra::Transform>::const_iterator const & itBeg
		, std::vector<rigibra::Transform>::const_iterator const & itEnd
		, rigibra::Transform const & expXform
		)
	{
		constexpr bool norm{ false };
		using namespace orinet::compare;
		Stats const stats{ differenceStats(itBeg, itEnd, expXform, norm) };
		return stats.theMaxMagDiff;
	}

	//! Examples for documentation - evaluate once
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		//
		// Simulate data
		//

		constexpr std::size_t numMea{ 3u };
		constexpr std::size_t numErr{ 2u };
		std::vector<rigibra::Transform> xforms;
		xforms.reserve(numMea + numErr);

		// simulate collection of rigibra::transform measurements including...
		engabra::g3::Vector const expLoc{ 1.2, 2.3, 3.4 };
		rigibra::PhysAngle const expAng{ 3.2, 2.1, 1.3 };

		// robustly estimated result should be 'near' to this
		// with 'near' relative to normal error model
		rigibra::Transform const expXform
			{ expLoc, rigibra::Attitude(expAng) };

		// artificial measurement errors, as fraction of typical values
		constexpr double sigmaLoc{ (1./100.) * 1.5 }; // e.g. cm at 1.5 m
		constexpr double sigmaAng{ (5./1000.) }; // e.g. 5 pix at 1000 pix

		// ... a number of typical measurements - with Gaussian noise
		for (std::size_t nn{0u} ; nn < numMea ; ++nn)
		{
			rigibra::Transform const meaXform
				{ orinet::random::perturbedTransform
					(expLoc, expAng, sigmaLoc, sigmaAng)
				};
			xforms.emplace_back(meaXform);
		}

		// ... a few 'blunderous' measurements - from uniform probability
		for (std::size_t nn{0u} ; nn < numErr ; ++nn)
		{
			using engabra::g3::pi;
			std::pair<double, double> const locMinMax{ -10., 10. };
			std::pair<double, double> const angMinMax{ -pi, pi };
			rigibra::Transform const errXform
				{ orinet::random::uniformTransform(locMinMax, angMinMax) };
			xforms.emplace_back(errXform);
		}

		//
		// Get result of robust estimation
		//

		// Fit via median of transform location and angle components
		// NOTE: this function is only appropriate for small rotations
		rigibra::Transform const gotXform
			{ orinet::robust::transformViaParameters
				(xforms.cbegin(), xforms.cend())
			};

		// estimate expected variability of transform effects
		double const estMaxMag
			{ maxMagDifferenceFor
				( xforms.cbegin()
				, xforms.cbegin() + numMea // use only 'measured' ones
				, expXform
				)
			};

		double const tol{ estMaxMag };
		constexpr bool useNorm{ false };
		double gotMaxMag; // set in function next line
		bool const okay
			{ orinet::compare::similarResult
				(gotXform, expXform, useNorm, tol, &gotMaxMag)
			};
		if (! okay)
		{
			double const ratio{ gotMaxMag / estMaxMag };
			oss << "Failure of robust fit to mea+err data\n";
			oss << "   numMea: " << numMea << '\n';
			oss << "   numErr: " << numErr << '\n';
			for (rigibra::Transform const & xform : xforms)
			{
				oss << " xform: " << xform << '\n';
			}
			using namespace engabra::g3::io;
			oss << "   sigLoc: " << fixed(sigmaLoc) << '\n';
			oss << "   sigAng: " << fixed(sigmaAng) << '\n';
			oss << "      tol: " << fixed(tol) << '\n';
			oss << "gotMaxMag: " << fixed(gotMaxMag) << '\n';
			oss << "estMaxMag: " << fixed(estMaxMag) << '\n';
			oss << "    ratio: " << fixed(ratio) << '\n';
			oss << "      exp: " << expXform << '\n';
			oss << "      got: " << gotXform << '\n';
		}

		// [DoxyExample01]

		if (! true)
		{
			oss << "Failure: Implement template test case\n";
			oss << "exp: " << "" << '\n';
			oss << "got: " << "" << '\n';
		}
	}

	//! Test several cases
	void
	test1
		( std::ostream & oss
		)
	{
		// Test needs a larger number of measurement/errors for statistics
		// to stabilize (for estimating expected measurement noise which
		// is used in test condition below).
		constexpr std::size_t numTrials{ 32u*1024u };
		constexpr std::size_t numMea{ 15u };
		constexpr std::size_t numErr{ 10u };
		constexpr double tolFactor{ 3. }; // ? hard to say what this should be
		constexpr double sigmaLoc{ (1./100.) * 1.5 }; // e.g. cm at 1.5 m
		constexpr double sigmaAng{ (5./1000.) }; // e.g. 5 pix at 1000 pix
		constexpr bool showStats{ false };

		double maxRatio(-1.);
		// perform large number of pseudo-random trials
		std::set<std::size_t> testGoods;
		std::set<std::size_t> testFails;
		for (std::size_t numTrial{0u} ; numTrial < numTrials ; ++numTrial)
		{
			// establish an arbitrary starting transform test case
			using engabra::g3::pi;
			std::pair<double, double> const locMinMax{ -2., 2. };
			std::pair<double, double> const angMinMax{ -pi, pi };
			rigibra::Transform const expXform
				{ orinet::random::uniformTransform(locMinMax, angMinMax) };

			// [DoxyExample02]

			// simulate noisy observation data for this test case
			std::vector<rigibra::Transform> const xforms
				{ orinet::random::noisyTransforms
					(expXform, numMea, numErr, sigmaLoc, sigmaAng, locMinMax)
				};

			// obtain robustly estimated transformation
			// Fit via median of transformation *results*
			// NOTE: this function is appropriate for any size rotation
			rigibra::Transform const gotXform
				{ orinet::robust::transformViaEffect
					(xforms.cbegin(), xforms.cend())
				};

			// [DoxyExample02]

			// estimate expected variability of transform effects
			double const estMaxMag
				{ maxMagDifferenceFor
					( xforms.cbegin()
					, xforms.cbegin() + numMea // use only 'measured' ones
					, expXform
					)
				};

			double const tol{ tolFactor * estMaxMag };
			constexpr bool useNorm{ false };
			double gotMaxMag; // set in function next line
			bool const okay
				{ orinet::compare::similarResult
					(gotXform, expXform, useNorm, tol, &gotMaxMag)
				};

			if (showStats)
			{
				double const ratio{ gotMaxMag / estMaxMag };
				maxRatio = std::max(maxRatio, ratio);
				using namespace engabra::g3;
				std::cout
					<< "estMaxMag(1): " << io::fixed(estMaxMag)
					<< "  gotMaxMag: " << io::fixed(gotMaxMag)
					<< "  ratio: " << io::fixed(ratio)
					<< "  maxRatio: " << io::fixed(maxRatio)
					<< '\n';
			}

			if (! okay)
			{
				testFails.insert(numTrial);
				double const ratio{ gotMaxMag / estMaxMag };
				oss << '\n';
				oss << "Failure of robust fit trial no. " << numTrial << '\n';
				oss << "   numMea: " << numMea << '\n';
				oss << "   numErr: " << numErr << '\n';
				for (rigibra::Transform const & xform : xforms)
				{
					oss << " xform: " << xform << '\n';
				}
				using namespace engabra::g3::io;
				oss << "   sigLoc: " << fixed(sigmaLoc) << '\n';
				oss << "   sigAng: " << fixed(sigmaAng) << '\n';
				oss << "      tol: " << fixed(tol) << '\n';
				oss << "gotMaxMag: " << fixed(gotMaxMag) << '\n';
				oss << "estMaxMag: " << fixed(estMaxMag) << '\n';
				oss << "    ratio: " << fixed(ratio) << '\n';
				oss << "      exp: " << expXform << '\n';
				oss << "      got: " << gotXform << '\n';
			//	break;
			}
			else
			{
				testGoods.insert(numTrial);
			}

			/*
			std::cout << '\n';
			for (rigibra::Transform const & xform : xforms)
			{
				using engabra::g3::magnitude;
				double const magAng
					{ magnitude(xform.theAtt.physAngle().theBiv) };
				std::cout << "xform: " << xform
					<< "  magAng: " << magAng
					<< '\n';
			}
			*/
		}

		if (! testFails.empty())
		{
			oss << '\n';
			oss << "Good Count: " << std::setw(5u) << testGoods.size()
				<< "  Good Percent: "
				<< 100.*(double)testGoods.size()/(double)numTrials
				<< '\n';
			oss << "Fail Count: " << std::setw(5u) << testFails.size()
				<< "  Fail Percent: "
				<< 100.*(double)testFails.size()/(double)numTrials
				<< '\n';
		}

	}

	void
	checkMedian
		( std::ostream & oss
		, std::vector<double> const & vals
		, double const & expMedian
		, std::string const & tname
		)
	{
		std::vector<double> tmps(vals.cbegin(), vals.cend());
		// randomly shuffle values
		static std::mt19937 gen{33288566u};
		std::shuffle(tmps.begin(), tmps.end(), gen);

		// extract median
		double const gotMedian{ orinet::robust::medianOf(tmps) };

		// check for case where not expecting answer (e.g. no data)
		bool okay{ false };
		if (! engabra::g3::isValid(expMedian))
		{
			if (! engabra::g3::isValid(gotMedian))
			{
				okay = true;
			}
		}
		else
		{
			okay = engabra::g3::nearlyEquals(gotMedian, expMedian);
		}

		if (! okay)
		{
			oss << "Failure of medianOf test " << tname << '\n';
			oss << "exp: " << expMedian << '\n';
			oss << "got: " << gotMedian << '\n';
		}

	}

	//! Test medianOf() function
	void
	test2
		( std::ostream & oss
		)
	{
		std::vector<double> const v0{                       };
		std::vector<double> const v1{           0.          };
		std::vector<double> const v2{        -1., 1.        };
		std::vector<double> const v3{      -2., 0., 5.      };
		std::vector<double> const v4{    -7, -1., 1., 3.    };
		std::vector<double> const v5{  -4, -3., 0., 4., 5.  };
		std::vector<double> const v6{-8, -7, -1., 1., 3., 9 };

		static double const nan{ engabra::g3::null<double>() };
		checkMedian(oss, v0, nan, "v0");
		checkMedian(oss, v1,  0., "v1");
		checkMedian(oss, v2,  0., "v2");
		checkMedian(oss, v3,  0., "v3");
		checkMedian(oss, v4,  0., "v4");
		checkMedian(oss, v5,  0., "v5");
		checkMedian(oss, v6,  0., "v6");
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
	test2(oss);

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


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

#include <cmath>
#include <iostream>
#include <set> // for development info
#include <sstream>


namespace
{
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
				{ orinet::rand::perturbedTransform
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
				{ orinet::rand::uniformTransform(locMinMax, angMinMax) };
			xforms.emplace_back(errXform);
		}

		//
		// Get result of robust estimation
		//

		using orinet::transform::robustViaParameters;
		rigibra::Transform const gotXform
			{ robustViaParameters(xforms.cbegin(), xforms.cend()) };

		constexpr double numSigmas{ 3. }; // test case sensitive
		using orinet::rand::sigmaMagForSigmaLocAng;
		double const tol
			{ numSigmas * sigmaMagForSigmaLocAng(sigmaLoc, sigmaAng) };
		if (! orinet::similarResult(gotXform, expXform, false, tol) )
		{
			oss << "Failure of robust fit to mea+err data\n";
			oss << "numMea: " << numMea << '\n';
			oss << "numErr: " << numErr << '\n';
			for (rigibra::Transform const & xform : xforms)
			{
				oss << " xform: " << xform << '\n';
			}
			oss << "sigmaLoc: " << engabra::g3::io::fixed(sigmaLoc) << '\n';
			oss << "sigmaAng: " << engabra::g3::io::fixed(sigmaAng) << '\n';
			oss << "     tol: " << engabra::g3::io::fixed(tol) << '\n';
			oss << "exp: " << expXform << '\n';
			oss << "got: " << gotXform << '\n';
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
//		constexpr std::size_t numTrials{ 32u*1024u };
std::cout << "TODO: update test cases\n";
constexpr std::size_t numTrials{ 10u };
		constexpr std::size_t numMea{ 3u };
		constexpr std::size_t numErr{ 2u };
		constexpr double sigmaLoc{ (1./100.) * 1.5 }; // e.g. cm at 1.5 m
		constexpr double sigmaAng{ (5./1000.) }; // e.g. 5 pix at 1000 pix

		std::set<std::size_t> testGoods;
		std::set<std::size_t> testFails;
		for (std::size_t numTrial{0u} ; numTrial < numTrials ; ++numTrial)
		{
			// establish an arbitrary starting transform test case
			using engabra::g3::pi;
			std::pair<double, double> const locMinMax{ -10., 10. };
			std::pair<double, double> const angMinMax{ -pi, pi };
			rigibra::Transform const expXform
				{ orinet::rand::uniformTransform(locMinMax, angMinMax) };

			// simulate noisy observation data for this test case
			std::vector<rigibra::Transform> const xforms
				{ orinet::rand::noisyTransforms
					(expXform, numMea, numErr, sigmaLoc, sigmaAng)
				};

			// obtain robustly estimated transformation
			using orinet::transform::robustViaParameters;
			rigibra::Transform const gotXform
				{ robustViaParameters(xforms.cbegin(), xforms.cend()) };

			// TODO - figure out what distribution and DOFs are involved.
			constexpr double numSigmas{ 4. }; // test case sensitive
			using engabra::g3::sq;
			double const estSigma
				{ orinet::rand::sigmaMagForSigmaLocAng(sigmaLoc, sigmaAng) };
			double const tol{ numSigmas * estSigma };
			double maxMag; // set in function next line
			using orinet::similarResult;
			if (! similarResult(gotXform, expXform, false, tol, &maxMag) )
			{
				testFails.insert(numTrial);
				double const ratio{ maxMag / estSigma };
				oss << '\n';
				oss << "Failure of robust fit trial no. " << numTrial << '\n';
				oss << "numMea: " << numMea << '\n';
				oss << "numErr: " << numErr << '\n';
				for (rigibra::Transform const & xform : xforms)
				{
					oss << " xform: " << xform << '\n';
				}
				oss << "sigLoc: " << engabra::g3::io::fixed(sigmaLoc) << '\n';
				oss << "sigAng: " << engabra::g3::io::fixed(sigmaAng) << '\n';
				oss << "   tol: " << engabra::g3::io::fixed(tol) << '\n';
				oss << "maxMag: " << engabra::g3::io::fixed(maxMag) << '\n';
				oss << "estSig: " << engabra::g3::io::fixed(estSigma) << '\n';
				oss << " ratio: " << engabra::g3::io::fixed(ratio) << '\n';
				oss << "   exp: " << expXform << '\n';
				oss << "   got: " << gotXform << '\n';
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


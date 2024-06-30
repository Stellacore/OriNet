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
#include "OriNet/robust.hpp"

#include "random.hpp" // for simulation support

#include <Rigibra>

#include <cmath>
#include <iostream>
#include <set> // for development info
#include <sstream>


namespace sim
{

	/*! \brief Simulate observation data including measurements and blunders
	 *
	 * The collection of transformations include samples from two
	 * populations.
	 *
	 * The first population generates multiple simulated "measured"
	 * transforms in which each instance should be "near" to the provided
	 * expXform one with the discrepancy determined by normally
	 * distributed (pseudo)random noise having deviation sigmaLoc on
	 * the position and sigmaAng on the angle components.
	 *
	 * The second population represents blunder transformations which
	 * are created from component data values that uniformly span the
	 * range of allowed values (as specified in the function
	 * orinet::rand::uniformTransform().
	 */
	std::vector<rigibra::Transform>
	noisyTransforms
		( rigibra::Transform const & expXform
		, std::size_t const & numMea = 3u
		, std::size_t const & numErr = 2u
		, double const & sigmaLoc = ((1./100.) * 1.5) // e.g. cm at 1.5 m
		, double const & sigmaAng = ((5./1000.)) // e.g. 5 pix at 1000 pix
		)
	{
		std::vector<rigibra::Transform> xforms;
		xforms.reserve(numMea + numErr);

		engabra::g3::Vector const expLoc = expXform.theLoc;
		rigibra::PhysAngle const expAng{ expXform.theAtt.physAngle() };

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
			rigibra::Transform const errXform
				{ orinet::rand::uniformTransform() };
			xforms.emplace_back(errXform);
		}

		return xforms;
	}

} // [sim]


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
			rigibra::Transform const errXform
				{ orinet::rand::uniformTransform() };
			xforms.emplace_back(errXform);
		}

		//
		// Get result of robust estimation
		//

		rigibra::Transform const gotXform
			{ orinet::robustTransformFrom(xforms.cbegin(), xforms.cend()) };

		// Unclear what's best, but following seems not unreasonable.
		// Error in location should translate directly to maxMag result
		// error.  Error in angle components should change maxMag by
		// angle error time basis vector length (unity). Therefore,
		// the two errors seem like they should add 'rmse' style.
		constexpr double numSigmas{ 3. }; // test case sensitive
		double const tol
			{ numSigmas * std::sqrt(sigmaLoc*sigmaLoc + sigmaAng*sigmaAng) };
		if (! orinet::similarResult(gotXform, expXform, tol) )
		{
			oss << "Failure of robust fit to mea+err data\n";
			oss << "numMea: " << numMea << '\n';
			oss << "numErr: " << numErr << '\n';
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
		constexpr std::size_t numMea{ 3u };
		constexpr std::size_t numErr{ 2u };
		constexpr double sigmaLoc{ (1./100.) * 1.5 }; // e.g. cm at 1.5 m
		constexpr double sigmaAng{ (5./1000.) }; // e.g. 5 pix at 1000 pix

std::set<std::size_t> testGoods;
std::set<std::size_t> testFails;
std::size_t const numTrials{ 1024u };
		for (std::size_t numTrial{0u} ; numTrial < numTrials ; ++numTrial)
		{
			// establish an arbitrary starting transform test case
			rigibra::Transform
				const expXform{ orinet::rand::uniformTransform() };

			// simulate noisy observation data for this test case
			std::vector<rigibra::Transform> const xforms
				{ sim::noisyTransforms
					(expXform, numMea, numErr, sigmaLoc, sigmaAng)
				};

			// obtain robustly estimated transformation
			rigibra::Transform const gotXform
				{ orinet::robustTransformFrom
					(xforms.cbegin(), xforms.cend())
				};

			// Unclear what's best, but following seems not unreasonable.
			// Error in location should translate directly to maxMag result
			// error.  Error in angle components should change maxMag by
			// angle error time basis vector length (unity). Therefore,
			// the two errors seem like they should add 'rmse' style.
			constexpr double numSigmas{ 3. }; // test case sensitive
			using engabra::g3::sq;
			double const tol
				{ numSigmas * std::sqrt(sq(sigmaLoc) + sq(sigmaAng)) };
			double maxMag; // set in function
			if (! orinet::similarResult(gotXform, expXform, tol, &maxMag) )
			{
testFails.insert(numTrial);
				oss << '\n';
				oss << "Failure of robust fit trial no. " << numTrial << '\n';
				oss << "numMea: " << numMea << '\n';
				oss << "numErr: " << numErr << '\n';
				oss << "sigLoc: " << engabra::g3::io::fixed(sigmaLoc) << '\n';
				oss << "sigAng: " << engabra::g3::io::fixed(sigmaAng) << '\n';
				oss << "   tol: " << engabra::g3::io::fixed(tol) << '\n';
				oss << "maxMag: " << engabra::g3::io::fixed(maxMag) << '\n';
				oss << "   exp: " << expXform << '\n';
				oss << "   got: " << gotXform << '\n';
//				break;
			}
else
{
testGoods.insert(numTrial);
}

			/*
			std::cout << '\n';
			for (rigibra::Transform const & xform : xforms)
			{
				std::cout << "xform: " << xform << '\n';
			}
			*/
		}

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


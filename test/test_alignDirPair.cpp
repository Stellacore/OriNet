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
\brief Unit tests (and example) code for OriNet::alignPair
*/


#include "OriNet/align.hpp"

#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <iostream>
#include <sstream>


namespace
{

namespace sim
{
	//! \brief Generate random pairs of directions
	inline
	orinet::DirPair
	directionPair
		()
	{
		using namespace engabra::g3;
		orinet::DirPair dirPair{ null<Vector>(), null<Vector>() };
		for (;;)
		{
			using orinet::rand::randomDirection;
			Vector const aDir{ randomDirection() };
			Vector const bDir{ randomDirection() };
			double const dot{ (aDir * bDir).theSca[0] };
			constexpr double tol{ 1. - 1.e-9 }; // avoid (anti)parallel dirs
			if (std::abs(dot) < tol)
			{
				dirPair = std::make_pair(aDir, bDir);
				break;
			}
		}
		return dirPair;
	}

	//! \brief Generate 'noisy' body frame direction pair
	inline
	orinet::DirPair
	bodyDirectionPair
		( orinet::DirPair const & refDirPair
		, rigibra::Attitude const & attBodWrtRef
		)
	{
		using namespace engabra::g3;

		// measurements in reference frame
		Vector const & a0 = refDirPair.first;
		Vector const & b0 = refDirPair.second;

		// perturb measurements by random error (the 4th measurement DOM)
		// ref. theory/alignDifPairs.lyx
		static std::mt19937 gen(47562958u);
		std::uniform_real_distribution<> dist(1./1024., 1.);
		double const nu{ dist(gen) };
		double const wp{ 1. + nu };
		double const wn{ 1. - nu };
		// perturbed values that remain coplaner with (a0,b0)
		Vector const aTmp{ .5 * (wp * a0 + wn * b0) };
		Vector const bTmp{ .5 * (wn * a0 + wp * b0) };

		// measurements in body frame
		Vector const a1{ attBodWrtRef(aTmp) };
		Vector const b1{ attBodWrtRef(bTmp) };

		return std::make_pair(a1, b1);
	}

} // [sim]


	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// arbitrary rigid body attitude
		rigibra::Attitude const expAtt{ rigibra::PhysAngle{ 1., .5, -.7 } };

		// simulate measurement data
		using namespace engabra::g3;
		orinet::DirPair const refDirPair
			{ e1, direction(e1+e2) };
		orinet::DirPair const bodDirPair
			{ sim::bodyDirectionPair(refDirPair, expAtt) };

		rigibra::Attitude const gotAtt
			{ orinet::alignDirPair(refDirPair, bodDirPair) };

		// [DoxyExample01]

		if (! nearlyEquals(gotAtt, expAtt))
		{
			oss << "Failure of alignDirPair individual test\n";
			oss << "exp: " << expAtt << '\n';
			oss << "got: " << gotAtt << '\n';
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


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
\brief Unit tests (and example) code for OriNet::align::attitudeFromDirPairs
*/


#include "OriNet/align.hpp"

#include "OriNet/compare.hpp"
#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <iostream>
#include <sstream>


namespace
{
	//! \brief Angle between first and second direction in pair.
	inline
	engabra::g3::BiVector
	angleBetween
		( orinet::align::DirPair const & dirs
		)
	{
		using namespace engabra::g3;
		Vector const & d1 = dirs.first;
		Vector const & d2 = dirs.second;
		return logG2(d1 * d2).theBiv;
	}

	//! \brief Display info on internal angle between directions in pair.
	inline
	std::string
	dirInfo
		( orinet::align::DirPair const & dirs
		)
	{
		std::ostringstream oss;
		using namespace engabra::g3;
		BiVector const angle{ angleBetween(dirs) };
		oss << angle << "  mag: " << magnitude(angle);
		return oss.str();
	}


namespace sim
{
	//! \brief Generate random pairs of directions
	inline
	orinet::align::DirPair
	directionPair
		( std::pair<double, double> const & minMaxAngleMag = { .1, 3. }
		)
	{
		using namespace engabra::g3;
		orinet::align::DirPair dirPair{ null<Vector>(), null<Vector>() };
		for (;;)
		{
			Vector const aDir{ orinet::random::directionVector() };
			Vector const bDir{ orinet::random::directionVector() };

			BiVector const angle{ logG2(aDir * bDir).theBiv };
			double const angleMag{ magnitude(angle) };

			// avoid (anti)parallel dirs
			double const & minAngleMag = minMaxAngleMag.first;
			double const & maxAngleMag = minMaxAngleMag.second;
			if ((minAngleMag < angleMag) && (angleMag < maxAngleMag))
			{
				dirPair = std::make_pair(aDir, bDir);
				break;
			}
		}
		return dirPair;
	}

	//! \brief Generate 'noisy' body frame direction pair
	inline
	orinet::align::DirPair
	bodyDirectionPair
		( orinet::align::DirPair const & refDirPair
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
		std::uniform_real_distribution<> dist(1./128., 32./128.);
		double const nu{ dist(gen) };
		double const wp{ 1. + nu };
		double const wn{ 1. - nu };
		// perturbed values that remain coplaner with (a0,b0)
		Vector const aTmp{ direction(.5 * (wp * a0 + wn * b0)) };
		Vector const bTmp{ direction(.5 * (wn * a0 + wp * b0)) };

		// measurements in body frame
		Vector const a1{ attBodWrtRef(aTmp) };
		Vector const b1{ attBodWrtRef(bTmp) };

		return std::make_pair(a1, b1);
	}

} // [sim]

	//! Compare two attitude instances and report details if different
	inline
	void
	checkAtt
		( std::ostream & oss
		, rigibra::Attitude const & expAtt
		, rigibra::Attitude const & gotAtt
		, std::string const & testName
		, orinet::align::DirPair const & refDirs = {}
		, orinet::align::DirPair const & bodDirs = {}
		)
	{
		// reconstruction of test case can be sensistive (e.g. for
		// very different sizes of the two included angle sizes)
		// and for attitude cases that represent near a half turn
		// Since alignment algorithm involves quadratic products, it
		// seems reasonable to set this to sqrt of machine epsilon.
		// In practice, errors seem to be less than about 1e-11.
		static double const tol
			{ std::sqrt(std::numeric_limits<double>::epsilon()) };

		using namespace rigibra;
		using namespace engabra::g3;
		double maxMag;
		if (! orinet::similarResult(gotAtt, expAtt, tol, &maxMag))
		{
			PhysAngle const gotPhys{ gotAtt.physAngle() };
			PhysAngle const expPhys{ expAtt.physAngle() };
			BiVector const diffPhys{ gotPhys.theBiv - expPhys.theBiv };
			oss << '\n';
			oss << "Failure of " << testName << " test\n";
			oss << "    exp: " << expAtt
				<< "  mag: " << magnitude(expPhys.theBiv) << '\n';
			oss << "    got: " << gotAtt
				<< "  mag: " << magnitude(gotPhys.theBiv) << '\n';
			oss << "refDirs: " << refDirs
				<< "  incl.Angle: " << dirInfo(refDirs)
				<< '\n';
			oss << "bodDirs: " << bodDirs
				<< "  incl.Angle: " << dirInfo(bodDirs)
				<< '\n';
			oss << "    dif: " << io::enote(diffPhys) << '\n';
			oss << "    tol: " << io::enote(tol) << '\n';
			oss << " maxMag: " << io::enote(maxMag) << '\n';
			constexpr double eps{ std::numeric_limits<double>::epsilon() };
			oss << "  ratio: " << io::fixed(maxMag/eps) << '\n';
		}
	}

	//! Check simple canse and provide example for documentation
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
		orinet::align::DirPair const refDirPair
			{ e1, direction(e1+e2) };
		orinet::align::DirPair const bodDirPair
			{ sim::bodyDirectionPair(refDirPair, expAtt) };

		rigibra::Attitude const gotAtt
			{ orinet::align::attitudeFromDirPairs(refDirPair, bodDirPair) };

		// [DoxyExample01]

		checkAtt(oss, expAtt, gotAtt, "attitudeFromDirPairs individual");
	}

	//! check special cases
	void
	test1
		( std::ostream & oss
		)
	{
		// half turn rotation
		{
			// arbitrary rigid body attitude
			using namespace engabra::g3;
			rigibra::Attitude const expAtt{ rigibra::PhysAngle{ pi * e12 } };

			// simulate measurement data
			orinet::align::DirPair const refDirs{ e1, direction(e1+e2) };
			// exact 180 deg rotation
			orinet::align::DirPair const bodDirs
				{ expAtt(refDirs.first), expAtt(refDirs.second) };

			rigibra::Attitude const gotAtt
				{ orinet::align::attitudeFromDirPairs(refDirs, bodDirs) };

			checkAtt(oss, expAtt, gotAtt, "attitudeFromDirPairs pi*e12");
		}

		// no rotation
		{
			// arbitrary rigid body attitude
			using namespace rigibra;
			Attitude const expAtt{ identity<Attitude>() };

			// simulate measurement data
			using namespace engabra::g3;
			orinet::align::DirPair const refDirs{ e1, direction(e1+e2) };
			// exact 180 deg rotation
			orinet::align::DirPair const & bodDirs = refDirs;

			rigibra::Attitude const gotAtt
				{ orinet::align::attitudeFromDirPairs(refDirs, bodDirs) };

			checkAtt(oss, expAtt, gotAtt, "attitudeFromDirPairs identity");
		}
	}

	//! check large number of cases
	void
	test2
		( std::ostream & oss
		)
	{
		constexpr std::size_t numRuns{ 128u*1024u };

		for (std::size_t numRun{0u} ; numRun < numRuns ; ++numRun)
		{
			// arbitrary rigid body attitude
			using namespace engabra::g3;
			std::pair<double, double> const angMinMax{ -pi, pi };

			// simulate random test case
			using namespace rigibra;
			Attitude const expAtt{ orinet::random::uniformAttitude(angMinMax) };
			orinet::align::DirPair const refDirs{ sim::directionPair() };
			orinet::align::DirPair const bodDirs
				{ sim::bodyDirectionPair(refDirs, expAtt) };

			// compute best fit attitude
			rigibra::Attitude const gotAtt
				{ orinet::align::attitudeFromDirPairs(refDirs, bodDirs) };

			// check solution
			std::ostringstream tmsg;
			tmsg << "attitudeFromDirPairs volume run " << numRun;
			checkAtt(oss, expAtt, gotAtt, tmsg.str(), refDirs, bodDirs);
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


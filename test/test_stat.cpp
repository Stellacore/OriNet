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
\brief Unit tests (and example) code for OriNet::stat
*/


#include "OriNet/stat.hpp"

#include "OriNet/compare.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <iostream>
#include <random>
#include <sstream>


namespace
{
	inline
	void
	checkMedian
		( std::ostream & oss
		, orinet::stat::track::Values const stats
		, double const & expMedian
		, std::string const & tname
		)
	{
		bool okay{ false };
		double const gotMedian{ stats.median() };
		// check for nan,nan condition
		if (! engabra::g3::isValid(expMedian))
		{
			okay = (! engabra::g3::isValid(gotMedian));
		}
		else
		{
			// check for numerically same values
			okay = engabra::g3::nearlyEquals(gotMedian, expMedian);
		}
		if (! okay)
		{
			oss << "Failure of " << tname << "test\n";
			oss << "exp: " << expMedian << '\n';
			oss << "got: " << gotMedian << '\n';
		}
	}

	//! Test tracking of scalar values
	void
	test0
		( std::ostream & oss
		)
	{
		constexpr double nan{ engabra::g3::null<double>() };

		// add samples incrementally
		// { -8., -6., -1.,  1.,  3., 4., 9. };

		// Allocate (at least) enough space to hold all values
		// (to avoid reallocations)
		orinet::stat::track::Values stats(32u);
		checkMedian(oss, stats, nan, "empty");
		stats.insert(-8.);
		checkMedian(oss, stats, -8., "one value");
		stats.insert(-6.);
		checkMedian(oss, stats, -7., "two values");
		stats.insert( 9.);
		stats.insert(-1.);
		stats.insert( 3.);
		stats.insert( 1.);
		checkMedian(oss, stats,  0., "six values");
		stats.insert( 4.);
		checkMedian(oss, stats,  1., "seven values");
	}

	//! Test tracking of vectors
	void
	test1
		( std::ostream & oss
		)
	{
		using engabra::g3::Vector;
		std::vector<double> coordValues{ -8., -6., -1.,  1.,  3., 4., 9. };
		Vector expMedian{ 1., 1., 1. };

		std::vector<double> xvals{ coordValues };
		std::vector<double> yvals{ coordValues };
		std::vector<double> zvals{ coordValues };
		static std::mt19937 gen(44233674u);
		std::shuffle(xvals.begin(), xvals.end(), gen);
		std::shuffle(yvals.begin(), yvals.end(), gen);
		std::shuffle(zvals.begin(), zvals.end(), gen);
		std::vector<Vector> vecs;
		for (std::size_t nn{0u} ; nn < zvals.size() ; ++nn)
		{
			Vector const vec{ xvals[nn], yvals[nn], zvals[nn] };
			vecs.emplace_back(vec);
		}

		// [DoxyExample01]

		// For efficienty, pre-reserve enough space for anticipated use cases
		constexpr std::size_t reserveSize{ 16u };

		// use stats tracking on vectors
		orinet::stat::track::Vectors stats(reserveSize);
		Vector gotMedian{};
		for (engabra::g3::Vector const & vec : vecs)
		{
			stats.insert(vec);
			gotMedian = stats.median();
		}

		// [DoxyExample01]

		if (! engabra::g3::nearlyEquals(gotMedian, expMedian))
		{
			oss << "Failure of vector tracker median test\n";
			oss << "exp: " << expMedian << '\n';
			oss << "got: " << gotMedian << '\n';
		}
	}

	//! Test tracking of attitude poses
	void
	test2
		( std::ostream & oss
		)
	{
		// angle sizes for rotation about arbitrary plane
		double const scale{ .01 };
		std::vector<double> values{ -8., -6., 1.,  1.,  3., 4., 9. };
		double const valMedian{ 1. };
		static std::mt19937 gen(66637789u);
		std::shuffle(values.begin(), values.end(), gen);

		constexpr std::size_t reserveSize{ 16u };
		orinet::stat::track::Attitudes stats(reserveSize);

		using namespace rigibra;
		using namespace engabra::g3;

		// plane of rotation
		BiVector const rotDir{ direction( 2.*e23 + 3.*e31 - 4.*e12) };
		Attitude gotMedian{};
		for (std::size_t nn{0u} ; nn < values.size() ; ++nn)
		{
			double const angSize{ scale * values[nn] };
			PhysAngle const pAng{ angSize * rotDir };
			Attitude const att{ pAng };
			stats.insert(att);
			gotMedian = stats.median();
		}
		Attitude expMedian;
		{
			double const angSize{ scale * valMedian };
			PhysAngle const pAng{ angSize * rotDir };
			Attitude const att{ pAng };
			expMedian = att;
		}

		// Test method could probably be improved with some consideration

		Attitude const difMedian{ gotMedian * inverse(expMedian) };
		constexpr double tolMag{ .000100 };
		double const difMag{ magnitude(difMedian.physAngle().theBiv) };
		if (! (difMag < tolMag))
		{
			oss << "Failure of attitude tracker median test\n";
			oss << "exp: " << expMedian << '\n';
			oss << "got: " << gotMedian << '\n';
			oss << "dif: " << difMedian << '\n';
			oss << "difMag: " << io::fixed(difMag) << '\n';
			oss << "tolMag: " << io::fixed(tolMag) << '\n';
		}
	}

	//! Test tracking of attitude poses
	void
	test3
		( std::ostream & oss
		)
	{
		// angle sizes for rotation about arbitrary plane
		double const scale{ .01 };
		std::vector<double> values{ -8., -6., 1.,  1.,  3., 4., 9. };
		double const valMedian{ 1. };
		static std::mt19937 gen(36366525u);
		std::shuffle(values.begin(), values.end(), gen);

		std::vector<double> xvals{ values };
		std::vector<double> yvals{ values };
		std::vector<double> zvals{ values };
		std::shuffle(xvals.begin(), xvals.end(), gen);
		std::shuffle(yvals.begin(), yvals.end(), gen);
		std::shuffle(zvals.begin(), zvals.end(), gen);

		constexpr std::size_t reserveSize{ 16u };
		orinet::stat::track::Transforms stats(reserveSize);

		using namespace rigibra;
		using namespace engabra::g3;

		// plane of rotation
		BiVector const rotDir{ direction( 2.*e23 + 3.*e31 - 4.*e12) };
		Transform gotMedian{};
		for (std::size_t nn{0u} ; nn < values.size() ; ++nn)
		{
			// create a jumble of locations
			Vector const loc{ xvals[nn], yvals[nn], zvals[nn] };
			// create a jumble of attitudes
			double const angSize{ scale * values[nn] };
			PhysAngle const pAng{ angSize * rotDir };
			Transform const xform{ loc, Attitude{ pAng } };
			stats.insert(xform);
			gotMedian = stats.median();
		}
		Transform expMedian{};
		{
			// create a jumble of locations
			Vector const loc{ valMedian, valMedian, valMedian };
			// create a jumble of attitudes
			double const angSize{ scale * valMedian };
			PhysAngle const pAng{ angSize * rotDir };
			Transform const xform{ loc, Attitude{ pAng } };
			expMedian = xform;
		}

		// Test method could probably be improved with some consideration

		bool const okayLoc{ nearlyEquals(gotMedian.theLoc, expMedian.theLoc) };
		//
		Attitude const difMedian
			{ gotMedian.theAtt * inverse(expMedian.theAtt) };
		constexpr double tolAngMag{ .000100 };
		double const difAngMag{ magnitude(difMedian.physAngle().theBiv) };
		bool const okayAtt{ (difAngMag < tolAngMag) };
		//
		bool const okay{ okayLoc && okayAtt };
		//
		if (! okay)
		{
			oss << "Failure of attitude tracker median test\n";
			oss << "exp: " << expMedian << '\n';
			oss << "got: " << gotMedian << '\n';
			oss << "  okayLoc: " << okayLoc << '\n';
			oss << "  okayAtt: " << okayAtt << '\n';
			oss << "difAngMag: " << io::fixed(difAngMag) << '\n';
			oss << "tolAngMag: " << io::fixed(tolAngMag) << '\n';
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
	test3(oss);

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


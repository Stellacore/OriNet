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

#include <Engabra>
#include <Rigibra>

#include <iostream>
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

	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

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

		// [DoxyExample01]

//		stat::track::Vectors
//		stat::track::Attitudes

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


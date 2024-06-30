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


#ifndef OriNet_random_INCL_
#define OriNet_random_INCL_

/*! \file
\brief Functions for generating pseudo-random data values

Example:
\snippet test_CN.cpp DoxyExample01

*/


#include <Engabra>
#include <Rigibra>

#include <random>


namespace orinet
{
	//! A transformation with arbitrary parameters values
	inline
	rigibra::Transform
	aRandomTransform
		()
	{
		// Range of translations - plus/minus this limit
		constexpr double limLoc{ 10. };
		// Range of rotation angles - plus/minus this limit
		constexpr double limAng{ engabra::g3::pi };

		// Configure pseudo-random number distribution generator
		static std::mt19937 gen(7484020u);
		static std::uniform_real_distribution<> distLocs(-limLoc, limLoc);
		static std::uniform_real_distribution<> distAngs(-limAng, limAng);

		// Use pseudo-random number generation to create transformation
		using namespace engabra::g3;
		return rigibra::Transform
			{ Vector{ distLocs(gen), distLocs(gen), distLocs(gen) }
			, rigibra::Attitude
				{ rigibra::PhysAngle
					{ distAngs(gen), distAngs(gen), distAngs(gen) }
				}
			};
	}


} // [orinet]


#endif // OriNet_random_INCL_

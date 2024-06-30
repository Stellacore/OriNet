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

namespace rand
{
	// Range of translations - plus/minus this limit
	constexpr double sLimLoc{ 10. };

	// Range of rotation angles - plus/minus this limit
	constexpr double sLimAng{ engabra::g3::pi };

	//! \brief A transformation with uniformly distributed parameters values
	inline
	rigibra::Transform
	perturbedTransform
		( engabra::g3::Vector const & meanLoc
		, rigibra::PhysAngle const & meanAng
		, double const & sigmaLoc = (1./100.) * sLimLoc
		, double const & sigmaAng = (1./100.) * sLimAng
		)
	{	
		// Configure pseudo-random number distribution generator
		static std::mt19937 gen(31035893u);
		static std::normal_distribution<> distLocs(0., sigmaLoc);
		static std::normal_distribution<> distAngs(0., sigmaAng);

		// Use pseudo-random number generation to create transformation
		return rigibra::Transform
			{ engabra::g3::Vector
				{ meanLoc[0] + distLocs(gen)
				, meanLoc[1] + distLocs(gen)
				, meanLoc[2] + distLocs(gen)
				}
			, rigibra::Attitude
				{ rigibra::PhysAngle
					{ meanAng.theBiv[0] + distAngs(gen)
					, meanAng.theBiv[1] + distAngs(gen)
					, meanAng.theBiv[2] + distAngs(gen)
					}
				}
			};
	}


	//! \brief A transformation with uniformly distributed parameters values
	inline
	rigibra::Transform
	uniformTransform
		()
	{
		// Configure pseudo-random number distribution generator
		static std::mt19937 gen(74844020u);
		static std::uniform_real_distribution<> distLocs(-sLimLoc, sLimLoc);
		static std::uniform_real_distribution<> distAngs(-sLimAng, sLimAng);

		// Use pseudo-random number generation to create transformation
		return rigibra::Transform
			{ engabra::g3::Vector
				{ distLocs(gen), distLocs(gen), distLocs(gen) }
			, rigibra::Attitude
				{ rigibra::PhysAngle
					{ distAngs(gen), distAngs(gen), distAngs(gen) }
				}
			};
	}

} // [rand]

} // [orinet]


#endif // OriNet_random_INCL_

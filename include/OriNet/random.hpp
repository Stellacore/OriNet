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
#include <vector>


namespace orinet
{

namespace rand
{
	// Range of translations - plus/minus this limit
//	constexpr double sLimLoc{ 10. };

	// Range of rotation angles - plus/minus this limit
//	constexpr double sLimAng{ engabra::g3::pi };

	/*! \brief Estimate distribution of triad transform residual magnitudes.
	 *
	 * For terminology, define a "hexad" as a collection of the six
	 * vectors associated with +/- versions of coordinate basis vectors.
	 * E.g. for standard Cartesian basis, the hexad are the six points 
	 * defined by +/-e1, +/-e2, +/-e3.
	 *
	 * For two different transformations, each transforms generally
	 * produces a different resulting hexad. This function attempts
	 * to estimate the RMSE magnitude of the respective difference
	 * vectors between each transformed hexad.
	 *
	 * Assume that the two transforms are generated with standard
	 * deviation of sigmaLoc in their offset vector components and with
	 * standard deviation sigmaAng in their physical angle component.
	 *
	 * This function provides a heuristic estimate what to expect for
	 * the stdandard deviation of vector magnitude between the two
	 * transformed hexads.
	 */
	inline
	double
	sigmaMagForSigmaLocAng
		( double const & sigmaLoc
			//!< Standard deviation of each transform offset vector component
		, double const & sigmaAng
			//!< Standard deviation of each transform angle bivector component
		)
	{
		// Unclear what's best, but following seems not unreasonable.
		// Error in location should translate directly to maxMag result
		// error.  Error in angle components should change maxMag by
		// angle error time basis vector length (unity). Therefore,
		// the two errors seem like they should add 'rmse' style.
		using engabra::g3::sq;
		double const estSigma
			{ std::sqrt(3.*sq(sigmaLoc) + 3.*sq(sigmaAng)) };
		return estSigma;
	}

	//! \brief A transformation with uniformly distributed parameters values
	inline
	rigibra::Transform
	perturbedTransform
		( engabra::g3::Vector const & meanLoc
		, rigibra::PhysAngle const & meanAng
		, double const & sigmaLoc
		, double const & sigmaAng
		)
	{	
		rigibra::Transform xform{ rigibra::null<rigibra::Transform>() };
		if (! ((sigmaLoc < 0.) || (sigmaAng < 0.)) )
		{
			// Configure pseudo-random number distribution generator
			static std::mt19937 gen(31035893u);
			std::normal_distribution<> distLocs(0., sigmaLoc);
			std::normal_distribution<> distAngs(0., sigmaAng);

			// Use pseudo-random number generation to create transformation
			xform = rigibra::Transform
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
		return xform;
	}


	//! \brief A transformation with uniformly distributed parameters values
	inline
	rigibra::Transform
	uniformTransform
		( std::pair<double, double> const & locMinMax
		, std::pair<double, double> const & angMinMax
		)
	{
		double const & locMin = locMinMax.first;
		double const & locMax = locMinMax.second;
		double const & angMin = angMinMax.first;
		double const & angMax = angMinMax.second;

		// Configure pseudo-random number distribution generator
		static std::mt19937 gen(74844020u);
		std::uniform_real_distribution<> distLocs(locMin, locMax);
		std::uniform_real_distribution<> distAngs(angMin, angMax);

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
		, std::size_t const & numMea
		, std::size_t const & numErr
		, double const & sigmaLoc
		, double const & sigmaAng
		, std::pair<double, double> const & locMinMax
			= { -10., 10. }
		, std::pair<double, double> const & angMinMax
			= { -engabra::g3::pi, engabra::g3::pi }
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
				{ perturbedTransform (expLoc, expAng, sigmaLoc, sigmaAng) };
			xforms.emplace_back(meaXform);
		}

		// ... a few 'blunderous' measurements - from uniform probability
		for (std::size_t nn{0u} ; nn < numErr ; ++nn)
		{
			rigibra::Transform const errXform
				{ uniformTransform(locMinMax, angMinMax) };
			xforms.emplace_back(errXform);
		}

		return xforms;
	}

} // [rand]

} // [orinet]


#endif // OriNet_random_INCL_

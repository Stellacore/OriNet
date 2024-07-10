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
\snippet test_robust.cpp DoxyExample02

*/


#include <Engabra>
#include <Rigibra>

#include <random>
#include <vector>


namespace orinet
{

/*! \brief Functions for generating random data and transformations
 */
namespace random
{
	// Range of translations - plus/minus this limit
//	constexpr double sLimLoc{ 10. };

	// Range of rotation angles - plus/minus this limit
//	constexpr double sLimAng{ engabra::g3::pi };

	//! \brief A random unitary direction vector
	inline
	engabra::g3::Vector
	directionVector
		()
	{
		using namespace engabra::g3;
		Vector dir{ null<Vector>() };
		for (;;)
		{
			static std::mt19937 gen(36742620u);
			std::uniform_real_distribution<> dist(-1., 1.);
			Vector const aVec
				{ dist(gen)
				, dist(gen)
				, dist(gen)
				};
			double const mag{ magnitude(aVec) };
			if (! (1. < mag))
			{
				dir = (1./mag) * aVec;
				break;
			}
		}
		return dir;
	}


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

	//! \brief Location from normally distributed components
	inline
	engabra::g3::Vector
	perturbedLocation
		( engabra::g3::Vector const & meanLoc
		, double const & sigmaLoc
		)
	{
		engabra::g3::Vector loc{ engabra::g3::null<engabra::g3::Vector>() };
		if (! (sigmaLoc < 0.))
		{
			// Configure pseudo-random number distribution generator
			static std::mt19937 gen(82035133u);
			std::normal_distribution<> distLocs(0., sigmaLoc);

			// Use pseudo-random number generation to create transformation
			loc = engabra::g3::Vector
				{ meanLoc[0] + distLocs(gen)
				, meanLoc[1] + distLocs(gen)
				, meanLoc[2] + distLocs(gen)
				};
		}
		return loc;
	}

	//! \brief Attitude from normally distributed Physical angle components
	inline
	rigibra::Attitude
	perturbedAttitude
		( rigibra::PhysAngle const & meanAng
		, double const & sigmaAng
		)
	{
		rigibra::Attitude att{ rigibra::null<rigibra::Attitude>() };
		if (! (sigmaAng < 0.))
		{
			// Configure pseudo-random number distribution generator
			static std::mt19937 gen(18448574u);
			std::normal_distribution<> distAngs(0., sigmaAng);

			// Use pseudo-random number generation to create transformation
			att = rigibra::Attitude
				{ rigibra::PhysAngle
					{ meanAng.theBiv[0] + distAngs(gen)
					, meanAng.theBiv[1] + distAngs(gen)
					, meanAng.theBiv[2] + distAngs(gen)
					}
				};
		}
		return att;
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
			// Use pseudo-random number generation to create transformation
			xform = rigibra::Transform
				{ perturbedLocation(meanLoc, sigmaLoc)
				, perturbedAttitude(meanAng, sigmaAng)
				};
		}
		return xform;
	}

	//! \brief A transformation with uniformly distributed parameters values
	inline
	rigibra::Transform
	perturbedTransform
		( rigibra::Transform const & expXform
		, double const & sigmaLoc
		, double const & sigmaAng
		)
	{	
		engabra::g3::Vector const expLoc = expXform.theLoc;
		rigibra::PhysAngle const expAng{ expXform.theAtt.physAngle() };

		return perturbedTransform(expLoc, expAng, sigmaLoc, sigmaAng);
	}

	//! \brief A transformation with uniformly distributed parameters values
	inline
	engabra::g3::Vector
	uniformLocation
		( std::pair<double, double> const & locMinMax
		)
	{
		using namespace engabra::g3;
		Vector loc{ null<Vector>() };

		double const & locMin = locMinMax.first;
		double const & locMax = locMinMax.second;

		// Configure pseudo-random number distribution generator
		static std::mt19937 gen(99981274u);
		std::uniform_real_distribution<> distLocs(locMin, locMax);

		// Use pseudo-random number generation to create transformation
		loc = Vector{ distLocs(gen), distLocs(gen), distLocs(gen) };

		return loc;
	}

	//! \brief An attitude with uniformly distributed parameters values
	inline
	rigibra::Attitude
	uniformAttitude
		( std::pair<double, double> const & angMinMax
		)
	{
		using namespace rigibra;
		Attitude att{ null<Attitude>() };
		double const & angMin = angMinMax.first;
		double const & angMax = angMinMax.second;

		// Configure pseudo-random number distribution generator
		static std::mt19937 gen(48169386u);
		std::uniform_real_distribution<> distAngs(angMin, angMax);

		// keep angle size within principle range
		using namespace engabra::g3;
		BiVector angle{ distAngs(gen), distAngs(gen), distAngs(gen) };
		double mag{ magnitude(angle) };
		if (turnHalf < mag)
		{
			BiVector const dir{ direction(angle) };
			mag = fmod(mag, turnHalf);
			angle = mag * dir;
		}

		// Use pseudo-random number generation to create transformation
		att = Attitude{ PhysAngle{ angle } };

		return att;
	}

	//! \brief A transformation with uniformly distributed parameters values
	inline
	rigibra::Transform
	uniformTransform
		( std::pair<double, double> const & locMinMax
		, std::pair<double, double> const & angMinMax
			= { -engabra::g3::pi, engabra::g3::pi }
		)
	{
		return rigibra::Transform
			{ uniformLocation(locMinMax)
			, uniformAttitude(angMinMax)
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
	inline
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

} // [random]

} // [orinet]


#endif // OriNet_random_INCL_

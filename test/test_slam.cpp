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
\brief Test (and example) code for exercising OriNet with a 'SLAM' scenario.

SLAM (Simultaneous Location and Mapping) is a computer vision concept
in which moving cameras establish their own location relative to various
detected landmark features, the location of which must also be determined.

This example code explores the type of SLAM in which detected object
space features are classified and associated with individual rigid
bodies. This example code demonstrates simulation of a sequence of
sensing events as if by a moving video camera. Each event produces a
rigid body orientation with one or more object space bodies. The result
of the program is to compute a 3D model of the orientation of all the
object space bodies.

A main test feature is that the object model is updated after each new
observation is added in order to simulate a real-time use case in which
OriNet is providing continuously updated object space model information.

*/


#include "OriNet/OriNet"
#include "OriNet/compare.hpp"
#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <iostream>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <vector>


namespace sim
{
	using CamKey = std::size_t;
	using FeaKey = std::size_t;

	// use offset values to make visual distinction in diganostic output
	constexpr FeaKey sFeaKey0{  900 };
	constexpr CamKey sCamKey0{ 1000 };

	//! Uniformly distributed index into (non-empty) container.
	inline
	std::size_t
	randomIndexInto
		( std::size_t const & size
		)
	{
		if (! (0u < size))
		{
			std::cerr << "FATAL:"
				" calling randomIndexInto with empty container" << std::endl;
			exit(1);
		}
		static std::mt19937 gen(35364653u);
		std::uniform_int_distribution<> dist(0u, size-1u);
		return dist(gen);
	}

	//! Simulate distribution of random object space features
	std::map<FeaKey, rigibra::Transform>
	expFeaXforms
		( std::size_t const & numFea = 20u
		, double const & pmDist = 10.
		)
	{
		using rigibra::Transform;
		std::map<FeaKey, Transform> expFeaXforms;

		// simulate object feature body distribution
		constexpr std::size_t numMea{ 0u }; // no impact here
		constexpr double locSigma{ 0. }; // no impact here
		constexpr double angSigma{ 0. }; // no impact here
		std::pair<double, double> locMinMax{ -pmDist, pmDist };
		constexpr std::pair<double, double> angMinMax{ -1., 1. };

		// generate a collection of feature orientations
		std::vector<Transform> const feaXforms
			{ orinet::random::noisyTransforms
				( rigibra::identity<Transform>()
				, numMea, numFea
				, locSigma, angSigma // unused for 0 measurements
				, locMinMax, angMinMax
				)
			};

		// assign key values to each (arbitrarily)
		FeaKey feaKey{ sim::sFeaKey0 };
		for (Transform const & feaXform : feaXforms)
		{
			expFeaXforms[feaKey] = feaXform;
			++feaKey;
		}

		return expFeaXforms;
	}


	/*! \brief Produce orientations as a function of time.
	 *
	 * Provides orientations which are (pseudo)random perturbations of
	 * a simple deterministic path.
	 */
	struct Trajectory
	{
		double const theSpeed{ 1./4. }; // [m/s]

		inline
		Trajectory
			( double const & speed
			)
			: theSpeed(speed)
		{ }

		inline
		virtual
		~Trajectory
			() = default;

		//! Orientation at time tau for deterministic trajectory model.
		inline
		virtual
		rigibra::Transform
		pathOrientation
			( double const & tau // [s]
			) const = 0;

		//! Orientation at time tau for deterministic trajectory model.
		inline
		rigibra::Transform
		perturbedOrientation
			( double const & tau // [s]
//			, double const & locSigma = 1./100.
//			, double const & angSigma = 5./1000.
//			, double const & probErr = .2 // blunder probability
, double const & locSigma = 0./100.
, double const & angSigma = 0./1000.
, double const & probErr = .0 // blunder probability
			, std::pair<double, double> const & locMinMax = { -.5, .5 }
			, std::pair<double, double> const & angMinMax = { -.5, .5 }
			) const
		{
			rigibra::Transform xPathWrtRef{ pathOrientation(tau) };

			// determine if return value should be measurement or blunder
			static std::mt19937 gen(47686779u);
			std::uniform_real_distribution<> dist(0., 1.);
			bool const isBlunder{ (dist(gen) < probErr) };

			// simulate appropriate type of transform
			rigibra::Transform xBodyWrtPath;
			using namespace orinet::random;
			if (isBlunder)
			{
				xBodyWrtPath = uniformTransform(locMinMax, angMinMax);
			}
			else
			{
				xBodyWrtPath = perturbedTransform
					(xPathWrtRef, locSigma, angSigma);
			}

			return (xBodyWrtPath * xPathWrtRef);
		}

	}; // Trajectory

	/*! \brief Trajectory with an underlying linear model
	 */
	struct TrajectoryLine : public Trajectory
	{
		engabra::g3::Vector const theDir0{ engabra::g3::e1 };
		engabra::g3::Vector const theStart
			{ engabra::g3::zero<engabra::g3::Vector>() };
		rigibra::Attitude const theAtt0
			{ rigibra::identity<rigibra::Attitude>() };

		inline
		TrajectoryLine
			( double const & speed = 1./4.
			)
			: Trajectory(speed)
		{ }

		//! Orientation at time tau for deterministic trajectory model.
		inline
		virtual
		rigibra::Transform
		pathOrientation
			( double const & tau // [s]
			) const
		{
			using namespace engabra::g3;
			Vector const loc{ theStart + theSpeed*tau*theDir0 };
			using namespace rigibra;
			return Transform{ loc, theAtt0 };
		}

	}; // TrajectoryLine

	/*! \brief Trajectory with an underlying circular model
	 */
	struct TrajectoryCircle : public Trajectory
	{
		double const theRadius{ engabra::g3::null<double> () };
		engabra::g3::Vector const theCenter
			{ engabra::g3::zero<engabra::g3::Vector>() };
		engabra::g3::Vector const thePlaneDir1{ engabra::g3::e1 };
		engabra::g3::Vector const thePlaneDir2{ engabra::g3::e2 };
		rigibra::Attitude const theAtt0
			{ rigibra::identity<rigibra::Attitude>() };

		engabra::g3::BiVector const thePlaneDir{ engabra::g3::e12 };

		inline
		TrajectoryCircle
			( double const & radius
				= 1.
			, engabra::g3::Vector const & center
				= engabra::g3::zero<engabra::g3::Vector>()
			, engabra::g3::Vector const & planeDir1
				= engabra::g3::e1
			, engabra::g3::Vector const & planeDir2
				= engabra::g3::e2
			, double const & speed
				= 1./4.
			)
			: Trajectory(speed)
			, theRadius{ radius }
			, theCenter{ center }
			, thePlaneDir1{ planeDir1 }
			, thePlaneDir2{ planeDir2 }
			, thePlaneDir
				{ engabra::g3::direction((thePlaneDir1*thePlaneDir2).theBiv) }
		{ }

		//! Orientation at time tau for deterministic trajectory model.
		inline
		virtual
		rigibra::Transform
		pathOrientation
			( double const & tau // [s]
			) const
		{
			using namespace engabra::g3;
			using namespace rigibra;

			// rotate starting vector through angle depending on time
			double const angSpeed{ theSpeed / theRadius };
			PhysAngle const ang{ tau * angSpeed * thePlaneDir };
			Attitude const att(ang);
			Vector const loc{ att(thePlaneDir1) };

			return Transform{loc, theAtt0};
		}

	}; // TrajectoryCircle


	//! Simulate camera observing several features.
	inline
	std::map<std::pair<CamKey, FeaKey>, rigibra::Transform>
	xformCamWrtFeas
		( TrajectoryCircle const & trajCam
		, double const & tau
		, std::map<FeaKey, rigibra::Transform> const & expFeaXforms
		, std::size_t const & numFeas = 3u
		)
	{
		using rigibra::Transform;
		std::map<std::pair<CamKey, FeaKey>, Transform> mapCamFeaXforms;

		// get camera position
		Transform const xCamWrtRef{ trajCam.perturbedOrientation(tau) };

		// get relative transformations to several targets
		while (mapCamFeaXforms.size() < numFeas)
		{
			// select a pseudo-random feature to be "observed" next
			std::size_t const randNdx
				{ randomIndexInto(expFeaXforms.size()) };
			std::map<FeaKey, Transform>::const_iterator
				itFeaXform{ expFeaXforms.cbegin() };
			std::advance(itFeaXform, randNdx);
			FeaKey const & feaKey = itFeaXform->first;
			Transform const & xFeaWrtRef = itFeaXform->second;

			Transform const xRefWrtFea{ inverse(xFeaWrtRef) };
			Transform const xCamWrtFea{ xCamWrtRef * xRefWrtFea };
			//
			sim::CamKey const & camKey = sim::sCamKey0;
			mapCamFeaXforms.emplace_hint
				( mapCamFeaXforms.end()
				, std::make_pair
					( std::make_pair(camKey, feaKey)
					, xCamWrtFea
					)
				);
		}
		return mapCamFeaXforms;
	}

} // [sim]

	//! get the maximum magnitude (hexad) error between the two collections
	inline
	double
	maxMagErrBetween
		( std::map<sim::FeaKey, rigibra::Transform> const & gotFeaXforms
		, std::map<sim::FeaKey, rigibra::Transform> const & expFeaXforms
		)
	{
		double maxMagErr{ -1. };

		using rigibra::Transform;
		for (std::map<sim::FeaKey, Transform>::value_type
			const & gotFeaXform : gotFeaXforms)
		{
			sim::FeaKey const & feaKey = gotFeaXform.first;
			Transform const & gotXform = gotFeaXform.second;

			std::map<sim::FeaKey, Transform>::const_iterator const itExp
				{ expFeaXforms.find(feaKey) };
			if (expFeaXforms.cend() == itExp)
			{
				std::cerr << "FATAL Error" << std::endl;
				exit(1);
			}
			Transform const & expXform = itExp->second;

			constexpr bool useNorm{ false };
			using orinet::compare::maxMagResultDifference;
			double const maxErr
				{ maxMagResultDifference(gotXform, expXform, useNorm) };

			maxMagErr = std::max(maxMagErr, maxErr);
		}
		return maxMagErr;
	}

	//! Description of feaXforms content
	inline
	std::string
	infoString
		( std::map<sim::FeaKey, rigibra::Transform> const & feaXforms
		, std::string const & title = {}
		)
	{
		std::ostringstream oss;
		oss << '\n';
		for (std::map<sim::FeaKey, rigibra::Transform>::value_type
			const & feaXform : feaXforms)
		{
			oss << title
				<< ' ' << feaXform.first
				<< ' ' << feaXform.second
				<< '\n';
		}
		return oss.str();
	}

	//! Update robust transform network with one exposure's worth of features.
	inline
	void
	updateNetwork
		( orinet::network::Geometry * const & ptNetGeo
		, std::map<std::pair<sim::CamKey, sim::FeaKey>, rigibra::Transform>
			const & mapCamFeaXforms
		)
	{
		using namespace rigibra;

		// generate network edges
		using Iter = typename
			std::map<std::pair<sim::CamKey, sim::FeaKey>, Transform>
			::const_iterator;
		for (Iter it1{mapCamFeaXforms.cbegin()}
			; mapCamFeaXforms.cend() != it1 ; ++it1)
		{
			Transform const & xCamWrtFea1 = it1->second;
			sim::FeaKey const & feaKey1 = it1->first.second;
			Iter it2{ it1 };
			++it2;
			for ( ; mapCamFeaXforms.cend() != it2 ; ++it2)
			{
				Transform const & xCamWrtFea2 = it2->second;
				sim::FeaKey const & feaKey2 = it2->first.second;
				Transform const xFea2wrtCam{ inverse(xCamWrtFea2) };
				Transform const x2w1 { xFea2wrtCam * xCamWrtFea1 };

	/*
	std::cout
	<< "feaKey1,2: " << feaKey1 << ' ' << feaKey2
	<< " x2w1: " << x2w1 << '\n';
	*/

				using namespace orinet::network;

				EdgeDir const edgeDir{ feaKey1, feaKey2 };

				std::shared_ptr<EdgeBase> ptGraphEdge
					{ ptNetGeo->edge(edgeDir) };
				if (ptGraphEdge)
				{
					// accumulate into existing edge
					EdgeRobust * const ptEdgeRobust
						{ reinterpret_cast<EdgeRobust *>
							(ptGraphEdge.get())
						};
					ptEdgeRobust->accumulateXform(x2w1);
	//std::cout << " accumulating onto edge: " << feaKey1 << ' ' << feaKey2 << '\n';
	////std::cout << " x2w1: " << x2w1 << '\n';
	//std::cout << ptEdgeRobust->infoString("robustEdge") << '\n';
				}
				else
				{
					constexpr std::size_t reserveSize{ 4096u };
					std::shared_ptr<EdgeBase> const ptEdge
						{ std::make_shared<EdgeRobust>
							(edgeDir, x2w1, reserveSize)
						};
					// insert new edge into geometry network
	//std::cout << "adding new edge between: " << feaKey1 << ' ' << feaKey2 << '\n';
					ptNetGeo->insertEdge(ptEdge);
				}
			}
		}
	}

namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		using namespace rigibra;

		// count error values that exceed tolerance (used for test condition)
		constexpr double tolErr{ 1.e-6 };
		std::set<double> maxErrValues;

		// simulate a number of object space features
		std::size_t numFea{ 20u };
		std::map<sim::FeaKey, Transform> const expFeaXforms
			{ sim::expFeaXforms(numFea) };

		// simulate on going camera trajectory
		sim::TrajectoryCircle const trajCam{};

		// update network geometry continously for a period of time
		orinet::network::Geometry netGeo;
		double tau{ 0. };
		constexpr double dtau{ 1./32. };
		for (;;)
		{
			tau = tau + dtau;
			if (60.125 < tau)
			{
				break;
			}

			// simulate a single exposure and feature extraction operations
			std::map<std::pair<sim::CamKey, sim::FeaKey>, Transform>
				const mapCamFeaXforms
				{ sim::xformCamWrtFeas(trajCam, tau, expFeaXforms) };

			// update robust network
			updateNetwork(&netGeo, mapCamFeaXforms);

			// Lock-in first iteration's first camera station as reference
			static sim::FeaKey const feaKey0
				{ expFeaXforms.cbegin()->first };
			static Transform const xform0
				{ expFeaXforms.cbegin()->second };

			// propagate features through current robust network
			std::map<sim::FeaKey, Transform> const gotFeaXforms
				{ netGeo.propagateTransforms(feaKey0, xform0) };

			// asset the quality of the result
			double const maxErr
				{ maxMagErrBetween(gotFeaXforms, expFeaXforms) };

			// check max error against tolerance
			if (! (maxErr < tolErr))
			{
				maxErrValues.insert(maxErr);
			}

using engabra::g3::io::fixed;
std::cout
	<< "tau,maxErr: " << fixed(tau)
	<< ' ' << fixed(maxErr)
	<< '\n';

		} // over time

//std::cout << "netGeo:\n" << netGeo.infoStringContents() << '\n';

		// [DoxyExample01]

		// [DoxyExample01]

		if (! maxErrValues.empty())
		{
			oss << "Failure of maxErr value test\n";
			oss << "exp: " << 0. << '\n';
			oss << "got: " << maxErrValues.size() << '\n';
		}

std::cout << "\nmaxErrValues.size: " << maxErrValues.size() << '\n';

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


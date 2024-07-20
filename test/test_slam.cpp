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
#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <iostream>
#include <map>
#include <random>
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
	template <typename Container>
	inline
	std::size_t
	randomIndexInto
		( Container const & container
		)
	{
		if (container.empty())
		{
			std::cerr << "FATAL:"
				" calling randomIndexInto with empty container" << std::endl;
			exit(1);
		}
		static std::mt19937 gen(35364653u);
		std::uniform_int_distribution<> dist(0u, container.size()-1u);
		return dist(gen);
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
		, std::vector<rigibra::Transform> const & xFeatures
		, std::size_t const & numFeas = 3u
		)
	{
		using rigibra::Transform;
		std::map<std::pair<CamKey, FeaKey>, Transform> mapCamFeaXforms;

		// get camera position
		Transform const xCamWrtRef{ trajCam.perturbedOrientation(tau) };

		// get relative transformations to several targets
//		for (std::size_t nFea{0u} ; nFea < numFeas ; ++nFea)
		while (mapCamFeaXforms.size() < numFeas)
		{
			std::size_t const randNdx{ randomIndexInto(xFeatures) };
			Transform const & xFeaWrtRef = xFeatures[randNdx];
			Transform const xRefWrtFea{ inverse(xFeaWrtRef) };
			Transform const xCamWrtFea{ xCamWrtRef * xRefWrtFea };
			//
			sim::CamKey const & camKey = sim::sCamKey0;
			sim::FeaKey const feaKey{ sim::sFeaKey0 + randNdx };
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


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		using namespace rigibra;

		// simulate object feature body distribution
		constexpr std::size_t numMea{ 0u }; // no impact here
		constexpr double locSigma{ 0. }; // no impact here
		constexpr double angSigma{ 0. }; // no impact here
		constexpr std::pair<double, double> locMinMax{ -10., 10. };
		constexpr std::pair<double, double> angMinMax{ -1., 1. };
//		std::size_t numFea{ 20u };
std::size_t numFea{ 5u };
		std::vector<Transform> const xFeatures
			{ orinet::random::noisyTransforms
				( identity<Transform>()
				, numMea, numFea
				, locSigma, angSigma // unused for 0 measurements
				, locMinMax, angMinMax
				)
			};

auto const locSorter
	{ [] (Transform const & xfm1, Transform const & xfm2)
		{
		//	using namespace engabra::g3;
		//	return (magnitude(xfm1.theLoc) < magnitude(xfm2.theLoc));
			return (xfm1.theLoc[0] < xfm2.theLoc[0]);
		}
	};
std::vector<Transform> xSorts{ xFeatures };
std::sort(xSorts.begin(), xSorts.end(), locSorter);

std::cout << "xSorts...\n";
for (Transform const & xSort : xSorts)
{
	std::cout << xSort << '\n';
}
std::cout << '\n';
/*
*/


		// simulate on going camera trajectory
		sim::TrajectoryCircle const trajCam{};

		// update network geometry continously for a period of time
		orinet::network::Geometry netGeo;
		double tau{ 0. };
		constexpr double dtau{ 1./32. };
		for (;;)
		{
std::cout << '\n';
			tau = tau + dtau;
			if (0.125 < tau)
			{
				break;
			}

			//
			// simulate a single exposure along with feature extraction
			//

			std::map<std::pair<sim::CamKey, sim::FeaKey>, Transform>
				const mapCamFeaXforms
				{ sim::xformCamWrtFeas(trajCam, tau, xFeatures) };

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

std::cout
	<< "feaKey1,2: " << feaKey1 << ' ' << feaKey2
	<< " x2w1: " << x2w1 << '\n';
/*
*/

					using namespace orinet::network;

					EdgeDir const edgeDir{ feaKey1, feaKey2 };

					std::shared_ptr<EdgeBase> ptGraphEdge
						{ netGeo.edge(edgeDir) };
					if (ptGraphEdge)
					{
						// accumulate into existing edge
						EdgeRobust * const ptEdgeRobust
							{ reinterpret_cast<EdgeRobust *>
								(ptGraphEdge.get())
							};
						ptEdgeRobust->accumulateXform(x2w1);
std::cout << " accumulating onto edge: " << feaKey1 << ' ' << feaKey2 << '\n';
////std::cout << " x2w1: " << x2w1 << '\n';
//std::cout << ptEdgeRobust->infoString("robustEdge") << '\n';
					}
					else
					{
						constexpr std::size_t reserveSize{ 32u };
						std::shared_ptr<EdgeBase> const ptEdge
							{ std::make_shared<EdgeRobust>
								(edgeDir, x2w1, reserveSize)
							};
						// insert new edge into geometry network
std::cout << "adding new edge between: " << feaKey1 << ' ' << feaKey2 << '\n';
						netGeo.insertEdge(ptEdge);
					}
				}
			}

			sim::FeaKey const & feaKey0
				= mapCamFeaXforms.cbegin()->first.second;
			Transform const & xform0
				= mapCamFeaXforms.cbegin()->second;
			using orinet::network::StaKey;
			std::map<StaKey, Transform> const fitGeos
				{ netGeo.propagateTransforms(feaKey0, xform0) };

/*
std::cout << '\n';
for (std::map<StaKey, Transform>::value_type const & fitGeo : fitGeos)
{
	std::cout << "** fitGeo: " << fitGeo.first << ' ' << fitGeo.second << '\n';
}
*/

		} // over time

//std::cout << "netGeo:\n" << netGeo.infoStringContents() << '\n';

		// [DoxyExample01]

		// [DoxyExample01]

		// TODO replace this with real test code
		std::string gotName{ __FILE__ };
		std::string expName{"_.cpp"};
		bool const isTemplate{ std::string::npos != gotName.find(expName) };
		if (! isTemplate)
		{
			oss << "Failure: Implement template test case\n";
			oss << "exp: " << expName << '\n';
			oss << "got: " << gotName << '\n';
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


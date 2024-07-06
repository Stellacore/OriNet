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


#ifndef OriNet_sim_INCL_
#define OriNet_sim_INCL_

/*! \file
\brief Functions for simulating for testing/experimentation with OriNet.

Example:
\snippet SimNetwork.cpp DoxyExample02
*/


#include "OriNet/align.hpp"
#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <map>
#include <numeric>
#include <random>
#include <utility>
#include <vector>


namespace orinet
{

namespace sim
{
	//! Association of stations in From/Into order.
	using NdxPair = std::pair<std::size_t, std::size_t>;

	//! \brief Generate random pairs of directions
	inline
	align::DirPair
	directionPair
		( std::pair<double, double> const & minMaxAngleMag = { .1, 3. }
		)
	{
		using namespace engabra::g3;
		align::DirPair dirPair{ null<Vector>(), null<Vector>() };
		for (;;)
		{
			Vector const aDir{ random::directionVector() };
			Vector const bDir{ random::directionVector() };

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
	align::DirPair
	bodyDirectionPair
		( align::DirPair const & refDirPair
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

	//! Create a collection of (pseudo)random station orientations
	inline
	std::vector<rigibra::Transform>
	sequentialStations
		( std::size_t const & numStas
		)
	{
		std::vector<rigibra::Transform> stas;
		stas.reserve(numStas);
		using namespace engabra::g3;
		Vector loc{ 0. };
		for (std::size_t numSta{0u} ; numSta < numStas ; ++numSta)
		{
			using namespace rigibra;
			Transform const xform{ loc, identity<Attitude>() };
			stas.emplace_back(xform);
			loc = loc + 10.*e1;
		}
		return stas;
	}

	//! Create a collection of (pseudo)random station orientations
	inline
	std::vector<rigibra::Transform>
	randomStations
		( std::size_t const & numStas
		, std::pair<double, double> const & locMinMax
		)
	{
		std::vector<rigibra::Transform> stas;
		stas.reserve(numStas);
		for (std::size_t numSta{0u} ; numSta < numStas ; ++numSta)
		{
			stas.emplace_back
				(random::uniformTransform(locMinMax));
		}
		return stas;
	}

	//! simulate backsight observations
	inline
	std::map<NdxPair, std::vector<rigibra::Transform> >
	backsightTransforms
		( std::vector<rigibra::Transform> const & expStas
		, std::size_t const & numBacksight
		, std::size_t const & numMea
		, std::size_t const & numErr
		, std::pair<double, double> const & locMinMax
		, std::pair<double, double> const & angMinMax
			= { -engabra::g3::pi, engabra::g3::pi }
		, double const & sigmaLoc = 1./8.
		, double const & sigmaAng = 5./1024.
		)
	{
		std::map<NdxPair, std::vector<rigibra::Transform> > pairXforms;

		// simulate measurements station by station)
		std::vector<std::size_t> staNdxs(expStas.size());
		std::iota(staNdxs.begin(), staNdxs.end(), 0u);
		for (std::size_t currSta{0u} ; currSta < expStas.size() ; ++currSta)
		{
			rigibra::Transform const & expCurrWrtRef = expStas[currSta];

			// generate backsight transforms for this station
			static std::mt19937 gen(55342463u);
			std::shuffle(staNdxs.begin(), staNdxs.begin() + currSta, gen);

			std::size_t const nbMax{ std::min(currSta, numBacksight) };
			for (std::size_t backSta{0u} ; backSta < nbMax ; ++backSta)
			{
				std::size_t const & fromNdx = staNdxs[backSta];
				std::size_t const & intoNdx = currSta;

				// connect randomly with previous stations
				rigibra::Transform const & expBackWrtRef = expStas[fromNdx];

				// compute expected relative setup transformation
				rigibra::Transform const expRefWrtBack
					{ rigibra::inverse(expBackWrtRef) };
				rigibra::Transform const expCurrWrtBack
					{ expCurrWrtRef * expRefWrtBack };

				// simulate backsight transformations
				std::vector<rigibra::Transform> const obsXforms
					{ random::noisyTransforms
						( expCurrWrtBack
						, numMea
						, numErr
						, sigmaLoc
						, sigmaAng
						, locMinMax
						, angMinMax
						)
					};

				// record relative transforms for later processing
				pairXforms.emplace_hint
					( pairXforms.end()
					, std::make_pair(NdxPair{fromNdx, intoNdx}, obsXforms)
					);
			}
		}

		return pairXforms;
	}

} // [sim]

} // [orinet]


#endif // OriNet_sim_INCL_

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
\brief Demonstrate robust network formation.
*/


#include "OriNet/OriNet"

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <iostream>
#include <map>
#include <numeric>
#include <vector>


namespace
{
	//! Association of stations in From/Into order.
	using NdxPair = std::pair<std::size_t, std::size_t>;

} // [anon]

namespace sim
{
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
				(orinet::random::uniformTransform(locMinMax));
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


		// simulate measurements (station by station)
		std::vector<std::size_t> staNdxs(expStas.size());
		std::iota(staNdxs.begin(), staNdxs.end(), 0u);
		for (std::size_t currSta{0u} ; currSta < expStas.size() ; ++currSta)
		{
			rigibra::Transform const & expCurrWrtRef = expStas[currSta];

std::cout << "\nCurrent index: " << currSta << '\n';

			// generate backsight transforms for this station
			static std::mt19937 gen(55342463u);
			std::shuffle(staNdxs.begin(), staNdxs.begin() + currSta, gen);
std::cout << "  backsight to: ";

			std::size_t const nbMax{ std::min(currSta, numBacksight) };
			for (std::size_t backSta{0u} ; backSta < nbMax ; ++backSta)
			{
				// connect randomly with previous stations
std::cout << ' ' << staNdxs[backSta];
				rigibra::Transform const & expBackWrtRef = expStas[backSta];

				// compute expected relative setup transformation
				rigibra::Transform const expRefWrtBack
					{ rigibra::inverse(expBackWrtRef) };
				rigibra::Transform const expCurrWrtBack
					{ expCurrWrtRef * expRefWrtBack };

				// simulate backsight transformations
				std::vector<rigibra::Transform> const obsXforms
					{ orinet::random::noisyTransforms
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
				std::size_t const & fromNdx = backSta;
				std::size_t const & intoNdx = currSta;
				pairXforms.emplace_hint
					( pairXforms.end()
					, std::make_pair(NdxPair{fromNdx, intoNdx}, obsXforms)
					);
			}
std::cout << '\n';
		}

		return pairXforms;
	}

} // [sim]


/*! \brief 
 *
 * Ref Usage string in main code for details, i.e.:
 * \snippet demo_network.cpp DoxyExample01
 */
int
main
	( int argc
	, char * argv[]
	)
{
	// [DoxyExample01]
	char const * const useMsg =
	R"(
	This program demonstrates determination of rigid body
	network.  The network is associated with a directed
	graph comprising nodes and edges in which the nodes
	are considered to be rigid body frames and the edges
	are rigid body transformations between them.

	This program utilizes simulation to generate multiple
	rigid body transformations between nodes.  Some of the
	transformations are generated with small (and normally
	distributed) errors, while others are generated with
	arbitrarily large error to represent outliners/blunders.

	The approach here is modeled on a survey adjustment in
	which a number of stations are "setup".  Each setup
	includes determining the relationship with a few prior
	setups (akin to back-sight operations).

	The overall steps are:
	 * Generate a collection of random ideal station setups
	 * For each station added, simulate multiple back-sights
	   - select up to 2 (pseudo)random previous stations
	   - simulate multiple measurement and outlier setups
	     to each of the backsight stations.
	   - compute median error with robust orientation (for
	     use as graph edge weight)
	   - add new station to graph as a node
	   - add robust estimate of orientations as graph edge
	   - add inverse transformation to graph as reverse edge
	 * Find minimum spanning tree in graph
	 * Use minimum spanning tree to connect graph into single
	   network coordinate frame.
	)";
	// [DoxyExample01]

	if (! (2u == argc))
	{

		std::cerr
			<< '\n' << argv[0] << '\n'
			<< useMsg << '\n'
			<< "\nUsage: <progname> outFile" << '\n'
			;
		return 1;
	}
	std::cout << "\nHi from " << __FILE__ << '\n';

	// Configuration parameters
	constexpr std::size_t numStations{ 10u };
	constexpr std::size_t numBacksight{ 3u };
	constexpr std::size_t numMea{ 7u };
	constexpr std::size_t numErr{ 3u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };

	// generate collection of expected station orientations
	// (used for generating simulation data)
	std::vector<rigibra::Transform> const expStas
		{ sim::randomStations(numStations, locMinMax) };
std::cout << "number stations: " << expStas.size() << '\n';

	// simulate backsight observation data
	std::map<NdxPair, std::vector<rigibra::Transform> > const pairXforms
		{ sim::backsightTransforms
			(expStas, numBacksight, numMea, numErr, locMinMax)
		};
std::cout << "number backsights: " << pairXforms.size() << '\n';

	// process observations (enter into graph)
	for (std::pair<NdxPair, std::vector<rigibra::Transform> >
		const & pairXform : pairXforms)
	{
		std::size_t const & fromNdx = pairXform.first.first;
		std::size_t const & intoNdx = pairXform.first.second;
		std::vector<rigibra::Transform> const & xforms = pairXform.second;
std::cout << "NdxPair: " << fromNdx << ' ' << intoNdx << '\n';

		for (rigibra::Transform const & xform : xforms)
		{
std::cout << "  xform: " << xform << '\n';
		}
	}

}


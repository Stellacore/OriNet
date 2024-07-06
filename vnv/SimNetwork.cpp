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


#include <OriNet>
#include <OriNet/random.hpp>
#include <OriNet/sim.hpp>

#include <Engabra>
#include <Rigibra>

#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <vector>


namespace
{
	//! Association of stations in From/Into order.
	using NdxPair = std::pair<std::size_t, std::size_t>;

	// [DoxyExample01]
	char const * const useMsg =
	R"(
	This program demonstrates determination of rigid body
	network.  The network is associated with an undirected
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

} // [anon]



/*! \brief 
 *
 * Ref Usage string in main code for details, i.e.:
 * \snippet SimNetwork.cpp DoxyExample01
 */
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (2 < argc))
	{
		std::cerr
			<< '\n' << argv[0] << '\n'
			<< useMsg << '\n'
			<< "\nUsage: <progname>"
				" <network_all.dot> <network_mst.dot>"
			<< '\n'
			;
		return 1;
	}
	std::filesystem::path const dotPathAll{ argv[1] };
	std::filesystem::path const dotPathMst{ argv[2] };

	//
	// Configuration parameters
	//

	constexpr bool showResult{ true };
//#define EasyCase

#	if defined(EasyCase)
	constexpr std::size_t numStations{ 8u };
	constexpr std::size_t numBacksight{ 2u };
	constexpr std::size_t numMea{ 1u };
	constexpr std::size_t numErr{ 0u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };
#	else // EasyCase
	// general test data
	constexpr std::size_t numStations{ 10u };
	constexpr std::size_t numBacksight{ 3u };
	constexpr std::size_t numMea{ 7u };
	constexpr std::size_t numErr{ 3u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };
	/*
	// large size data
	constexpr std::size_t numStations{ 1000u };
	constexpr std::size_t numBacksight{ 5u };
	constexpr std::size_t numMea{ 400u };
	constexpr std::size_t numErr{  50u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };
	*/
#	endif // EasyCase

	// [DoxyExample02]
	//
	// Generate collection of expected station orientations
	// (used for generating simulation data)
	//
	std::vector<rigibra::Transform> const expStas
		{ orinet::sim::sequentialStations(numStations) };
	//	{ orinet::sim::randomStations(numStations, locMinMax) };

	// simulate backsight observation data
	std::map<NdxPair, std::vector<rigibra::Transform> > const pairXforms
		{ orinet::sim::backsightTransforms
			(expStas, numBacksight, numMea, numErr, locMinMax)
		};

	// [DoxyExample02]

	//
	// Populate graph: station frame nodes and robustly fit transform edges
	//

	orinet::network::Geometry geoNet;

	for (std::map<NdxPair, std::vector<rigibra::Transform> >::value_type
		const & pairXform : pairXforms)
	{
		// compute robustly fit transformation for this edge
		orinet::network::EdgeOri const edgeOri
			{ orinet::network::edgeOriMedianFit(pairXform.second) };

		// insert robust transform into network
		geoNet.addEdge(pairXform.first, edgeOri);
	}


	// save network topology to graphviz '.dot' file format
	geoNet.saveNetworkGraphic(dotPathAll);

	//
	// Find minimum spanning tree
	//

	std::vector<graaf::edge_id_t> const mstEdgeIds
		{ geoNet.spanningEdgeOris() };

	orinet::network::Geometry const mstNet{ geoNet.networkTree(mstEdgeIds) };

	mstNet.saveNetworkGraphic(dotPathMst);

	//
	// Update station orientations by traversing MST
	//

	// traverse mst from node 0 (since 0 always present in non-empty graph)
	orinet::network::StaNdx const staNdx0{ 0u };
	rigibra::Transform const & staXform0 = expStas[staNdx0];
	std::vector<rigibra::Transform> const gotStas
		{ mstNet.propagateTransforms(staNdx0, staXform0) };

	//
	// Display computed/propagated station locations
	//

	if (showResult)
	{
		std::size_t const numSta{ expStas.size() };
		std::cout << "\n==============";
		for (std::size_t nn{0u} ; nn < numSta ; ++nn)
		{
			rigibra::Transform const & expSta = expStas[nn];
			rigibra::Transform const & gotSta = gotStas[nn];
			std::cout
				<< '\n'
				<< "exp[" << nn << "] " << expSta << '\n'
				<< "got[" << nn << "] " << gotSta << '\n'
				;
		}
		std::cout << "==============\n";
	}

}


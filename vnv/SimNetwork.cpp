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
#include <Engabra>
#include <Rigibra>

#include <graaflib/graph.h>
#include <graaflib/io/dot.h>
#include <graaflib/algorithm/minimum_spanning_tree/kruskal.h>
#include <graaflib/algorithm/graph_traversal/breadth_first_search.h>

#include <algorithm>
#include <array>
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

//std::cout << "\nCurrent index: " << currSta << '\n';

			// generate backsight transforms for this station
			static std::mt19937 gen(55342463u);
			std::shuffle(staNdxs.begin(), staNdxs.begin() + currSta, gen);
//std::cout << "  backsight to: ";

			std::size_t const nbMax{ std::min(currSta, numBacksight) };
			for (std::size_t backSta{0u} ; backSta < nbMax ; ++backSta)
			{
				// connect randomly with previous stations
//std::cout << ' ' << staNdxs[backSta];
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
				std::size_t const & fromNdx = staNdxs[backSta];
				std::size_t const & intoNdx = currSta;
				pairXforms.emplace_hint
					( pairXforms.end()
					, std::make_pair(NdxPair{fromNdx, intoNdx}, obsXforms)
					);
			}
//std::cout << '\n';
		}

		return pairXforms;
	}

} // [sim]


namespace net
{

	//! Use station setups as network graph nodes
	struct Station
	{
		std::size_t theStaNdx;
		rigibra::Transform theExpXform;
		rigibra::Transform theGotXform;

	}; // Station

	//! Use rigid body transformation as network graph edges
	struct Edge : public graaf::weighted_edge<double>
	{
		//! station-to-station tranformation for network graph edges
		rigibra::Transform theXform;
		double theMaxMagErr;

		inline
		explicit
		Edge
			( rigibra::Transform const & xform
			, double const & maxMagErr
			)
			: theXform{ xform }
			, theMaxMagErr{ maxMagErr }
		{ }

		inline
		~Edge
			() = default;

		inline
		bool
		operator<
			( Edge const & other
			) const noexcept
		{
			return (this->get_weight() < other.get_weight());
		}

		inline
		bool
		operator!=
			( Edge const & other
			) const noexcept
		{
			return ((other < (*this)) || ((*this) < other));
		}

		[[nodiscard]]
		inline
		double
		get_weight
			() const noexcept override
		{
			return theMaxMagErr;
		}

	}; // Edge

	/*! Generate graph structure with robust edges
	 *
	 * The edges contain transformations. These rigid body transformations
	 * are interpreted to be (largerNodeNumber) w.r.t. (smallerNodeNumber)
	 */
	graaf::undirected_graph<Station, Edge>
	graphFrom
		( std::vector<rigibra::Transform> const expStas
		, std::map<NdxPair, std::vector<rigibra::Transform> >
			const & pairXforms
		)
	{
		graaf::undirected_graph<Station, Edge> network;

		// populate graph with a node for every edge
		using StaNdx = std::size_t;
		using VertId = graaf::vertex_id_t;
		std::map<StaNdx, VertId> mapVertFromNdx;

		// loop overall stations
		for (std::size_t staNdx{0u} ; staNdx < expStas.size() ; ++staNdx)
		{
			rigibra::Transform const & expSta = expStas[staNdx];
			static rigibra::Transform const empty
				{ rigibra::null<rigibra::Transform>() };
			Station const station{ staNdx, expSta, empty };
			// add vertex into map
			mapVertFromNdx[staNdx] = network.add_vertex(station);
		}

		// loop over all backsight station pairs
		// - compute a robust estimate of relationship using all observations
		// - enter robust estimate into graph as edges
		for (std::map<NdxPair, std::vector<rigibra::Transform> >::value_type
			const & pairXform : pairXforms)
		{

			std::size_t const & fromNdx = pairXform.first.first;
			std::size_t const & intoNdx = pairXform.first.second;
			if (! (fromNdx < intoNdx))
			{
				std::cerr << "Fatal Error:"
					" Network convention assumes IntoWrtFrom for which"
					" \n(fromNdx < intoNdx) indicates forward direction\n";
				exit(1);
			}
			std::vector<rigibra::Transform> const & xforms = pairXform.second;

			// compute robustly rigid body transform to use for edge
			rigibra::Transform const fitXform
				{ orinet::robust::transformViaEffect
					(xforms.cbegin(), xforms.cend())
				};
			orinet::compare::Stats const stats
				{ orinet::compare::differenceStats
					(xforms.cbegin(), xforms.cend(), fitXform, false)
				};
			double const & fitMaxMagErr = stats.theMedMagDiff;

			// access corresponding graph nodes
			VertId const & fromVert = mapVertFromNdx.at(fromNdx);
			VertId const & intoVert = mapVertFromNdx.at(intoNdx);
			// check that from/into nodes are in graph
			bool const fromOkay{ network.has_vertex(fromVert) };
			bool const intoOkay{ network.has_vertex(intoVert) };
			if (! (fromOkay && intoOkay))
			{
				std::cerr << "bad vertex lookup\n" << std::endl;
				exit(8);
			}

			// add edge
			Edge const edge{ fitXform, fitMaxMagErr  };
			network.add_edge(fromVert, intoVert, edge);
		}

		return network;
	}

	//! Construct a label string for vertex station info
	inline
	std::string
	vertLabel
		( graaf::vertex_id_t const & vId
		, net::Station const & sta
		)
	{
		std::ostringstream lbl;
		lbl << "label="
			<< '"'
			<< vId << "='" << sta.theStaNdx << "'"
			<< '"';
		return lbl.str();
	}

	//! Construct a label string for backsight transform info
	inline
	std::string
	edgeLabel
		( graaf::edge_id_t const & eId
		, net::Edge const & edge
		)
	{
		std::ostringstream lbl;
		lbl << "label="
			<< '"'
			<< eId.first << "-->" << eId.second
			<< '\n'
			<< edge.get_weight()
			<< '"';
		return lbl.str();
	}

	//! \brief Save graph information to graphviz '.dot' graphic file.
	inline
	void
	saveNetworkGraphic
		( graaf::undirected_graph<net::Station, net::Edge> const network
		, std::filesystem::path const & dotPath
		)
	{
		graaf::io::to_dot(network, dotPath, vertLabel, edgeLabel);
	}

	//! Create (sub)graph from network, that contains only specified edges
	graaf::undirected_graph<Station, Edge>
	graphFromEdges
		( std::vector<graaf::edge_id_t> const & eIds
		, graaf::undirected_graph<net::Station, net::Edge> const & network
		)
	{
		graaf::undirected_graph<Station, Edge> outGraph;

		// add vertices to output graph and remember mapping
		std::map<graaf::vertex_id_t, graaf::vertex_id_t> mapOutFmInp;
		for (graaf::edge_id_t const & eId : eIds)
		{
			std::cout << "eId: " << eId.first << ',' << eId.second << '\n';
			std::array<graaf::vertex_id_t, 2u> const inps
				{ eId.first, eId.second };
			for (graaf::vertex_id_t const & inp : inps)
			{
				std::map<graaf::vertex_id_t, graaf::vertex_id_t>::const_iterator
					const itFind{ mapOutFmInp.find(inp) };
				if (mapOutFmInp.end() == itFind)
				{
					Station const & station = network.get_vertex(inp);
					mapOutFmInp[inp] = outGraph.add_vertex(station);
				}
			}
		}
		
		// add edges to output graph
		for (graaf::edge_id_t const & eId : eIds)
		{
			Edge const & edge = network.get_edge(eId);
			outGraph.add_edge
				( mapOutFmInp.at(eId.first)
				, mapOutFmInp.at(eId.second)
				, edge
				);
		}

		return outGraph;
	}

} // [net]


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
	std::cout << "\nHi from " << __FILE__ << '\n';

	//
	// Configuration parameters
	//
	/*
	constexpr std::size_t numStations{ 10u };
	constexpr std::size_t numBacksight{ 3u };
	constexpr std::size_t numMea{ 7u };
	constexpr std::size_t numErr{ 3u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };
	*/
	constexpr std::size_t numStations{ 8u };
	constexpr std::size_t numBacksight{ 2u };
	constexpr std::size_t numMea{ 2u };
	constexpr std::size_t numErr{ 0u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };

	//
	// Generate collection of expected station orientations
	// (used for generating simulation data)
	//
	std::vector<rigibra::Transform> const expStas
		{ sim::randomStations(numStations, locMinMax) };
std::cout << "number stations: " << expStas.size() << '\n';

	// simulate backsight observation data
	std::map<NdxPair, std::vector<rigibra::Transform> > const pairXforms
		{ sim::backsightTransforms
			(expStas, numBacksight, numMea, numErr, locMinMax)
		};
std::cout << "number backsights: " << pairXforms.size() << '\n';

	//
	// Populate graph: station frame nodes and robustly fit transform edges
	//

	graaf::undirected_graph<net::Station, net::Edge> const network
		{ net::graphFrom(expStas, pairXforms) };

	// save network topology to graphviz '.dot' file format
	saveNetworkGraphic(network, dotPathAll);

	//
	// Find minimum spanning tree
	//

	std::vector<graaf::edge_id_t> const eIds
		{ graaf::algorithm::kruskal_minimum_spanning_tree(network) };

	// The mst contents get updated below
	graaf::undirected_graph<net::Station, net::Edge> mst
		{ net::graphFromEdges(eIds, network) };

	saveNetworkGraphic(mst, dotPathMst);

	//
	// Update station orientations by traversing MST
	//

	struct Propagator
	{
		graaf::undirected_graph<net::Station, net::Edge> & theMst;
		std::vector<rigibra::Transform> const & theStas;

		inline
		void
		operator()
			( graaf::edge_id_t const & eId
			) const
		{
			graaf::vertex_id_t const & vId1 = eId.first;
			graaf::vertex_id_t const & vId2 = eId.second;
			net::Station & fromSta = theMst.get_vertex(vId1);
			net::Station & intoSta = theMst.get_vertex(vId2);
			std::size_t const & fromStaNdx = fromSta.theStaNdx;
			std::size_t const & intoStaNdx = intoSta.theStaNdx;
			net::Edge const & edge = theMst.get_edge(eId);

			rigibra::Transform const tmpIntoWrtFrom{ edge.theXform };
			rigibra::Transform xIntoWrtFrom{};
			if (fromStaNdx < intoStaNdx)
			{
				xIntoWrtFrom = tmpIntoWrtFrom;
			}
			else
			{
				xIntoWrtFrom = rigibra::inverse(tmpIntoWrtFrom);
			}

			theMst.remove_edge(vId1, vId2);

			if (! rigibra::isValid(fromSta.theGotXform))
			{
				fromSta.theGotXform = fromSta.theExpXform;
				std::cout << "\n\n@@@ setting fromSta.theGotXform"
					" for fromStaNdx: " << fromStaNdx << '\n';
			}
			rigibra::Transform const & xFromWrtRef = fromSta.theGotXform;
			intoSta.theGotXform = xIntoWrtFrom * xFromWrtRef;

			std::cout
				<< '\n'
				<< "Prop: eId: "
				<< vId1 << "-->" << vId2
				<< ' '
				<< "'" << fromStaNdx << "'-->'" << intoStaNdx << "'"
				<< '\n';
			std::cout
				<< "  fromWrtRef (exp): " << fromSta.theExpXform
				<< '\n'
				<< "  fromWrtRef (got): " << fromSta.theGotXform
				<< '\n';
			std::cout
				<< "   edgeIntoWrtFrom: " << xIntoWrtFrom
				<< '\n';
			std::cout
				<< "  intoWrtRef (exp): " << intoSta.theExpXform
				<< '\n'
				<< "  intoWrtRef (got): " << intoSta.theGotXform
				<< '\n';
		}

	}; // Propagator

	Propagator propagator{ mst, expStas };
	graaf::algorithm::breadth_first_traverse(mst, 0, propagator);
}

/*
	struct Station
	{
		std::size_t theStaNdx;
		rigibra::Transform theExpXform;
		rigibra::Transform theGotXform;

	}; // Station

	struct Edge : public graaf::weighted_edge<double>
	{
		rigibra::Transform theXform;
		double theMaxMagErr;
	...

	}

*/

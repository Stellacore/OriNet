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

namespace sim
{
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
				pairXforms.emplace_hint
					( pairXforms.end()
					, std::make_pair(NdxPair{fromNdx, intoNdx}, obsXforms)
					);
			}
		}

		return pairXforms;
	}

} // [sim]


namespace network
{
	using VertId = graaf::vertex_id_t;
	using EdgeId = graaf::edge_id_t;
	using StaNdx = std::size_t;
	using LoHiPair = std::pair<StaNdx, StaNdx>;

	/*! \brief Station Frame - i.e. associated with a rigid body pose.
	 *
	 */
	struct StaFrame
	{
		StaNdx theStaNdx;

	}; // StaFrame

	/*! \brief Rigid body orientation betweem two station frames.
	 *
	 * NOTE: the forward direction of the transformation is associated
	 * with StaFrame.theStaNdx values in the following sense.
	 * \arg If (frameA.theStaNdx < frameB.theStaNdx), then transform
	 *      theLoHiXform represents frame B w.r.t. A.
	 * \arg If (frameB.theStaNdx < frameA.theStaNdx), then transform
	 *      theLoHiXform represents frame A w.r.t. B.
	 *
	 * The inverse() function provides the transformation for an
	 * edge being traversed in the other direction.
	 */
	struct EdgeXform : public graaf::weighted_edge<double>
	{
		rigibra::Transform theLoHiXform{ rigibra::null<rigibra::Transform>() };
		double theFitErr{ engabra::g3::null<double>() };
		LoHiPair theNdxPair;

		//! Value ctor.
		inline
		explicit
		EdgeXform
			( rigibra::Transform const & lohiXform
			, double const & fitErr
			, LoHiPair const & fromIntoNdxs
			)
			: theLoHiXform{ lohiXform }
			, theFitErr{ fitErr }
			, theNdxPair{ fromIntoNdxs }
		{ }

		//! Construct with null/invalid member values.
		inline
		EdgeXform
			() = default;

		//! No-op dtor.
		inline
		~EdgeXform
			() = default;

		//! Edge weight (is transformation fit error - theFitErr)
		[[nodiscard]]
		inline
		double
		get_weight
			() const noexcept override
		{
			return theFitErr;
		}

		//! Sort in order of increasing edge weight (transformation error)
		inline
		bool
		operator<
			( EdgeXform const & other
			) const noexcept
		{
			return (this->get_weight() < other.get_weight());
		}

		//! True if this and other have different edge weights
		inline
		bool
		operator!=
			( EdgeXform const & other
			) const noexcept
		{
			return ((other < (*this)) || ((*this) < other));
		}

		//! Transformation (Hi-Ndx w.r.t. Lo-Ndx)
		inline
		rigibra::Transform const &
		xform
			() const
		{
			return theLoHiXform;
		}

		//! An instance associated with edge in reverse direction.
		inline
		EdgeXform
		inverse
			() const
		{
			LoHiPair ndxRev{ theNdxPair.second, theNdxPair.first };
			return EdgeXform
				(rigibra::inverse(theLoHiXform), theFitErr, ndxRev);
		}

	}; // EdgeXform

	//! Robust transformation computed from collection of transforms
	inline
	EdgeXform
	edgeXformMedianFit
		( std::vector<rigibra::Transform> const & xHiWrtLos
		, LoHiPair const & ndxPair
		)
	{
		// compute robust fit to collection of transforms
		rigibra::Transform const fitXform
			{ orinet::robust::transformViaEffect
				(xHiWrtLos.cbegin(), xHiWrtLos.cend())
			};
		// estimate quality of the fit value
		orinet::compare::Stats const stats
			{ orinet::compare::differenceStats
				(xHiWrtLos.cbegin(), xHiWrtLos.cend(), fitXform, false)
			};
		// generate weighted edge from the data
		double const & fitErr = stats.theMedMagDiff;
		return EdgeXform{fitXform, fitErr, ndxPair};
	}


	//! Construct a label string for vertex station info
	inline
	std::string
	vertLabel
		( graaf::vertex_id_t const & vId
		, StaFrame const & staFrame
		)
	{
		std::ostringstream lbl;
		lbl << "label="
			<< '"'
			<< vId << "='" << staFrame.theStaNdx << "'"
			<< '"';
		return lbl.str();
	}

	//! Construct a label string for backsight transform info
	inline
	std::string
	edgeLabel
		( graaf::edge_id_t const & eId
		, EdgeXform const & edgeXform
		)
	{
		std::ostringstream lbl;
		lbl << "label="
			<< '"'
			<< eId.first << "-->" << eId.second
			<< '\n'
			<< edgeXform.get_weight()
			<< '"';
		return lbl.str();
	}


	/*! \brief Representation of the geometry of a rigid body network.
	 *
	 * Uses a graph data structure to store StaFrame instances as nodes
	 * and rigid body transformations as edge relationships between them.
	 *
	 */
	struct Geometry
	{
		std::map<StaNdx, VertId> theVertIdFromStaNdx{};

		graaf::undirected_graph<StaFrame, EdgeXform> theGraph{};


		//! Check if staNdx already in graph, if not, then add vertex
		// Geometry::
		inline
		void
		ensureStaFrameExists
			( StaNdx const & staNdx
			)
		{
			if (theVertIdFromStaNdx.end() == theVertIdFromStaNdx.find(staNdx))
			{
				StaFrame const staFrame{ staNdx };
				VertId const vId{ theGraph.add_vertex(staFrame) };
				theVertIdFromStaNdx[staNdx] = vId;
			}
		}

		//! Graaf vertex ID value for station index
		// Geometry::
		inline
		VertId
		vertIdForStaNdx
			( StaNdx const & staNdx
			) const
		{
			return theVertIdFromStaNdx.at(staNdx);
		}

		//! External station index for Graaf vertex ID value
		// Geometry::
		inline
		StaNdx
		staNdxForVertId
			( VertId const & vertId
			) const
		{
			StaFrame const & staFrame = theGraph.get_vertex(vertId);
			return staFrame.theStaNdx;
		}

		//! Insert transformation edge into graph
		// Geometry::
		inline
		void
		addEdge
			( LoHiPair const & staNdxLoHi
			, EdgeXform const & edgeXform
			)
		{
			// check if vertices (station nodes) are already in the graph
			StaNdx const & sta1 = staNdxLoHi.first;
			StaNdx const & sta2 = staNdxLoHi.second;
			ensureStaFrameExists(sta1);
			ensureStaFrameExists(sta2);

			VertId const vId1{ vertIdForStaNdx(sta1) };
			VertId const vId2{ vertIdForStaNdx(sta2) };
			theGraph.add_edge(vId1, vId2, edgeXform);
		}

		//! Edges forming a minimum path
		// Geometry::
		inline
		std::vector<graaf::edge_id_t>
		spanningEdgeXforms
			() const
		{
			return graaf::algorithm::kruskal_minimum_spanning_tree(theGraph);
		}

		/*! Create an instance populated according to edge list
		 *
		 * E.g. calling this function with result of spanningEdgeXforms()
		 * will return a new network that minimally spans this original
		 * instance.
		 *
		 *  snippet TODO
		 */
		// Geometry::
		inline
		Geometry
		networkTree
			( std::vector<graaf::edge_id_t> const eIds
			) const
		{
			Geometry network{};

			for (graaf::edge_id_t const & eId : eIds)
			{
				// get vertex Ids
				VertId const & vId1 = eId.first;
				VertId const & vId2 = eId.second;

				// get edge data
				EdgeXform const & origEdge = theGraph.get_edge(eId);

				// get vertex data
				StaFrame const & staFrame1 = theGraph.get_vertex(vId1);
				StaFrame const & staFrame2 = theGraph.get_vertex(vId2);

				StaNdx const & staNdx1 = staFrame1.theStaNdx;
				StaNdx const & staNdx2 = staFrame2.theStaNdx;

				LoHiPair staNdxLoHi;
				EdgeXform useEdge{};
				if (staNdx1 < staNdx2)
				{
					staNdxLoHi = { staNdx1, staNdx2 };
					useEdge = origEdge;
				}
				else
				if (staNdx2 < staNdx1)
				{
					staNdxLoHi = { staNdx2, staNdx1 };
					useEdge = origEdge.inverse();
				}

				network.addEdge(staNdxLoHi, useEdge);
			}

			return network;
		}

		/*! Transformations computed by propagation through network
		 *
		 * Note that later computed transformations overwrite earlier ones.
		 * In general, this is method is probably most useful if run
		 * on a network that represents a minimum spanning tree.
		 */
		inline
		std::vector<rigibra::Transform>
		propagateXforms
			( StaNdx const & staNdx0
			, rigibra::Transform const & staXform0
			, std::size_t const & numStaNdxs
			, std::vector<rigibra::Transform> const & expStas
			) const
		{
			using namespace rigibra;
			static Transform const nullXform{ null<Transform>() };
			std::vector<rigibra::Transform> gotXforms(numStaNdxs, nullXform);

			// set first station orientation
			gotXforms[staNdx0] = staXform0;

			//! Functor for processing encountered edges
			struct Propagator
			{
				Geometry const & theGeo;
				std::vector<rigibra::Transform> & gotStas;
				std::vector<rigibra::Transform> const & expStas;

				inline
				void
				operator()
					( graaf::edge_id_t const & eId
					) const
				{
					VertId const & vId1 = eId.first;
					VertId const & vId2 = eId.second;
					StaNdx const staNdx1{ theGeo.staNdxForVertId(vId1) };
					StaNdx const staNdx2{ theGeo.staNdxForVertId(vId2) };

					EdgeXform const & edgeXform = theGeo.theGraph.get_edge(eId);

					EdgeXform useEdgeXform{ edgeXform };
					LoHiPair ndxLoHiPair{ staNdx1, staNdx2 };
					if (staNdx2 < staNdx1)
					{
						useEdgeXform = edgeXform.inverse();
						ndxLoHiPair = LoHiPair{ staNdx2, staNdx1 };
					}

					using namespace rigibra;
					StaNdx const loNdx = ndxLoHiPair.first;
					StaNdx const hiNdx = ndxLoHiPair.second;
					Transform const & x1wRef = gotStas[loNdx];
					Transform const & x2wRef = gotStas[hiNdx];

					if (isValid(x1wRef))
					{
						// propagate from 1 forward into 2
						Transform const x2w1{ useEdgeXform.xform() };
						Transform const x2wRef{ x2w1 * x1wRef };
						gotStas[hiNdx] = x2wRef;
					}
					else
					if (isValid(x2wRef))
					{
						// propagate from 2 back to 1
						Transform const x1w2{ useEdgeXform.xform() };
						Transform const x1wRef{ x1w2 * x2wRef };
						gotStas[loNdx] = x1wRef;
					}
					else
					{
						std::cerr << "FATAL ERROR - bad Graph sort\n";
						exit(1);
					}
				}

			}; // Propagator;

			VertId const vId0{ vertIdForStaNdx(staNdx0) };
			Propagator const propagator{ *this, gotXforms, expStas };
			graaf::algorithm::breadth_first_traverse
				(theGraph, vId0, propagator);

			return gotXforms;
		}

		//! \brief Save graph information to graphviz '.dot' graphic file.
		// Geometry::
		inline
		void
		saveNetworkGraphic
			( std::filesystem::path const & dotPath
			) const
		{
			graaf::io::to_dot(theGraph, dotPath, vertLabel, edgeLabel);
		}

	}; // Geometry

} // [network]


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

//#define EasyCase
#	if defined(EasyCase)
	constexpr std::size_t numStations{ 8u };
	constexpr std::size_t numBacksight{ 2u };
	constexpr std::size_t numMea{ 1u };
	constexpr std::size_t numErr{ 0u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };
#	else // EasyCase
	constexpr std::size_t numStations{ 10u };
	constexpr std::size_t numBacksight{ 3u };
	constexpr std::size_t numMea{ 7u };
	constexpr std::size_t numErr{ 3u };
	constexpr std::pair<double, double> locMinMax{ 0., 100. };
#	endif // EasyCase

	//
	// Generate collection of expected station orientations
	// (used for generating simulation data)
	//
	std::vector<rigibra::Transform> const expStas
		{ sim::sequentialStations(numStations) };
	//	{ sim::randomStations(numStations, locMinMax) };

	// simulate backsight observation data
	std::map<NdxPair, std::vector<rigibra::Transform> > const pairXforms
		{ sim::backsightTransforms
			(expStas, numBacksight, numMea, numErr, locMinMax)
		};

	//
	// Populate graph: station frame nodes and robustly fit transform edges
	//

	network::Geometry geoNet;

	for (std::map<NdxPair, std::vector<rigibra::Transform> >::value_type
		const & pairXform : pairXforms)
	{
		// compute robustly fit transformation for this edge
		network::EdgeXform const edgeXform
			{ network::edgeXformMedianFit(pairXform.second, pairXform.first) };

		// insert robust transform into network
		geoNet.addEdge(pairXform.first, edgeXform);
	}


	// save network topology to graphviz '.dot' file format
	geoNet.saveNetworkGraphic(dotPathAll);

	//
	// Find minimum spanning tree
	//

	std::vector<graaf::edge_id_t> const mstEdgeIds
		{ geoNet.spanningEdgeXforms() };

	network::Geometry const mstNet{ geoNet.networkTree(mstEdgeIds) };

	mstNet.saveNetworkGraphic(dotPathMst);

	//
	// Update station orientations by traversing MST
	//

	// traverse mst from node 0 (since 0 always present in non-empty graph)
	network::StaNdx const staNdx0{ 0u };
	rigibra::Transform const & staXform0 = expStas[staNdx0];
	std::size_t const numStas{ expStas.size() };
	std::vector<rigibra::Transform> const gotStas
		{ mstNet.propagateXforms(staNdx0, staXform0, numStas, expStas) };
	//	{ geoNet.propagateXforms(staNdx0, staXform0, numStas) };

	constexpr bool showResult{ true };
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


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
\brief Functions for creating and processing orientation network data.

Example:
\snippet test_network.cpp DoxyExample01

*/

#include "OriNet/network.hpp"

#include "OriNet/compare.hpp"
#include "OriNet/robust.hpp"

#include <Engabra>
#include <graaflib/algorithm/graph_traversal/breadth_first_search.h>
#include <graaflib/algorithm/minimum_spanning_tree/kruskal.h>
#include <graaflib/graph.h>
#include <graaflib/io/dot.h>
#include <Rigibra>

#include <filesystem>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>


namespace orinet
{

namespace network
{

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
		, EdgeOri const & edgeOri
		)
	{
		std::ostringstream lbl;
		lbl << "label="
			<< '"'
			<< eId.first << "-->" << eId.second
			<< '\n'
			<< edgeOri.get_weight()
			<< '"';
		return lbl.str();
	}


	EdgeOri
	edgeOriMedianFit
		( std::vector<rigibra::Transform> const & xHiWrtLos
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
		return EdgeOri{fitXform, fitErr};
	}


void
Geometry :: ensureStaFrameExists
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

VertId
Geometry :: vertIdForStaNdx
	( StaNdx const & staNdx
	) const
{
	return theVertIdFromStaNdx.at(staNdx);
}

StaNdx
Geometry :: staNdxForVertId
	( VertId const & vertId
	) const
{
	StaFrame const & staFrame = theGraph.get_vertex(vertId);
	return staFrame.theStaNdx;
}

// public:

void
Geometry :: addEdge
	( LoHiPair const & staNdxLoHi
	, EdgeOri const & edgeOri
	)
{
	// check if vertices (station nodes) are already in the graph
	StaNdx const & sta1 = staNdxLoHi.first;
	StaNdx const & sta2 = staNdxLoHi.second;
	ensureStaFrameExists(sta1);
	ensureStaFrameExists(sta2);

	VertId const vId1{ vertIdForStaNdx(sta1) };
	VertId const vId2{ vertIdForStaNdx(sta2) };
	theGraph.add_edge(vId1, vId2, edgeOri);
}

std::vector<graaf::edge_id_t>
Geometry :: spanningEdgeOris
	() const
{
	return graaf::algorithm::kruskal_minimum_spanning_tree(theGraph);
}

std::size_t
Geometry :: sizeVerts
	() const
{
	return theGraph.vertex_count();
}

std::size_t
Geometry :: sizeEdges
	() const
{
	return theGraph.edge_count();
}

Geometry
Geometry :: networkTree
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
		EdgeOri const & origEdge = theGraph.get_edge(eId);

		// get vertex data
		StaFrame const & staFrame1 = theGraph.get_vertex(vId1);
		StaFrame const & staFrame2 = theGraph.get_vertex(vId2);

		StaNdx const & staNdx1 = staFrame1.theStaNdx;
		StaNdx const & staNdx2 = staFrame2.theStaNdx;

		// set transformation edge consistent with LoHiNdx convention
		LoHiPair staNdxLoHi;
		EdgeOri useEdge{};
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

std::vector<rigibra::Transform>
Geometry :: propagateTransforms
	( StaNdx const & staNdx0
	, rigibra::Transform const & staXform0
	) const
{
	std::vector<rigibra::Transform> gotXforms;

	using namespace rigibra;

	std::size_t const numStaNdxs{ theGraph.vertex_count() };
	if (0u < numStaNdxs)
	{
		gotXforms.resize(numStaNdxs);
		static Transform const nullXform{ null<Transform>() };
		std::fill(gotXforms.begin(), gotXforms.end(), nullXform);

		// set first station orientation
		gotXforms[staNdx0] = staXform0;

		struct Propagator
		{
			Geometry const & theGeo;
			std::vector<rigibra::Transform> & gotStas;

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

				EdgeOri const & edgeOri = theGeo.theGraph.get_edge(eId);

				EdgeOri useEdgeOri{ edgeOri };
				LoHiPair ndxLoHiPair{ staNdx1, staNdx2 };
				if (staNdx2 < staNdx1)
				{
					useEdgeOri = edgeOri.inverse();
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
					Transform const x2w1{ useEdgeOri.xform() };
					Transform const x2wRef{ x2w1 * x1wRef };
					gotStas[hiNdx] = x2wRef;
				}
				else
				if (isValid(x2wRef))
				{
					// propagate from 2 back to 1
					Transform const x1w2{ useEdgeOri.xform() };
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
		Propagator const propagator{ *this, gotXforms};
		graaf::algorithm::breadth_first_traverse
			(theGraph, vId0, propagator);
	}

	return gotXforms;
}

void
Geometry :: saveNetworkGraphic
	( std::filesystem::path const & dotPath
	) const
{
	graaf::io::to_dot(theGraph, dotPath, vertLabel, edgeLabel);
}


} // [network]

} // [orinet]


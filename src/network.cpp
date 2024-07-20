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

#include <algorithm>
#include <filesystem>
#include <iomanip>
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
			<< vId << "='" << staFrame.theStaKey << "'"
			<< '"';
		return lbl.str();
	}

	//! Construct a label string for backsight transform info
	inline
	std::string
	edgeLabel
		( graaf::edge_id_t const & eId
		, EdgeOri const & edge
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


void
Geometry :: ensureStaFrameExists
	( StaKey const & staKey
	)
{
	if (theVertIdFromStaKey.end() == theVertIdFromStaKey.find(staKey))
	{
		StaFrame const staFrame{ staKey };
		VertId const vId{ theGraph.add_vertex(staFrame) };
		theVertIdFromStaKey[staKey] = vId;
	}
}

VertId
Geometry :: vertIdForStaKey
	( StaKey const & staKey
	) const
{
	return theVertIdFromStaKey.at(staKey);
}

StaKey
Geometry :: staKeyForVertId
	( VertId const & vertId
	) const
{
	StaFrame const & staFrame = theGraph.get_vertex(vertId);
	return staFrame.theStaKey;
}

// public:

void
Geometry :: addEdge
	( LoHiKeyPair const & staKeyLoHi
	, EdgeOri const & edge
	)
{
	// check if vertices (station nodes) are already in the graph
	StaKey const & sta1 = staKeyLoHi.first;
	StaKey const & sta2 = staKeyLoHi.second;

	ensureStaFrameExists(sta1);
	ensureStaFrameExists(sta2);

	VertId const vId1{ vertIdForStaKey(sta1) };
	VertId const vId2{ vertIdForStaKey(sta2) };
	theGraph.add_edge(vId1, vId2, edge);
}

std::vector<graaf::edge_id_t>
Geometry :: spanningEdgeOris
	() const
{
	return graaf::algorithm::kruskal_minimum_spanning_tree(theGraph);
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

		StaKey const & staKey1 = staFrame1.theStaKey;
		StaKey const & staKey2 = staFrame2.theStaKey;

		// set transformation edge consistent with LoHiNdx convention
		LoHiKeyPair staKeyLoHi;
		EdgeOri useEdge{};
		if (staKey1 < staKey2)
		{
			staKeyLoHi = { staKey1, staKey2 };
			useEdge = origEdge;
		}
		else
		if (staKey2 < staKey1)
		{
			staKeyLoHi = { staKey2, staKey1 };
			useEdge = origEdge.edgeReversed();
		}

		network.addEdge(staKeyLoHi, useEdge);
	}

	return network;
}

std::vector<rigibra::Transform>
Geometry :: propagateTransforms
	( StaKey const & // staKey0
	, rigibra::Transform const & // staXform0
	) const
{
	std::vector<rigibra::Transform> gotXforms;

/*
	using namespace rigibra;

	std::size_t const numStaKeys{ theGraph.vertex_count() };
	if (0u < numStaKeys)
	{
		gotXforms.resize(numStaKeys);
		static Transform const nullXform{ null<Transform>() };
		std::fill(gotXforms.begin(), gotXforms.end(), nullXform);

		// set first station orientation
		gotXforms[staKey0] = staXform0;

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
				StaKey const staKey1{ theGeo.staKeyForVertId(vId1) };
				StaKey const staKey2{ theGeo.staKeyForVertId(vId2) };

				EdgeOri const & edgeOri = theGeo.theGraph.get_edge(eId);

				EdgeOri useEdgeOri{ edgeOri };
				LoHiKeyPair lohiKeys{ staKey1, staKey2 };
				if (staKey2 < staKey1)
				{
					useEdgeOri = edgeOri.edgeReversed();
					lohiKeys = LoHiKeyPair{ staKey2, staKey1 };
				}

				using namespace rigibra;
				StaKey const loNdx = lohiKeys.first;
				StaKey const hiNdx = lohiKeys.second;
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

		VertId const vId0{ vertIdForStaKey(staKey0) };
		Propagator const propagator{ *this, gotXforms};
		graaf::algorithm::breadth_first_traverse
			(theGraph, vId0, propagator);
	}
*/

	return gotXforms;
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

std::string
Geometry :: infoString
	( std::string const & title
	) const
{
	std::ostringstream oss;
	if (! title.empty())
	{
		oss << title << ' ';
	}
	oss
		<< "sizeVerts: " << sizeVerts()
		<< ' '
		<< "sizeEdges: " << sizeEdges()
		<< '\n';
	return oss.str();
}

std::string
Geometry :: infoStringContents
	( std::string const & title
	) const
{
	std::ostringstream oss;
	//
	using GType = graaf::undirected_graph<StaFrame, EdgeOri>;
	// Buffer results so that they can be sorted for output
	// Wastes memory and time, but makes output *MUCH* easier to read.
	std::vector<std::string> infoVerts;
	std::vector<std::string> infoEdges;
	//
	// report vertices
	GType::vertex_id_to_vertex_t const & vTypeById = theGraph.get_vertices();
	for (GType::vertex_id_to_vertex_t ::const_iterator
		iter{vTypeById.cbegin()} ; vTypeById.cend() != iter ; ++iter)
	{
		std::ostringstream tmpOss;
		graaf::vertex_id_t const & vId = iter->first;
		GType::vertex_t const & vType = vTypeById.at(vId);
		tmpOss
		//	<< "VertId: " << vId
		//	<< ' '
			<< "VertKey: " << std::setw(8u) << vType.key()
			;
		infoVerts.emplace_back(tmpOss.str());
	}
	//
	// report edges
	GType::edge_id_to_edge_t const & eTypeById = theGraph.get_edges();
	for (GType::edge_id_to_edge_t ::const_iterator
		iter{eTypeById.cbegin()} ; eTypeById.cend() != iter ; ++iter)
	{
		std::ostringstream tmpOss;
		graaf::edge_id_t const & eId = iter->first;
		GType::edge_t const & eType = eTypeById.at(eId);
		tmpOss
		//	<< "EdgeId:From,Into: "
		//	<< "Ids: " << eId.first << ", " << eId.second
		//	<< ' '
			<< "EdgeKey:From,Into: "
				<< std::setw(8u) << vTypeById.at(eId.first).key()
				<< ' '
				<< std::setw(8u) << vTypeById.at(eId.second).key()
				<< ' '
				<< std::setw(12u) << std::fixed << eType.get_weight()
				;
		infoEdges.emplace_back(tmpOss.str());
	}

	// sort vertice and edges
	std::sort(infoVerts.begin(), infoVerts.end());
	std::sort(infoEdges.begin(), infoEdges.end());

	oss << infoString(title);
	oss << "vertices...\n";
	for (std::string const & infoVert : infoVerts)
	{
		oss << infoVert << '\n';
	}
	oss << "edges...\n";
	for (std::string const & infoEdge : infoEdges)
	{
		oss << infoEdge << '\n';
	}

	return oss.str();
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


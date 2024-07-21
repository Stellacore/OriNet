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

#include "OriNet/networkGeometry.hpp"

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
		, std::shared_ptr<EdgeBase> const & ptEdge
		)
	{
		std::ostringstream lbl;
		lbl << "label="
			<< '"'
			<< eId.first << "-->" << eId.second
			<< '\n'
			<< ptEdge->get_weight()
			<< '"';
		return lbl.str();
	}


bool
Geometry :: hasStaKey
	( StaKey const & staKey
	) const
{
	return (theVertIdFromStaKey.end() != theVertIdFromStaKey.find(staKey));
}

void
Geometry :: ensureStaFrameExists
	( StaKey const & staKey
	)
{
	if (! hasStaKey(staKey))
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
	VertId id{ sNullKey };
	if (hasStaKey(staKey))
	{
		id = theVertIdFromStaKey.at(staKey);
	}
	return id;
}

StaKey
Geometry :: staKeyForVertId
	( VertId const & vertId
	) const
{
	StaKey staKey{ sNullKey };
	if (theGraph.has_vertex(vertId))
	{
		StaFrame const & staFrame = theGraph.get_vertex(vertId);
		staKey = staFrame.theStaKey;
	}
	return staKey;
}

std::shared_ptr<EdgeBase>
Geometry :: edgeBaseForEdgeId
	( graaf::edge_id_t const & eId
	) const
{
	std::shared_ptr<EdgeBase> ptUseEdge{ nullptr };

	VertId const & vId1 = eId.first;
	VertId const & vId2 = eId.second;
	if (theGraph.has_vertex(vId1) && theGraph.has_vertex(vId2))
	{
		StaKey const staKey1{ staKeyForVertId(vId1) };
		StaKey const staKey2{ staKeyForVertId(vId2) };

		// edge from graph traversal
		std::shared_ptr<EdgeBase> const & ptGraphEdge = theGraph.get_edge(eId);

		// check if edge needs to be reversed
		EdgeDir const & haveDir = ptGraphEdge->edgeDir();
		EdgeDir const wantDir{ staKey1, staKey2 };
		EdgeDir::DirCompare const dirComp{ wantDir.compareTo(haveDir) };
		if (EdgeDir::Forward == dirComp)
		{
			// edge for use in computation (may need reversing)
			ptUseEdge = ptGraphEdge;
		}
		else
		if (EdgeDir::Reverse == dirComp)
		{
			ptUseEdge = ptGraphEdge->reversedInstance();
		}
		else
		// if (EdgeDir::Different == dirComp)
		{
			std::cerr << "Fatal: bad network construction\n"
				<< "haveDir: " << haveDir << '\n'
				<< "wantDir: " << wantDir << '\n'
				;
		}
	}

	// edge from graph traversal
	return ptUseEdge;
}

void
Geometry::Propagator :: operator()
	( graaf::edge_id_t const & eId
	) const
{
	// Obtain edge tranform matching graph traversal direction

	std::shared_ptr<EdgeBase> const ptUseEdge
		{ thePtGeo->edgeBaseForEdgeId(eId) };

	//
	// Propagate transform in graph traveral direction
	//

	using namespace rigibra;

	// keys for accessing absolute orientation map being built
	StaKey const fromKey{ ptUseEdge->fromKey() };
	StaKey const intoKey{ ptUseEdge->intoKey() };

	// get starting transform wrt Ref (from prior activity)
	std::map<StaKey, Transform>::const_iterator
		const itFrom{ thePtStaXforms->find(fromKey) };
	Transform xFromWrtRef{ null<Transform>() };
	if (thePtStaXforms->end() != itFrom)
	{
		xFromWrtRef = itFrom->second;
	}
	else
	{
		std::cerr << "FATAL ERROR - bad Graph xFromWrtRef\n";
		// exit(1);
	}

	// get (re)directed edge transform Into wrt From
	Transform xIntoWrtFrom{ null<Transform>() };
	if (isValid(xFromWrtRef))
	{
		xIntoWrtFrom = ptUseEdge->xform();
	}
	else
	{
		std::cerr << "FATAL ERROR - bad Graph useEdge\n";
		// exit(1);
	}

	// compute ending propagated transform
	Transform xIntoWrtRef{ xIntoWrtFrom * xFromWrtRef };
	(*thePtStaXforms)[intoKey] = xIntoWrtRef;
}

// public:

void
Geometry :: insertEdge
	( std::shared_ptr<EdgeBase> const & ptEdge
	)
{
	// check if vertices (station nodes) are already in the graph
	StaKey const & sta1 = ptEdge->fromKey();
	StaKey const & sta2 = ptEdge->intoKey();

	ensureStaFrameExists(sta1);
	ensureStaFrameExists(sta2);

	VertId const vId1{ vertIdForStaKey(sta1) };
	VertId const vId2{ vertIdForStaKey(sta2) };
	if (! (isValid(vId1) && isValid(vId2)))
	{
		std::cerr << "FATAL: Geometry::insertEdge bad vertex management\n"
			<< " sta1: " << sta1
			<< " sta2: " << sta2
			<< " vId1: " << vId1
			<< " vId2: " << vId2
			<< '\n';
		exit(1);
	}
	theGraph.add_edge(vId1, vId2, ptEdge);
}

std::shared_ptr<EdgeBase>
Geometry :: edge
	( EdgeDir const & edgeDir
	) const
{
	std::shared_ptr<EdgeBase> ptEdge{ nullptr };

	StaKey const & sta1 = edgeDir.fromKey();
	StaKey const & sta2 = edgeDir.intoKey();

	VertId const vId1{ vertIdForStaKey(sta1) };
	VertId const vId2{ vertIdForStaKey(sta2) };
	if (isValid(vId1) && isValid(vId2))
	{
		if (theGraph.has_edge(vId1, vId2))
		{
			ptEdge = theGraph.get_edge(vId1, vId2);
		}
		else
		if (theGraph.has_edge(vId2, vId1))
		{
			ptEdge = theGraph.get_edge(vId2, vId1);
		}
	}

	return ptEdge;
}

std::vector<graaf::edge_id_t>
Geometry :: spanningEdgeBases
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
		std::shared_ptr<EdgeBase> const & ptOrigEdge = theGraph.get_edge(eId);

		// get vertex data
		StaFrame const & staFrame1 = theGraph.get_vertex(vId1);
		StaFrame const & staFrame2 = theGraph.get_vertex(vId2);

		StaKey const & staKey1 = staFrame1.theStaKey;
		StaKey const & staKey2 = staFrame2.theStaKey;

		// set transformation edge consistent with LoHiNdx convention
		std::shared_ptr<EdgeBase> ptUseEdge{ std::make_shared<EdgeBase>() };
		if (staKey1 < staKey2)
		{
			ptUseEdge = ptOrigEdge;
		}
		else
		if (staKey2 < staKey1)
		{
			ptUseEdge = ptOrigEdge->reversedInstance();
		}

		network.insertEdge(ptUseEdge);
	}

	return network;
}

std::map<StaKey, rigibra::Transform>
Geometry :: propagateTransforms
	( StaKey const & staKey0
	, rigibra::Transform const & staXform0
	) const
{
	std::map<StaKey, rigibra::Transform> staXforms;

	std::size_t const numStaKeys{ theGraph.vertex_count() };
	if (0u < numStaKeys)
	{
		// set first station orientation
		staXforms[staKey0] = staXform0;

		VertId const vId0{ vertIdForStaKey(staKey0) };
		if (isValid(vId0))
		{
			Propagator const propagator{ this, &staXforms };
			graaf::algorithm::breadth_first_traverse
				(theGraph, vId0, propagator);
		}
		else
		{
			std::cerr << "Invalid initial station reference:"
				<< " staKey0: " << staKey0
				<< " vId0: " << vId0
				<< '\n';
		}
	}

	return staXforms;
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
	using GType = graaf::undirected_graph<StaFrame, std::shared_ptr<EdgeBase> >;
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
		std::shared_ptr<EdgeBase> const edgeBase{ edgeBaseForEdgeId(eId) };
		if (edgeBase->fromKey() < edgeBase->intoKey())
		{
			tmpOss << "EdgeId: " << edgeBase->infoString();
		}
		else
		{
			tmpOss << "EdgeId: " << edgeBase->reversedInstance()->infoString();
		}
		infoEdges.emplace_back(tmpOss.str());
	}

	// sort vertice and edges
	std::sort(infoVerts.begin(), infoVerts.end());
	std::sort(infoEdges.begin(), infoEdges.end());

	oss << infoString(title);
	oss << "vertices...";
	for (std::string const & infoVert : infoVerts)
	{
		oss << '\n' << infoVert;
	}
	oss << '\n';
	oss << "edges...";
	for (std::string const & infoEdge : infoEdges)
	{
		oss << '\n' << infoEdge;
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


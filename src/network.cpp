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
	theGraph.add_edge(vId1, vId2, ptEdge);
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

		network.addEdge(ptUseEdge);
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

	using namespace rigibra;

	std::size_t const numStaKeys{ theGraph.vertex_count() };
	if (0u < numStaKeys)
	{
		// set first station orientation
		staXforms[staKey0] = staXform0;

std::cout << "::staKey0: " << staKey0 << '\n';
std::cout << "::staXform0: " << staXform0 << '\n';

		struct Propagator
		{
			Geometry const * const thePtGeo;
			std::map<StaKey, rigibra::Transform> * const thePtStaXforms;

			inline
			void
			operator()
				( graaf::edge_id_t const & eId
				) const
			{
std::cout << '\n';
std::cout << "--thePtStaXforms.size: " << thePtStaXforms->size() << '\n';
for (std::map<StaKey, rigibra::Transform>::value_type
	const & staXform : *thePtStaXforms)
{
	std::cout
		<< "--StaXforms: " << staXform.first
		<< ' ' << staXform.second
		<< "  isValid: " << isValid(staXform.second)
		<< '\n';
}

				VertId const & vId1 = eId.first;
				VertId const & vId2 = eId.second;
				StaKey const staKey1{ thePtGeo->staKeyForVertId(vId1) };
				StaKey const staKey2{ thePtGeo->staKeyForVertId(vId2) };

//std::cout << "::vId1: " << vId1 << '\n';
//std::cout << "::vId2: " << vId2 << '\n';
std::cout << "::staKey1: " << staKey1 << '\n';
std::cout << "::staKey2: " << staKey2 << '\n';

				std::shared_ptr<EdgeBase>
					const & ptGraphEdge = thePtGeo->theGraph.get_edge(eId);

std::cout << "::ptGraphEdge: " << *ptGraphEdge << '\n';

				std::shared_ptr<EdgeBase> ptUseEdge = ptGraphEdge;

std::cout << ":: useEdge(a): " << *ptUseEdge << '\n';

				EdgeDir const & haveDir = ptGraphEdge->edgeDir();
				EdgeDir const wantDir{ staKey1, staKey2 };
				if (EdgeDir::Reverse == wantDir.compareTo(haveDir))
				{
std::cout << ":: ----- REVERSE\n";
					ptUseEdge = ptGraphEdge->reversedInstance();
				}
std::cout << ":: useEdge(b): " << *ptUseEdge << '\n';

				StaKey const fromKey{ ptUseEdge->fromKey() };
				StaKey const intoKey{ ptUseEdge->intoKey() };

std::cout << "::fromKey: " << fromKey << '\n';
std::cout << "::intoKey: " << intoKey << '\n';

				using namespace rigibra;

				// get starting transform wrt Ref
				std::map<StaKey, Transform>::const_iterator
					const itFrom{ thePtStaXforms->find(fromKey) };
				Transform xFromWrtRef{ null<Transform>() };
				if (thePtStaXforms->end() != itFrom)
				{
					xFromWrtRef = itFrom->second;
std::cout << "::xFromWrtRef: " << xFromWrtRef << '\n';
				}
				else
				{
					std::cerr << "FATAL ERROR - bad Graph xFromWrtRef\n";
					exit(1);
				}

				// get (re)directed edge transform Into wrt From
				Transform xIntoWrtFrom{ null<Transform>() };
				if (isValid(xFromWrtRef))
				{
std::cout << "::useE.xform(): " << ptUseEdge->xform() << '\n';
					xIntoWrtFrom = ptUseEdge->xform();
				}
				else
				{
					std::cerr << "FATAL ERROR - bad Graph useEdge\n";
					exit(1);
				}
std::cout << "::xIntoWrtFrom: " << xIntoWrtFrom << '\n';

				// compute ending location
				Transform xIntoWrtRef{ xIntoWrtFrom * xFromWrtRef };
				(*thePtStaXforms)[intoKey] = xIntoWrtRef;
std::cout << "::xIntoWrtRef : " << xIntoWrtRef << '\n';

			}

		}; // Propagator;

		VertId const vId0{ vertIdForStaKey(staKey0) };
std::cout << "::staKey0: " << staKey0 << '\n';
std::cout << "::vId0   : " << vId0 << '\n';
		Propagator const propagator{ this, &staXforms };
		graaf::algorithm::breadth_first_traverse
			(theGraph, vId0, propagator);
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
				<< std::setw(12u) << std::fixed << eType->get_weight()
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


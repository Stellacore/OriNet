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


#ifndef OriNet_network_Geometry_INCL_
#define OriNet_network_Geometry_INCL_

/*! \file
\brief Functions for creating and processing orientation network data.

*/

#include "networkEdge.hpp"

#include <Engabra>
#include <graaflib/graph.h>
#include <Rigibra>

#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>


namespace orinet
{

namespace network
{
	/*! \brief Representation of the geometry of a rigid body network.
	 *
	 * Uses a graph data structure to store StaFrame instances as nodes
	 * and rigid body transformations as edge relationships between them.
	 *
	 * Example: (from test/test_network.cpp)
	 *
	 * Use orinet::random functions to simulate station orientations
	 * for use in simulating edge relative orientations.
	 * \snippet test_network.cpp DoxyExampleSim
	 *
	 * Construct a network::Geometry instance for network analysis below.
	 * \snippet test_network.cpp DoxyExampleCreate
	 *
	 * Simulate edge relative orientations - in practical application
	 * these would likely be the input data for orientation network
	 * formation and analysis.
	 * \snippet test_network.cpp DoxyExampleEdges
	 *
	 * Find an optimum connectivity through the network by which every
	 * station is transitively connected to every other. Connection
	 * path is the one that minmimzes the cumulative relative orientation
	 * 'fitErr' sum.
	 * \snippet test_network.cpp DoxyExampleThin
	 *
	 * Compute each station orientation using the thinned (minimum
	 * spanning tree) orientation network.
	 * \snippet test_network.cpp DoxyExamplePropagate
	 */
	class Geometry
	{
		//! Lookup map: station data index from graph vertex index 
		std::map<StaKey, VertId> theVertIdFromStaKey{};

		//! Graph data structure for storing/processing network relationships
		graaf::undirected_graph<StaFrame, std::shared_ptr<EdgeBase> >
			theGraph{};

		//! True if station is already a node in graph
		bool
		hasStaKey
			( StaKey const & staKey
			) const;

		//! Check if staKey already in graph, if not, then add vertex
		void
		ensureStaFrameExists
			( StaKey const & staKey
			);

		//! Graaf vertex ID value for station index
		VertId
		vertIdForStaKey
			( StaKey const & staKey
			) const;

		//! External station index for Graaf vertex ID value
		StaKey
		staKeyForVertId
			( VertId const & vertId
			) const;

	public:

		/*! \brief Insert transformation edge into graph
		 *
		 * Example:
	 	 * \snippet test_network.cpp DoxyExampleThin
		 */
		void
		insertEdge
			( std::shared_ptr<EdgeBase> const & ptEdge
			);

		//! Edge (expressed in order of edgeDir key values).
		std::shared_ptr<EdgeBase>
		edge
			( EdgeDir const & edgeDir
			) const;

		//! Edges forming a minimum path
		std::vector<graaf::edge_id_t>
		spanningEdgeBases
			() const;

		/*! \brief Create an instance populated according to edge list
		 *
		 * E.g. calling this function with result of spanningEdgeOris()
		 * will return a new network that minimally spans this original
		 * instance.
		 *
		 * Example:
	 	 * \snippet test_network.cpp DoxyExampleThin
		 */
		Geometry
		networkTree
			( std::vector<graaf::edge_id_t> const eIds
			) const;

		/*! \brief Transformations computed by propagation through network
		 *
		 * Note that later computed transformations overwrite earlier ones.
		 * In general, this is method is probably most useful if run
		 * on a network that represents a minimum spanning tree.
		 *
		 * Example:
	 	 * \snippet test_network.cpp DoxyExamplePropagate
		 */
		std::map<StaKey, rigibra::Transform>
		propagateTransforms
			( StaKey const & staKey0
			, rigibra::Transform const & staXform0
			) const;

		//! Number of vertices in graph
		std::size_t
		sizeVerts
			() const;

		//! Number of edges in graph
		std::size_t
		sizeEdges
			() const;

		//! \brief Descriptive information about this instance
		std::string
		infoString
			( std::string const & title = {}
			) const;

		/*! \brief Detailed information about this instance
		 *
		 * \note The Vertex type (StaFrame) must have operator<<()
		 * overloaded!
		 */
		std::string
		infoStringContents
			( std::string const & title = {}
			) const;

		//! \brief Save graph information to graphviz '.dot' graphic file.
		void
		saveNetworkGraphic
			( std::filesystem::path const & dotPath
			) const;

	}; // Geometry

} // [network]


} // [orinet]

namespace
{
	//! Put object info to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::network::StaFrame const & staFrame
		)
	{
		ostrm << staFrame.key();
		return ostrm;
	}

	//! Put instance to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::network::Geometry const & geo
		)
	{
		ostrm << geo.infoString();
		return ostrm;
	}

} //[anon]


#endif // OriNet_network_Geometry_INCL_

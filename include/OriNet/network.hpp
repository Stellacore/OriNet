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


#ifndef OriNet_network_INCL_
#define OriNet_network_INCL_

/*! \file
\brief Functions for creating and processing orientation network data.

*/

#include <Engabra>
#include <graaflib/graph.h>
#include <Rigibra>

#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>


namespace orinet
{

namespace network
{
	//! Vertex type - used in graph structure library
	using VertId = graaf::vertex_id_t;

	//! Edge type - used in graph structure library
	using EdgeId = graaf::edge_id_t;
	// using EdgeId = std::pair<std::size_t, std::size_t>;

	//! Station orientations referenced by index (e.g. to external collection)
	using StaKey = std::size_t;

	using LoHiKeyPair = std::pair<StaKey, StaKey>;

	/*! \brief Station Frame - i.e. associated with a rigid body pose.
	 *
	 */
	struct StaFrame
	{
		StaKey const theStaKey{ std::numeric_limits<StaKey>::max() };

		inline
		StaKey
		key
			() const
		{
			return theStaKey;
		}

	}; // StaFrame

	/*! \brief Ordered pair of station keys for edge direction interpretation.
	 *
	 * A graph structure is used to model the network connectivity and is
	 * execeptionally useful for network traversal operations. However,
	 * some algorithms (minimum spanning tree in particular) require an
	 * undirected graph. This is contrary to the directed nature of
	 * rigid body orientations between two station nodes.
	 *
	 * To resolve the directed/undirected contention, the forward direction
	 * of all edge relative orientations is determined by the station
	 * orientation indices (values stored in nodes). The forward direction
	 * is defined by the logic:
	 *
	 * \arg Strictly \b required that: (theFromKey < theIntoKey)
	 * \arg The "From" station is associated with tranform Domain
	 * \arg The "Into" station is associated with tranform Range
	 * \arg Forward transform iterpreted as Into(WithRespectTo)From or
	 *      operationally interpreted as xInto = tranform(xFrom)
	 */
	struct EdgeDir
	{
		//! Domain key for edge transformation interpretations
		StaKey theFromKey{ std::numeric_limits<StaKey>::max() };

		//! Range key for edge transformation interpretations
		StaKey theIntoKey{ std::numeric_limits<StaKey>::max() };

		// Check order interpretation
		enum DirCompare
		{
			  Different
			, Forward
			, Reverse
		};

		//! Vertex key interpreted as edge domain.
		inline
		StaKey const &
		fromKey
			() const
		{
			return theFromKey;
		}

		//! Vertex key interpreted as edge range.
		inline
		StaKey const &
		intoKey
			() const
		{
			return theIntoKey;
		}

		//! True if this edge is potentially valid (keys are different)
		inline
		bool
		isValid
			() const
		{
			return
				(  (theFromKey < std::numeric_limits<StaKey>::max())
				&& (theIntoKey < std::numeric_limits<StaKey>::max())
				&& (theFromKey != theIntoKey)
				);
		}

		//! Compare this direction iterpretation with that of testDir.
		inline
		DirCompare
		compareTo
			( EdgeDir const & testDir
			) const
		{
			DirCompare relation{ Different };
			if (isValid())
			{
				if  (  (testDir.fromKey() == fromKey())
					&& (testDir.intoKey() == intoKey())
					)
				{
					relation = Forward;
				}
				else
				if  (  (testDir.intoKey() == fromKey())
					&& (testDir.fromKey() == intoKey())
					)
				{
					relation = Reverse;
				}
			}
			return relation;
		}

		//! True if this edge is in the "forward" direction (FromKey < IntoKey)
		inline
		bool
		isForward
			() const
		{
			return (theFromKey < theIntoKey);
		}

		//! True if this edge is in the "reverse" direction (IntoKey < FromKey)
		inline
		bool
		isReverse
			() const
		{
			return (theIntoKey < theFromKey);
		}

	}; // EdgeDir

	/*! \brief Base class for edges compatible with Geometry graph structures.
	 *
	 * Derived classes should override the xform() method to provide
	 * a transformation exprssing the geometric relationship between
	 * stations identified with theFromStaKey and theIntoStaKey values.
	 */
	struct EdgeBase : public graaf::weighted_edge<double>
	{
		EdgeDir theEdgeDir{};

		//! Value ctor.
		inline
		explicit
		EdgeBase
			( EdgeDir const & edgeDir
			)
			: theEdgeDir{ edgeDir }
		{ }

		//! Construct with null/invalid member values.
		inline
		EdgeBase
			() = default;

		//! No-op dtor.
		virtual
		inline
		~EdgeBase
			() = default;

		//! Edge direction information
		inline
		EdgeDir const &
		edgeDir
			() const
		{
			return theEdgeDir;
		}

		//! Edge weight (is transformation fit error - theFitErr)
		[[nodiscard]]
		inline
		virtual
		double
		get_weight
			() const noexcept override
		{
			return 1.;
		}

		//! Sort in order of increasing edge weight (transformation error)
		inline
		bool
		operator<
			( EdgeBase const & other
			) const noexcept
		{
			return (this->get_weight() < other.get_weight());
		}

		//! True if this and other have different edge weights
		inline
		bool
		operator!=
			( EdgeBase const & other
			) const noexcept
		{
			return ((other < (*this)) || ((*this) < other));
		}

		//! Transformation (Hi-Ndx w.r.t. Lo-Ndx)
		virtual
		inline
		rigibra::Transform const &
		xform
			() const = 0;

	}; // EdgeBase

	/*! \brief Rigid body orientation betweem two station frames.
	 *
	 * NOTE: the forward direction of the transformation is associated
	 * with EdgeBase keys (theFromStaKey, theIntoStaKey).
	 *
	 * The inverse() function provides the transformation for an
	 * edge being traversed in the other direction.
	 */
	struct EdgeOri : public EdgeBase
	{
		rigibra::Transform theXform{ rigibra::null<rigibra::Transform>() };
		double theFitErr{ engabra::g3::null<double>() };

		//! Value ctor.
		inline
		explicit
		EdgeOri
			( EdgeDir const & edgeDir
			, rigibra::Transform const & xform
			, double const & fitErr
			)
			: EdgeBase(edgeDir)
			, theXform{ xform }
			, theFitErr{ fitErr }
		{ }

		//! Construct with null/invalid member values.
		inline
		EdgeOri
			() = default;

		//! No-op dtor.
		virtual
		inline
		~EdgeOri
			() = default;

		//! Transformation (Hi-Ndx w.r.t. Lo-Ndx)
		virtual
		inline
		rigibra::Transform const &
		xform
			() const
		{
			return theXform;
		}

		//! An instance associated with edge in reverse direction.
		inline
		EdgeOri
		edgeReversed
			() const
		{
			// invert *BOTH* interpretation keys and transform details
			EdgeDir revDir
				{ .theFromKey = theEdgeDir.intoKey()
				, .theIntoKey = theEdgeDir.fromKey()
				};
			return EdgeOri(revDir, rigibra::inverse(xform()), theFitErr);
		}

	}; // EdgeOri

	/*
	//! Robust transformation computed from collection of transforms
	EdgeOri
	edgeOriMedianFit
		( std::vector<rigibra::Transform> const & xHiWrtLos
		);
	*/


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
		graaf::undirected_graph<StaFrame, EdgeOri> theGraph{};

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
		addEdge
			( LoHiKeyPair const & staKeyLoHi
			, EdgeOri const & edgeOri
			);

		//! Edges forming a minimum path
		std::vector<graaf::edge_id_t>
		spanningEdgeOris
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
		std::vector<rigibra::Transform>
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
	/*
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
	*/

	//! Put geo::infoString() to stream.
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


#endif // OriNet_network_INCL_

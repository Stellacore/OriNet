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

Example:
\snippet test_network.cpp DoxyExample01

*/

#include <Engabra>
#include <graaflib/graph.h>
#include <Rigibra>

#include <filesystem>
#include <map>
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
	using StaNdx = std::size_t;

	/*! \brief Relative orientations between stations (.first < .second order)
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

	 * \arg Strictly \b required that: (LoHiPair.first < LoHiPair.second)
	 * \arg The "From" station is associated with LoHiPair.first
	 * \arg The "Into" station is associated with LoHiPair.second
	 * \arg Forward transform iterpreted as From(WithRespectTo)Into
	 */
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
	struct EdgeOri : public graaf::weighted_edge<double>
	{
		rigibra::Transform theLoHiXform{ rigibra::null<rigibra::Transform>() };
		double theFitErr{ engabra::g3::null<double>() };

		//! Value ctor.
		inline
		explicit
		EdgeOri
			( rigibra::Transform const & lohiXform
			, double const & fitErr
			)
			: theLoHiXform{ lohiXform }
			, theFitErr{ fitErr }
		{ }

		//! Construct with null/invalid member values.
		inline
		EdgeOri
			() = default;

		//! No-op dtor.
		inline
		~EdgeOri
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
			( EdgeOri const & other
			) const noexcept
		{
			return (this->get_weight() < other.get_weight());
		}

		//! True if this and other have different edge weights
		inline
		bool
		operator!=
			( EdgeOri const & other
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
		EdgeOri
		inverse
			() const
		{
			return EdgeOri(rigibra::inverse(theLoHiXform), theFitErr);
		}

	}; // EdgeOri


	/*! \brief Representation of the geometry of a rigid body network.
	 *
	 * Uses a graph data structure to store StaFrame instances as nodes
	 * and rigid body transformations as edge relationships between them.
	 *
	 */
	class Geometry
	{
		//! Lookup map: station data index from graph vertex index 
		std::map<StaNdx, VertId> theVertIdFromStaNdx{};

		//! Graph data structure for storing/processing network relationships
		graaf::undirected_graph<StaFrame, EdgeOri> theGraph{};

		//! Robust transformation computed from collection of transforms
		static
		EdgeOri
		edgeOriMedianFit
			( std::vector<rigibra::Transform> const & xHiWrtLos
			);

		//! Check if staNdx already in graph, if not, then add vertex
		// Geometry::
		void
		ensureStaFrameExists
			( StaNdx const & staNdx
			);

		//! Graaf vertex ID value for station index
		// Geometry::
		VertId
		vertIdForStaNdx
			( StaNdx const & staNdx
			) const;

		//! External station index for Graaf vertex ID value
		// Geometry::
		StaNdx
		staNdxForVertId
			( VertId const & vertId
			) const;

	public:

		//! Insert transformation edge into graph
		// Geometry::
		void
		addEdge
			( LoHiPair const & staNdxLoHi
			, EdgeOri const & edgeOri
			);

		//! Edges forming a minimum path
		// Geometry::
		std::vector<graaf::edge_id_t>
		spanningEdgeOris
			() const;

		/*! \brief Create an instance populated according to edge list
		 *
		 * E.g. calling this function with result of spanningEdgeOris()
		 * will return a new network that minimally spans this original
		 * instance.
		 */
		 // *  snippet TODO
		// Geometry::
		Geometry
		networkTree
			( std::vector<graaf::edge_id_t> const eIds
			) const;

		/*! \brief Transformations computed by propagation through network
		 *
		 * Note that later computed transformations overwrite earlier ones.
		 * In general, this is method is probably most useful if run
		 * on a network that represents a minimum spanning tree.
		 */
		std::vector<rigibra::Transform>
		propagateTransforms
			( StaNdx const & staNdx0
			, rigibra::Transform const & staXform0
			) const;

		//! \brief Save graph information to graphviz '.dot' graphic file.
		// Geometry::
		void
		saveNetworkGraphic
			( std::filesystem::path const & dotPath
			) const;

	}; // Geometry

} // [network]


} // [orinet]


#endif // OriNet_network_INCL_

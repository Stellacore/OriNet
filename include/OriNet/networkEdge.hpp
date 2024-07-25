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


#ifndef OriNet_network_Edge_INCL_
#define OriNet_network_Edge_INCL_

/*! \file
\brief Contains Classes and functions for network graph edge management.

Example:
\snippet test_network.cpp DoxyExample01

*/


#include "networkVert.hpp"
#include "stat.hpp"

#include <Engabra>
#include <graaflib/edge.h>
#include <Rigibra>

#include <iostream>
#include <sstream>
#include <string>


namespace orinet
{
namespace network
{

	//! Edge type - used in graph structure library
	using EdgeId = graaf::edge_id_t;
	// using EdgeId = std::pair<std::size_t, std::size_t>;


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
		StaKey theFromKey{ sNullKey };

		//! Range key for edge transformation interpretations
		StaKey theIntoKey{ sNullKey };

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
				(  (orinet::network::isValid(theFromKey))
				&& (orinet::network::isValid(theIntoKey))
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

		//! Swap domain and range node keys
		inline
		EdgeDir
		reverseEdgeDir
			() const
		{
			// invert interpretation of keys
			EdgeDir revDir
				{ .theFromKey = intoKey()
				, .theIntoKey = fromKey()
				};
			return revDir;
		}

		//! Descriptive information about this instance
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			if (isValid())
			{
				oss
					<< "from: " << theFromKey
					<< ' '
					<< "into: " << theIntoKey
					;
			}
			else
			{
				std::cout << "<null>";
			}
			return oss.str();
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
		EdgeDir
		edgeDir
			() const
		{
			return theEdgeDir;
		}

		//! Starting point of directed edge
		inline
		StaKey
		fromKey
			() const
		{
			return theEdgeDir.fromKey();
		}

		//! Ending point of directed edge
		inline
		StaKey
		intoKey
			() const
		{
			return theEdgeDir.intoKey();
		}

		//! True if this instance has valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theEdgeDir.isValid()
				&& rigibra::isValid(xform())
				&& engabra::g3::isValid(get_weight())
				);
		}

		//! Edge weight (null value)
		[[nodiscard]]
		inline
		virtual
		double
		get_weight
			() const noexcept override
		{
			constexpr double wgt{ engabra::g3::null<double>() };
			return wgt;
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
		rigibra::Transform
		xform
			() const
		{
			using namespace rigibra;
			static Transform const xfm{ null<Transform>() };
			return xfm;
		}

		//! An instance associated with edge in reverse direction.
		virtual
		inline
		std::shared_ptr<EdgeBase>
		reversedInstance
			() const
		{
			return std::make_shared<EdgeBase>(theEdgeDir.reverseEdgeDir());
		}

		//! Descriptive information about this instance
		virtual
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const
		{
			std::ostringstream oss;
			if (! title.empty())
			{
				oss << title << ' ';
			}
			if (isValid())
			{
				oss
					<< theEdgeDir.infoString()
					<< ' '
					<< "xform: " << xform()
					;
			}
			else
			{
				std::cout << "<null>";
			}
			return oss.str();
		}

	}; // EdgeBase

	/*! \brief Rigid body orientation betweem two station frames.
	 *
	 * NOTE: the forward direction of the transformation is associated
	 * with EdgeBase keys (theFromStaKey, theIntoStaKey).
	 *
	 * The reversedInstance() function provides the transformation for an
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

		//! No-op dtor.
		virtual
		inline
		~EdgeOri
			() = default;

		//! True if this instance has valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theEdgeDir.isValid()
				&& engabra::g3::isValid(theFitErr)
				&& rigibra::isValid(xform())
				);
		}

		//! Transformation (Hi-Ndx w.r.t. Lo-Ndx)
		virtual
		inline
		rigibra::Transform
		xform
			() const override
		{
			return theXform;
		}

		//! Edge weight (transformation fit error - theFitErr)
		[[nodiscard]]
		inline
		virtual
		double
		get_weight
			() const noexcept override
		{
			return theFitErr;
		}

		//! An instance associated with edge in reverse direction.
		virtual
		inline
		std::shared_ptr<EdgeBase>
		reversedInstance
			() const override
		{
			return std::make_shared<EdgeOri>
				( edgeDir().reverseEdgeDir()
				, rigibra::inverse(xform())
				, theFitErr  // assume this stays the same
				);
		}

		//! Descriptive information about this instance
		virtual
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const override
		{
			std::ostringstream oss;
			if (isValid())
			{
				oss << EdgeBase::infoString(title);
				oss << ' '
					<< "fitErr: " << get_weight()
					;
			}
			else
			{
				std::cout << "<null>";
			}
			return oss.str();
		}

	}; // EdgeOri


	/*! \brief Robust rigid body transformation tracking betweem two stations.
	 *
	 * NOTE: the forward direction of the transformation is associated
	 * with EdgeBase keys (theFromStaKey, theIntoStaKey).
	 *
	 * The reversedInstance() function provides the transformation for an
	 * edge being traversed in the other direction.
	 */
	struct EdgeRobust : public EdgeBase
	{
		stat::track::Transforms theXformTracker;

		//! Value ctor.
		inline
		explicit
		EdgeRobust
			( EdgeDir const & edgeDir
			, rigibra::Transform const & xform
			, std::size_t const & reserveSize
			)
			: EdgeBase(edgeDir)
			, theXformTracker(reserveSize)
		{
			accumulateXform(xform);
		}

		//! No-op dtor.
		virtual
		inline
		~EdgeRobust
			() = default;

		//! True if this instance has valid data
		inline
		bool
		isValid
			() const
		{
			return
				(  theEdgeDir.isValid()
				&& rigibra::isValid(xform())
				);
		}

		//! Insert xform into transform accumulation tracker's running total
		inline
		void
		accumulateXform
			( rigibra::Transform const & xform
			)
		{
			theXformTracker.insert(xform);
		}

		//! Transformation (Hi-Ndx w.r.t. Lo-Ndx)
		virtual
		inline
		rigibra::Transform
		xform
			() const override
		{
			return theXformTracker.median();
		}

		//! \brief Unity for now - assuming all medians() are similar quality
		// TODO (transformation 50 percentile error) TODO
		[[nodiscard]]
		inline
		virtual
		double
		get_weight
			() const noexcept override
		{
			double weight{ engabra::g3::null<double>() };
			std::size_t const numXforms{ theXformTracker.size() };
			if (0u < numXforms) // 0u shouldn't be possible if edge exists
			{
				if (1u == numXforms)
				{
					// value to use for edges having *NO* available
					// quality estimate.
					constexpr double veryUncertain{ 1024.*1024. };
					weight = veryUncertain;
				}
				else
				{
					constexpr bool normComp{ false };
					weight = theXformTracker.medianErrorEstimate(normComp);
				}
			}
			return weight;
		}

		//! An instance associated with edge in reverse direction.
		virtual
		inline
		std::shared_ptr<EdgeBase>
		reversedInstance
			() const override
		{
			return std::make_shared<EdgeOri>
				( edgeDir().reverseEdgeDir()
				, rigibra::inverse(xform())
				, get_weight()  // assume this stays the same
				);
		}

		//! Descriptive information about this instance
		virtual
		inline
		std::string
		infoString
			( std::string const & title = {}
			) const override
		{
			std::ostringstream oss;
			if (isValid())
			{
				oss << EdgeBase::infoString(title);
				oss << ' '
					<< "trackSize: " << theXformTracker.size()
					;
			}
			else
			{
				std::cout << "<null>";
			}
			return oss.str();
		}

	}; // EdgeRobust


} // [network]

} // [orinet]

namespace
{
	//! Put instance to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::network::EdgeDir const & edgeDir
		)
	{
		ostrm << edgeDir.infoString();
		return ostrm;
	}

	//! Put instance to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::network::EdgeBase const & edge
		)
	{
		ostrm << edge.infoString();
		return ostrm;
	}

	//! Put instance to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::network::EdgeOri const & edge
		)
	{
		ostrm << edge.infoString();
		return ostrm;
	}

	//! Put instance to stream.
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::network::EdgeRobust const & edge
		)
	{
		ostrm << edge.infoString();
		return ostrm;
	}

} //[anon]

#endif // OriNet_network_Edge_INCL_

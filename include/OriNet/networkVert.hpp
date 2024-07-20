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


#ifndef OriNet_network_Vert_INCL_
#define OriNet_network_Vert_INCL_

/*! \file
\brief Contains Classes and functions for network graph vertex management.

Example:
\snippet test_network.cpp DoxyExample01

*/


#include <graaflib/graph.h>

#include <limits>


namespace orinet
{
namespace network
{

	//! Vertex type - used in graph structure library
	using VertId = graaf::vertex_id_t;

	//! Station orientations referenced by index (e.g. to external collection)
	using StaKey = std::size_t;

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

} // [network]

} // [orinet]


#endif // OriNet_network_Vert_INCL_

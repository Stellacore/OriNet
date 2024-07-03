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


#ifndef OriNet_align_INCL_
#define OriNet_align_INCL_

/*! \file
\brief Functions for computing alignment for rigid body attitude.

Example:
\snippet test_alignPair.cpp DoxyExample01

*/


#include <Engabra>
#include <Rigibra>


namespace orinet
{
	//! \brief Two arbitrary but not (anti)parallel unitary directions
	using DirPair = std::pair<engabra::g3::Vector, engabra::g3::Vector>;

	/*! \brief Attitude that 'best' fits body wrt reference frame rotation.
	 *
	 * Attitude that 'best' transforms the refDirPair into bodDirPair. The
	 * Transformation assures that the plane defined by the reference pair
	 * is transformed into the plane defined by the body pair. The "best"
	 * part is that that the mean reference direction is transformed into
	 * the mean body direction (but individual directions will, in general,
	 * not match exactly).
	 *
	 * Ref: theory/alignDirPairs.lyx
	 */
	inline
	rigibra::Attitude
	alignDirPair
		( DirPair const & refDirPair
			//!< Direction pair in reference frame
		, DirPair const & bodDirPair
			//!< Direction pair in body frame
		)
	{
		using namespace rigibra;
		Attitude att{ null<Attitude>() };
		return att;
	}


} // [orinet]


namespace
{

	//! Put pair of direction vectors to stream
	inline
	std::ostream &
	operator<<
		( std::ostream & ostrm
		, orinet::DirPair const & dirPair
		)
	{
		ostrm << dirPair.first << ' ' << dirPair.second;
		return ostrm;
	}

} // [anon]


#endif // OriNet_align_INCL_

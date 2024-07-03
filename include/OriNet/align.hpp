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

#include <limits>


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

		// access individual directions
		using namespace engabra::g3;
		Vector const & a0 = refDirPair.first;
		Vector const & b0 = refDirPair.second;
		Vector const & a1 = bodDirPair.first;
		Vector const & b1 = bodDirPair.second;

		// determine bivector angles defined by each direction pair
		BiVector const biv0{ (a0 * b0).theBiv };
		BiVector const biv1{ (a1 * b1).theBiv };
		constexpr double magTol{ std::numeric_limits<double>::epsilon() };
		double const mag0{ magnitude(biv0) };
		double const mag1{ magnitude(biv1) };
		if ((magTol < mag0) && (magTol < mag1))
		{
			BiVector const thetaDir0{ (1./mag0) * biv0 };
			BiVector const thetaDir1{ (1./mag1) * biv1 };
			// determine first rotation step - align planes
			Spinor const sqP{ -1 * (thetaDir1 * thetaDir0) };
			Spinor const spinP{ sqrtG2(sqP) };

			// rotate reference directions into plane of body directions
			Vector const at{ (spinP * a0 * reverse(spinP)).theVec };
			Vector const bt{ (spinP * b0 * reverse(spinP)).theVec };

			// get mean directions from temp and body direction pairs
			Vector const mt{ direction(.5 * (at + bt)) };
			Vector const m1{ direction(.5 * (a1 + b1)) };

			/*
			std::cout << "a0: " << a0 << '\n';
			std::cout << "b0: " << b0 << '\n';
			std::cout << "thetaDir0: " << thetaDir0 << '\n';
			std::cout << "thetaDir1: " << thetaDir1 << '\n';
			std::cout << "at: " << at << '\n';
			std::cout << "bt: " << bt << '\n';
			std::cout << "mt: " << mt << '\n';
			std::cout << "m1: " << m1 << '\n';
			*/

			// determine second rotation step - align mean directions
			Spinor const omega{ .5 * logG2(m1 * mt, thetaDir1) };
			Spinor const spinQ{ exp(omega) };

			// composite the sequential spinors into result
			Spinor const spinR{ spinQ * spinP };

			// return net rotation as attitude
			att = rigibra::Attitude(spinR);
		}

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

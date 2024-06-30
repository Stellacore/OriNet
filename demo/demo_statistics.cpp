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
\brief Demonstration relationship of hexad maxMag to sigma{Loc,Ang} values.
*/


#include "OriNet/compare.hpp"
#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <fstream>
#include <iostream>
#include <vector>


namespace
{
	//! A collection of values spanning a range
	inline
	std::vector<double>
	samples
		( std::size_t const & numSamps
			//!< Expecting 2 or greater
		, std::pair<double, double> const & minmax
			//!< Minimum/Maximum sample value (included)
		)
	{
		std::vector<double> samps;
		double const & min = minmax.first;
		double const & max = minmax.second;
		if ((min < max) && (1u < numSamps))
		{
			samps.reserve(numSamps);
			double const span{ max - min };
			double const delta{ span / static_cast<double>(numSamps) };
			for (double xx{min} ; (! (max < xx)) ; xx += delta)
			{
				samps.emplace_back(xx);
			}
		}
		return samps;
	}

}


/*! \brief Monte-Carlo simulation of hexad 'maxMag' statistics.
 *
 * Program generates output of that should be useful for fitting a
 * heuristic model to predict hexad 'maxMag' values associated with
 * the random::perturbedTransform() generated transformations.
 */
int
main
	( int argc
	, char * argv[]
	)
{
	if (! (2u == argc))
	{
		std::cerr << "Usage: <progname> outfile\n";
		return 1;
	}
	std::ofstream ofs(argv[1]);

std::cout << "Hi from " << __FILE__ << '\n';


	constexpr std::size_t numMea{ 19u };
	constexpr std::size_t numErr{ 0u }; // don't want blunders here

	/*
	std::vector<double> const sigmaLocs
		{ samples( 16u, std::make_pair(0.,  4./16.)) };
	std::vector<double> const sigmaAngs
		{ samples( 16u, std::make_pair(0., 4./128.)) };
	*/
	std::vector<double> const sigmaLocs
		{ samples( 2u, std::make_pair(0., 1./1024.)) };
	std::vector<double> const sigmaAngs
		{ samples( 2u, std::make_pair(0., 3.1415)) };

	using engabra::g3::pi;
	std::pair<double, double> const locMinMax{ -10., 10. };
	std::pair<double, double> const angMinMax{ -pi, pi };
	rigibra::Transform const xformBase
		{ orinet::rand::uniformTransform(locMinMax, angMinMax) };

	for (double const & sigmaLoc : sigmaLocs)
	{
		for (double const & sigmaAng : sigmaAngs)
		{
			std::vector<rigibra::Transform> const xformSamps
				{ orinet::rand::noisyTransforms
					(xformBase, numMea, numErr, sigmaLoc, sigmaAng)
				};

std::cout << "\n\n";
std::cout << "xformBase: " << xformBase << '\n';
for (rigibra::Transform const & xformSamp : xformSamps)
{
	std::cout << "xformSamp: " << xformSamp << '\n';
}

			for (rigibra::Transform const & xformSamp : xformSamps)
			{
				double const maxMag
					{ orinet::maxMagResultDifference(xformSamp, xformBase) };

				using namespace engabra::g3::io;
				ofs
					<< ' ' << fixed(sigmaLoc)
					<< ' ' << fixed(sigmaAng)
					<< ' ' << fixed(maxMag)
					<< '\n';
			}
			ofs << "\n\n";

		}
	}
}


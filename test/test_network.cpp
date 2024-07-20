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
\brief Unit tests (and example) code for OriNet::CN
*/


#include "OriNet/network.hpp"

#include "OriNet/random.hpp"

#include <Engabra>
#include <Rigibra>

#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>


namespace
{
	//! Examples for documentation
	void
	test0
		( std::ostream & oss
		)
	{
		// [DoxyExample01]

		// [DoxyExampleSim]

		constexpr std::pair<double, double> locMinMax{ -50., 100. };
		constexpr std::pair<double, double> angMinMax{ -3.14, +3.14 };

		using rigibra::Transform;
		using orinet::random::uniformTransform;
		std::vector<Transform> const expStas
			{ uniformTransform(locMinMax, angMinMax) // 0
			, uniformTransform(locMinMax, angMinMax)
			, uniformTransform(locMinMax, angMinMax)
			, uniformTransform(locMinMax, angMinMax)
			, uniformTransform(locMinMax, angMinMax)
			, uniformTransform(locMinMax, angMinMax) // 5
			};

		// [DoxyExampleSim]

		// [DoxyExampleCreate]

		// main network (will have redundant edge relative orientations)
		orinet::network::Geometry netGeo;

		// [DoxyExampleCreate]

		// [DoxyExampleEdges]

		// relative orientation between stations - IntoWrtFrom
		std::function<Transform(Transform const &, Transform const &)>
			const ro
			{ []
				( Transform const & xFromWrtRef
				, Transform const & xIntoWrtRef
				)
				{
					Transform const xRefWrtFrom
						{ rigibra::inverse(xFromWrtRef) };
					return (xIntoWrtRef * xRefWrtFrom);
				}
			};

		// specify a few arbitrary relative orientations to define network
		using orinet::network::LoHiKeyPair;
		std::vector<LoHiKeyPair> const edgeLoHis
			{ {0u, 1u}, {0u, 2u}, {0u, 4u}
			, {1u, 2u}, {1u, 4u}
			, {2u, 3u}, {2u, 5u}
			, {3u, 4u}
			, {4u, 5u}
			};
		double const fitErr{ .001 }; // assume all RelOri of equal quality
		for (LoHiKeyPair const & edgeLoHi : edgeLoHis)
		{
			using namespace orinet::network;
			StaKey const & fromKey = edgeLoHi.first;
			StaKey const & intoKey = edgeLoHi.second;
			std::shared_ptr<EdgeOri> const ptEdge
				{ std::make_shared<EdgeOri>
					( EdgeDir{ fromKey, intoKey }
					, ro(expStas[fromKey], expStas[intoKey])
					, fitErr
					)
				};
			netGeo.addEdge(edgeLoHi, ptEdge);
		}

		// [DoxyExampleEdges]

		// [DoxyExampleThin]

		// compute minimum path spanning tree
		// (along minimum relative orientation transform errors)
		using orinet::network::EdgeId;
		std::vector<EdgeId> const eIds{ netGeo.spanningEdgeBases() };

		orinet::network::Geometry const mstGeo{ netGeo.networkTree(eIds) };

		// [DoxyExampleThin]

		// [DoxyExamplePropagate]

		// propagate relative orientations into station orientations
		constexpr orinet::network::StaKey holdStaKey{ 3u };
		Transform const holdStaOri{ expStas[holdStaKey] };
		std::vector<Transform> const gotStas
			{ mstGeo.propagateTransforms(holdStaKey, holdStaOri) };

		// [DoxyExamplePropagate]

		// compare computed station orientations with expected ones
		if (! (gotStas.size() == expStas.size()))
		{
			oss << "Failure of gotStas size test\n";
			oss << "exp: " << expStas.size() << '\n';
			oss << "got: " << gotStas.size() << '\n';
		}
		else
		{
			std::size_t const numSta{ expStas.size() };
			for (std::size_t nn{0u} ; nn < numSta ; ++nn)
			{
				Transform const & gotSta = gotStas[nn];
				Transform const & expSta = expStas[nn];
				// use nearly exact comparison since no noise in sim data
				// adjust tolerance to range of station values
				double const locMag
					{ std::hypot(locMinMax.first, locMinMax.second) };
				double const tol
					{ locMag * std::numeric_limits<double>::epsilon() };
				if (! nearlyEquals(gotSta, expSta, tol))
				{
					oss << "Failure of gotSta data test\n";
					oss << " nn: " << nn << '\n';
					oss << "exp: " << expSta << '\n';
					oss << "got: " << gotSta << '\n';
				}
			}
		}

		// [DoxyExample01]
	}

	//! Check StaKey (station id) vs VertId (graph node) distinctions
	void
	test1
		( std::ostream & oss
		)
	{
		std::vector<std::size_t> const staKeys
			{ 1000u
			, 1001u
			, 1002u
			, 1003u
			, 1004u
			};

		using namespace orinet::network;
		Geometry netGeo;
		constexpr double fitErr{ 1. };
		std::size_t const numSta{ staKeys.size() };
		for (std::size_t fmNdx{0u} ; fmNdx < numSta ; ++fmNdx)
		{
			for (std::size_t toNdx{fmNdx + 1u} ; toNdx < numSta ; ++toNdx)
			{
				StaKey const & fromKey = staKeys[fmNdx];
				StaKey const & intoKey = staKeys[toNdx];
				using namespace rigibra;
				Transform const xIntoWrtFrom{ null<Transform>() };
				std::shared_ptr<EdgeOri> const ptEdge
					{ std::make_shared<EdgeOri>
						(EdgeDir{ fromKey, intoKey }, xIntoWrtFrom, fitErr)
					};
				netGeo.addEdge(LoHiKeyPair{ fromKey, intoKey }, ptEdge);
			}
		}

		// get descriptive information
		std::string const info{ netGeo.infoStringContents("netGeo") };
		// std::cout << info << '\n';

		// check that keys showup in reported info
		std::istringstream iss(info);
		std::set<StaKey> gotVertKeys;
		std::map<StaKey, std::size_t> gotEdgeKeyCounts;
		std::string line;
		std::string tmpLabel;
		StaKey tmpStaKey;
		while (iss.good())
		{
			line.clear();
			std::getline(iss, line);
			if (std::string::npos != line.find("VertKey"))
			{
				std::istringstream irec(line);
				irec >> tmpLabel >> tmpStaKey;
				gotVertKeys.insert(tmpStaKey);
			}
			else
			if (std::string::npos != line.find("EdgeKey"))
			{
				std::istringstream irec(line);
				// check from key
				irec >> tmpLabel >> tmpStaKey;
				++gotEdgeKeyCounts[tmpStaKey];
				// check into key
				irec >> tmpStaKey;
				++gotEdgeKeyCounts[tmpStaKey];
			}
		}

		// check number of unique vertices
		if (! (gotVertKeys.size() == staKeys.size()))
		{
			oss << "Failure of infoStringContents vertex count test\n";
			oss << "exp: " << staKeys.size() << '\n';
			oss << "got: " << gotVertKeys.size() << '\n';
		}

		// check how many time each vertex occurs in an edge
		for (std::map<StaKey, std::size_t>::value_type
			const & gotEdgeKeyCount : gotEdgeKeyCounts)
		{
			constexpr std::size_t expCount{ 4u }; // true for all key values
			StaKey const & gotKey = gotEdgeKeyCount.first;
			std::size_t const & gotCount = gotEdgeKeyCount.second;
			if (! (gotCount == expCount))
			{
				oss << "Failure of infoStringContents edge count test\n";
				oss << "key: " << gotKey << '\n';
				oss << "exp: " << expCount << '\n';
				oss << "got: " << gotCount << '\n';
			}
		}
	}
}

//! Check behavior of NS
int
main
	()
{
	int status{ 1 };
	std::stringstream oss;

	test0(oss);
	test1(oss);

	if (oss.str().empty()) // Only pass if no errors were encountered
	{
		status = 0;
	}
	else
	{
		// else report error messages
		std::cerr << "### FAILURE in test file: " << __FILE__ << std::endl;
		std::cerr << oss.str();
	}
	return status;
}


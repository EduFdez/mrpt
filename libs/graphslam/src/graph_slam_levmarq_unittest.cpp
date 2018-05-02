/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "graph_slam_levmarq_test_common.h"

#include <gtest/gtest.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::graphs;
using namespace mrpt::math;
using namespace std;

template <class my_graph_t>
class GraphSlamLevMarqTester : public GraphSlamLevMarqTest<my_graph_t>,
							   public ::testing::Test
{
   protected:
	virtual void SetUp() {}
	virtual void TearDown() {}
	void test_ring_path()
	{
		// This is the initial input graph (make a copy for later use):
		my_graph_t graph;
		GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);

		const my_graph_t graph_initial = graph;

		// ----------------------------
		//  Run graph slam:
		// ----------------------------
		mrpt::system::TParametersDouble params;
		// params["verbose"]  = 1;
		// params["profiler"] = 1;
		params["max_iterations"] = 1000;

		graphslam::TResultInfoSpaLevMarq levmarq_info;

		graphslam::optimize_graph_spa_levmarq(
			graph, levmarq_info, nullptr, params);

		// Do some basic checks on the results:
		EXPECT_GE(levmarq_info.num_iters, 10U);
		EXPECT_LE(levmarq_info.final_total_sq_error, 1e-2);

	}  // end test_ring_path

	void test_graph_bin_serialization()
	{
		my_graph_t graph;
		GraphSlamLevMarqTest<my_graph_t>::create_ring_path(graph);

		// binary dump:
		mrpt::io::CMemoryStream mem;
		auto arch = mrpt::serialization::archiveFrom(mem);
		arch << graph;

		{
			my_graph_t read_graph;
			mem.Seek(0);
			arch >> read_graph;

			EXPECT_EQ(read_graph.edges.size(), graph.edges.size());
			EXPECT_EQ(read_graph.nodes.size(), graph.nodes.size());

			// Also check that the edge values are OK:
			typename my_graph_t::const_iterator it1, it2;
			for (it1 = read_graph.edges.begin(), it2 = graph.edges.begin();
				 it1 != read_graph.edges.end(); ++it1, ++it2)
			{
				EXPECT_EQ(it1->first, it2->first);
				EXPECT_NEAR(
					0, (it1->second.getPoseMean().getAsVectorVal() -
						it2->second.getPoseMean().getAsVectorVal())
						   .array()
						   .abs()
						   .sum(),
					1e-9);
			}
		}
	}
};

using GraphSlamLevMarqTester2D = GraphSlamLevMarqTester<CNetworkOfPoses2D>;
using GraphSlamLevMarqTester3D = GraphSlamLevMarqTester<CNetworkOfPoses3D>;

TEST_F(GraphSlamLevMarqTester2D, OptimizeSampleRingPath)
{
	for (int seed = 1; seed < 5; seed++)
	{
		getRandomGenerator().randomize(seed);
		test_ring_path();
	}
}
TEST_F(GraphSlamLevMarqTester2D, BinarySerialization)
{
	getRandomGenerator().randomize(123);
	test_graph_bin_serialization();
}

TEST_F(GraphSlamLevMarqTester3D, OptimizeSampleRingPath)
{
	for (int seed = 1; seed < 5; seed++)
	{
		getRandomGenerator().randomize(seed);
		test_ring_path();
	}
}
TEST_F(GraphSlamLevMarqTester3D, BinarySerialization)
{
	getRandomGenerator().randomize(123);
	test_graph_bin_serialization();
}

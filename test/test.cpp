#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <sys/stat.h>

#include <rondo/graph.hpp>

class GraphTest : public ::testing::Test {
 protected:
  rondo::graph g_undirected = rondo::graph(false);
  rondo::graph g_directed   = rondo::graph(true);

  GraphTest() {
    EnsureDirExists("./tmp");
  }

  void SetUp() override {
    g_undirected.clear();
    g_directed.clear();
  }

  void TearDown() override {
  }

  void EnsureDirExists(const std::string &path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
      int ret = mkdir(path.c_str(), 0777);
      if (ret != 0) {
        throw std::runtime_error("unable to create directory: " + path);
      }
    } else if (!(info.st_mode & S_IFDIR)) {
      throw std::runtime_error("path exists but not a directory: " + path);
    }
}
};

TEST_F(GraphTest, Init) {
  EXPECT_EQ(g_undirected.size(), 0);
  EXPECT_TRUE(g_undirected.empty());
  EXPECT_EQ(g_directed.size(), 0);
  EXPECT_TRUE(g_directed.empty());
}

TEST_F(GraphTest, AddVertex) {
  auto v1 = g_undirected.add_vertex();
  EXPECT_EQ(g_undirected.size(), 1);
  EXPECT_FALSE(g_undirected.empty());

  auto v2 = g_undirected.add_vertex();
  EXPECT_EQ(g_undirected.size(), 2);

  auto v3 = g_directed.add_vertex();
  EXPECT_EQ(g_directed.size(), 1);
}

TEST_F(GraphTest, AddVertexWithProperties) {
  rondo::graph::property p("label1", "color1");
  auto v1   = g_undirected.add_vertex(p);
  auto result = g_undirected.get_property(v1);
  EXPECT_EQ(result.value().label, "label1");
  EXPECT_EQ(result.value().color, "color1");
}

TEST_F(GraphTest, AddEdge) {
  auto v1 = g_undirected.add_vertex();
  auto v2 = g_undirected.add_vertex();
  g_undirected.add_edge(v1, v2);

  EXPECT_TRUE(g_undirected.has_edge(v1, v2));
  EXPECT_TRUE(g_undirected.has_edge(v2, v1)); // Check for undirected back edge

  g_directed.add_edge(v1, v2);
  EXPECT_TRUE(g_directed.has_edge(v1, v2));
  EXPECT_FALSE(g_directed.has_edge(v2, v1));  // Should not exist for directed graph
}

TEST_F(GraphTest, RemoveEdge) {
  auto v1 = g_undirected.add_vertex();
  auto v2 = g_undirected.add_vertex();
  g_undirected.add_edge(v1, v2);
  g_undirected.remove_edge(v1, v2);

  EXPECT_FALSE(g_undirected.has_edge(v1, v2));
  EXPECT_FALSE(g_undirected.has_edge(v2, v1)); // Check removal of undirected back edge
}

TEST_F(GraphTest, GetVertices) {
  auto p1 = rondo::graph::property("label1", "color1");
  auto p2 = rondo::graph::property("label2", "color2");
  auto v1 = g_undirected.add_vertex(p1);
  auto v2 = g_undirected.add_vertex(p2);

  auto vertices = g_undirected.get_vertices();
  EXPECT_EQ(vertices.size(), 2);
  EXPECT_EQ(vertices[v1].label, p1.label);
  EXPECT_EQ(vertices[v1].color, p1.color);
  EXPECT_EQ(vertices[v2].label, p2.label);
  EXPECT_EQ(vertices[v2].color, p2.color);
}

TEST_F(GraphTest, GetEdges) {
  auto v1 = g_undirected.add_vertex();
  auto v2 = g_undirected.add_vertex();
  g_undirected.add_edge(v1, v2);

  auto edges = g_undirected.get_edges(v1);
  EXPECT_EQ(edges.size(), 1);

  auto edge_it = edges.begin();
  EXPECT_EQ(edge_it->first, v1);
  EXPECT_EQ(edge_it->second, v2);
}

TEST_F(GraphTest, GetWeight) {
  auto v1 = g_undirected.add_vertex();
  auto v2 = g_undirected.add_vertex();
  g_undirected.add_edge(v1, v2, 2.5);

  auto result = g_undirected.get_weight(v1, v2);
  EXPECT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 2.5);
}

TEST_F(GraphTest, SetProperty) {
  auto v1 = g_undirected.add_vertex();
  rondo::graph::property p("new_label", "new_color");
  g_undirected.set_property(v1, p);

  auto result = g_undirected.get_property(v1);
  EXPECT_EQ(result.value().label, "new_label");
  EXPECT_EQ(result.value().color, "new_color");
}

TEST_F(GraphTest, ToJson) {
  auto v1 = g_undirected.add_vertex(rondo::graph::property("node1", "red"));
  auto v2 = g_undirected.add_vertex(rondo::graph::property("node2", "blue"));
  g_undirected.add_edge(v1, v2, 3.5);

  auto json = g_undirected.to_json();

  EXPECT_EQ(json["directed"], false);
  EXPECT_EQ(json["vertices"].size(), 2);
  EXPECT_EQ(json["edges"].size(), 2); // Since it's undirected, there should be two edges (v1-v2 and v2-v1)

  // ... more checks if needed
}

TEST_F(GraphTest, FromJson) {
  std::string json_content = R"(
  {
    "directed": false,
    "vertices": [
        {"id": 1, "label": "node1", "color": "red"},
        {"id": 2, "label": "node2", "color": "blue"}
    ],
    "edges": [
        {"from": 1, "to": 2, "weight": 3.5}
    ]
  })";

  // Write JSON to a temporary file and read it into the graph
  std::string path = "./tmp/graph.json";
  std::ofstream out(path);
  out << json_content;
  out.close();

  // Load graph from JSON file
  g_undirected.from_json(path);

  // Perform checks to ensure the graph loaded correctly
  EXPECT_EQ(g_undirected.size(), 2);
  EXPECT_TRUE(g_undirected.has_edge(1, 2));
  EXPECT_DOUBLE_EQ(g_undirected.get_weight(1, 2).value(), 3.5);
}

TEST_F(GraphTest, DfsTraversal) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2);
  g_directed.add_edge(v1, v3);
  g_directed.add_edge(v2, v4);

  std::vector<rondo::graph::vertex> visited_order;
  g_directed.dfs(v1, [&visited_order](const rondo::graph::vertex &v) {
    visited_order.push_back(v);
  });

  std::vector<rondo::graph::vertex> expected_order = {v1, v2, v4, v3};
  EXPECT_EQ(visited_order, expected_order);
}

TEST_F(GraphTest, BfsTraversal) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2);
  g_directed.add_edge(v1, v3);
  g_directed.add_edge(v2, v4);

  std::vector<rondo::graph::vertex> visited_order;
  g_directed.bfs(v1, [&visited_order](const rondo::graph::vertex &v) {
    visited_order.push_back(v);
  });

  std::vector<rondo::graph::vertex> expected_order = {v1, v2, v3, v4};
  EXPECT_EQ(visited_order, expected_order);
}

TEST_F(GraphTest, DijkstraShortestPath) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2, rondo::graph::weight(1));
  g_directed.add_edge(v2, v3, rondo::graph::weight(1));
  g_directed.add_edge(v3, v4, rondo::graph::weight(1));
  g_directed.add_edge(v1, v4, rondo::graph::weight(10));

  auto result = g_directed.dijkstra(v1);

  std::vector<rondo::graph::vertex> path_to_v4 = rondo::graph::path_to(result, v4);
  std::vector<rondo::graph::vertex> expected_path = {v1, v2, v3, v4};
  EXPECT_EQ(path_to_v4, expected_path);
  EXPECT_DOUBLE_EQ(result[v4].first, 3);
}

TEST_F(GraphTest, BellmanFordShortestPath) {
  // Create a directed graph
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();

  // Add edges with weights
  g_directed.add_edge(v0, v1, rondo::graph::weight(6));
  g_directed.add_edge(v0, v2, rondo::graph::weight(7));
  g_directed.add_edge(v1, v2, rondo::graph::weight(8));
  g_directed.add_edge(v1, v3, rondo::graph::weight(5));
  g_directed.add_edge(v1, v4, rondo::graph::weight(-4));
  g_directed.add_edge(v2, v3, rondo::graph::weight(-3));
  g_directed.add_edge(v2, v4, rondo::graph::weight(9));
  g_directed.add_edge(v3, v1, rondo::graph::weight(-2));
  g_directed.add_edge(v4, v0, rondo::graph::weight(2));
  g_directed.add_edge(v4, v3, rondo::graph::weight(7));

  // Compute shortest paths from v0 using Bellman-Ford
  auto result = g_directed.bellman_ford(v0);

  // Check distances
  EXPECT_DOUBLE_EQ(result[v0].first, 0);
  EXPECT_DOUBLE_EQ(result[v1].first, 2);
  EXPECT_DOUBLE_EQ(result[v2].first, 7);
  EXPECT_DOUBLE_EQ(result[v3].first, 4);
  EXPECT_DOUBLE_EQ(result[v4].first, -2);

  // Check predecessors
  EXPECT_EQ(result[v0].second, rondo::graph::VERTEX_END);
  EXPECT_EQ(result[v1].second, v3);
  EXPECT_EQ(result[v2].second, v0);
  EXPECT_EQ(result[v3].second, v2);
  EXPECT_EQ(result[v4].second, v1);
}

TEST_F(GraphTest, BellmanFordEarlyExit) {
  // Create a directed graph with a cycle
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();

  g_directed.add_edge(v0, v1, rondo::graph::weight(4));
  g_directed.add_edge(v1, v2, rondo::graph::weight(-6));
  g_directed.add_edge(v2, v0, rondo::graph::weight(3));

  // Compute shortest paths from v0 using Bellman-Ford
  auto result = g_directed.bellman_ford(v0);

  // Check distances
  EXPECT_DOUBLE_EQ(result[v0].first, 0);
  EXPECT_DOUBLE_EQ(result[v1].first, 4);
  EXPECT_DOUBLE_EQ(result[v2].first, -2);

  // Check predecessors
  EXPECT_EQ(result[v0].second, rondo::graph::VERTEX_END);
  EXPECT_EQ(result[v1].second, v0);
  EXPECT_EQ(result[v2].second, v1);
}

TEST_F(GraphTest, BellmanFordNegativeCycle) {
  // Create a directed graph with a negative cycle
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  auto v5 = g_directed.add_vertex();

  g_directed.add_edge(v0, v1, rondo::graph::weight(5));
  g_directed.add_edge(v0, v2, rondo::graph::weight(4));
  g_directed.add_edge(v1, v2, rondo::graph::weight(-2));
  g_directed.add_edge(v1, v3, rondo::graph::weight(1));
  g_directed.add_edge(v2, v3, rondo::graph::weight(2));
  g_directed.add_edge(v2, v4, rondo::graph::weight(1));
  g_directed.add_edge(v2, v5, rondo::graph::weight(4));
  g_directed.add_edge(v3, v1, rondo::graph::weight(-1));
  g_directed.add_edge(v3, v5, rondo::graph::weight(3));
  g_directed.add_edge(v4, v5, rondo::graph::weight(4));

  // Compute shortest paths from v0 using Bellman-Ford
  EXPECT_THROW({
    g_directed.bellman_ford(v0);
  }, std::runtime_error);
}

TEST_F(GraphTest, FloydWarshallZeroEdges) {
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();

  auto result = g_directed.floyd_warshall();
  EXPECT_FALSE(result.distance(v0, v1).has_value());
}

TEST_F(GraphTest, FloydWarshallSelfLoop) {
  auto v0 = g_directed.add_vertex();
  g_directed.add_edge(v0, v0, rondo::graph::weight(1));

  auto result = g_directed.floyd_warshall();
  EXPECT_DOUBLE_EQ(result.distance(v0, v0).value(), 0);

  std::vector<rondo::graph::vertex> expect = {v0};
  EXPECT_EQ(result.path(v0, v0), expect);
}

TEST_F(GraphTest, FloydWarshallMaximumWeight) {
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  g_directed.add_edge(v0, v1, rondo::graph::WEIGHT_INF);

  auto result = g_directed.floyd_warshall();
  EXPECT_DOUBLE_EQ(result.distance(v0, v1).value(), rondo::graph::WEIGHT_INF);
}

TEST_F(GraphTest, FloydWarshallSingleVertex) {
  auto v0 = g_directed.add_vertex();

  auto result = g_directed.floyd_warshall();
  EXPECT_DOUBLE_EQ(result.distance(v0, v0).value(), 0);
}

TEST_F(GraphTest, FloydWarshallDisconnectedGraph) {
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  g_directed.add_edge(v0, v1, 1.0);

  auto result = g_directed.floyd_warshall();
  EXPECT_TRUE(result.distance(v0, v1).has_value());
  EXPECT_FALSE(result.distance(v1, v0).has_value());
  EXPECT_TRUE(result.path(v1, v0).empty());
}

TEST_F(GraphTest, FloydWarshallNegativeCycleTwoVertices) {
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();

  g_directed .add_edge(v0, v1, rondo::graph::weight(1));
  g_directed .add_edge(v1, v0, rondo::graph::weight(-2));
  EXPECT_THROW({
    g_directed.floyd_warshall();
  }, std::runtime_error);
}

TEST_F(GraphTest, FloydWarshallNegativeCycleBasic) {
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();

  g_directed .add_edge(v0, v1, rondo::graph::weight(2));
  g_directed .add_edge(v1, v2, rondo::graph::weight(3));
  g_directed .add_edge(v2, v1, rondo::graph::weight(-6));
  EXPECT_THROW({
    g_directed.floyd_warshall();
  }, std::runtime_error);
}

TEST_F(GraphTest, FloydWarshall) {
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  auto v5 = g_directed.add_vertex();

  g_directed.add_edge(v0, v1, rondo::graph::weight(6));
  g_directed.add_edge(v0, v2, rondo::graph::weight(4));
  g_directed.add_edge(v2, v3, rondo::graph::weight(3));
  g_directed.add_edge(v3, v4, rondo::graph::weight(-1));
  g_directed.add_edge(v4, v5, rondo::graph::weight(-2));
  g_directed.add_edge(v5, v1, rondo::graph::weight(1));

  auto result = g_directed.floyd_warshall();

  // dist.at(from).at(to)
  EXPECT_DOUBLE_EQ(result.distance(v0, v0).value(), 0);
  EXPECT_DOUBLE_EQ(result.distance(v0, v1).value(), 5);

  std::vector<rondo::graph::vertex> expect = {v0, v2, v3, v4, v5, v1};
  EXPECT_EQ(result.path(v0, v1), expect);
}

TEST_F(GraphTest, MST_EmptyGraph) {
  auto mst = g_undirected.mst();
  EXPECT_TRUE(mst.empty());
}

TEST_F(GraphTest, MST_SingleVertex) {
  g_undirected.add_vertex();
  auto mst = g_undirected.mst();
  EXPECT_EQ(mst.size(), 1);
}

TEST_F(GraphTest, MST_Connected) {
  g_undirected.add_edge(0, 1, rondo::graph::weight(10));
  g_undirected.add_edge(0, 2, rondo::graph::weight(6));
  g_undirected.add_edge(0, 3, rondo::graph::weight(5));
  g_undirected.add_edge(1, 3, rondo::graph::weight(15));
  g_undirected.add_edge(2, 3, rondo::graph::weight(4));

  auto mst = g_undirected.mst();

  EXPECT_EQ(mst.size(), 4);
}

TEST_F(GraphTest, MST_Disconnected) {
  g_undirected.add_vertex(0);
  g_undirected.add_edge(1, 2, rondo::graph::weight(15));
  g_undirected.add_edge(2, 3, rondo::graph::weight(10));

  auto mst0 = g_undirected.mst(0);
  auto mst1 = g_undirected.mst(1);

  EXPECT_EQ(mst0.size(), 1);
  EXPECT_EQ(mst1.size(), 3);
}

TEST_F(GraphTest, MST_DirectedGraph) {
  g_directed.add_edge(0, 1, rondo::graph::weight(5));
  g_directed.add_edge(1, 2, rondo::graph::weight(10));
  g_directed.add_edge(2, 0, rondo::graph::weight(5));

  EXPECT_THROW(g_directed.mst(), std::runtime_error);
}

TEST_F(GraphTest, FlowNetworkAddEdge) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2, rondo::graph::capacity(5));

  EXPECT_TRUE(g_directed.has_edge(v1, v2));
  EXPECT_FALSE(g_directed.has_edge(v2, v1)); // Should not exist for directed graph
}

TEST_F(GraphTest, FlowNetworkGetCapacity) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2, rondo::graph::capacity(5));

  auto result = g_directed.get_capacity(v1, v2);
  EXPECT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 5);
}

TEST_F(GraphTest, MaxFlowUndirectedError) {
  g_undirected.add_edge(0, 1, rondo::graph::capacity(5));
  g_undirected.add_edge(1, 2, rondo::graph::capacity(10));
  g_undirected.add_edge(2, 0, rondo::graph::capacity(5));

  auto result = g_undirected.max_flow(0, 2);
  EXPECT_FALSE(result.has_value());
}

TEST_F(GraphTest, MaxFlowNoSuchSource) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(5));
  g_directed.add_edge(1, 2, rondo::graph::capacity(10));
  g_directed.add_edge(2, 0, rondo::graph::capacity(5));

  auto result = g_directed.max_flow(3, 2);
  EXPECT_FALSE(result.has_value());
}

TEST_F(GraphTest, MaxFlowNoSuchSink) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(5));
  g_directed.add_edge(1, 2, rondo::graph::capacity(10));
  g_directed.add_edge(2, 0, rondo::graph::capacity(5));

  auto result = g_directed.max_flow(0, 3);
  EXPECT_FALSE(result.has_value());
}

TEST_F(GraphTest, MaxFlowOneWay) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(10));
  g_directed.add_edge(1, 2, rondo::graph::capacity(5));

  auto result = g_directed.max_flow(0, 2);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 5);
}

TEST_F(GraphTest, MaxFlowWithIntermediateNodes) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(10));
  g_directed.add_edge(1, 2, rondo::graph::capacity(5));
  g_directed.add_edge(2, 3, rondo::graph::capacity(8));

  auto result = g_directed.max_flow(0, 3);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 5);
}

TEST_F(GraphTest, MaxFlowMultiplePaths) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(10));
  g_directed.add_edge(1, 3, rondo::graph::capacity(10));
  g_directed.add_edge(0, 2, rondo::graph::capacity(15));
  g_directed.add_edge(2, 3, rondo::graph::capacity(10));

  auto result = g_directed.max_flow(0, 3);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 20);  // Sum of capacities of two paths
}

TEST_F(GraphTest, MaxFlowWithLoops) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(10));
  g_directed.add_edge(1, 2, rondo::graph::capacity(5));
  g_directed.add_edge(2, 1, rondo::graph::capacity(3));  // Loop back
  g_directed.add_edge(2, 3, rondo::graph::capacity(8));

  auto result = g_directed.max_flow(0, 3);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 5);  // Capacity limited by the 1->2 edge
}

TEST_F(GraphTest, MaxFlowBasic0) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(10));
  g_directed.add_edge(0, 3, rondo::graph::capacity(10));
  g_directed.add_edge(1, 2, rondo::graph::capacity(4));
  g_directed.add_edge(1, 3, rondo::graph::capacity(2));
  g_directed.add_edge(1, 4, rondo::graph::capacity(8));
  g_directed.add_edge(2, 5, rondo::graph::capacity(10));
  g_directed.add_edge(3, 4, rondo::graph::capacity(9));
  g_directed.add_edge(4, 2, rondo::graph::capacity(6));
  g_directed.add_edge(4, 5, rondo::graph::capacity(10));

  auto result = g_directed.max_flow(0, 5);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 19);
}

TEST_F(GraphTest, MaxFlowBasic1) {
  g_directed.add_edge(0, 1, rondo::graph::capacity(16));
  g_directed.add_edge(0, 2, rondo::graph::capacity(13));
  g_directed.add_edge(1, 2, rondo::graph::capacity(10));
  g_directed.add_edge(1, 3, rondo::graph::capacity(12));
  g_directed.add_edge(2, 1, rondo::graph::capacity(4));
  g_directed.add_edge(2, 4, rondo::graph::capacity(14));
  g_directed.add_edge(3, 2, rondo::graph::capacity(9));
  g_directed.add_edge(3, 5, rondo::graph::capacity(20));
  g_directed.add_edge(4, 3, rondo::graph::capacity(7));
  g_directed.add_edge(4, 5, rondo::graph::capacity(4));

  auto result = g_directed.max_flow(0, 5);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 23);
}

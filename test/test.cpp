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
  auto prop = g_undirected.get_property(v1);
  EXPECT_EQ(prop.label, "label1");
  EXPECT_EQ(prop.color, "color1");
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

TEST_F(GraphTest, GetNeighbors) {
  auto v1 = g_undirected.add_vertex();
  auto v2 = g_undirected.add_vertex();
  g_undirected.add_edge(v1, v2);

  auto neighbors = g_undirected.get_neighbors(v1);
  EXPECT_EQ(neighbors.size(), 1);
  EXPECT_EQ(neighbors[0].first, v2);
}

TEST_F(GraphTest, GetWeight) {
  auto v1 = g_undirected.add_vertex();
  auto v2 = g_undirected.add_vertex();
  g_undirected.add_edge(v1, v2, 2.5);

  rondo::weight w = g_undirected.get_weight(v1, v2);
  EXPECT_DOUBLE_EQ(w, 2.5);
}

TEST_F(GraphTest, SetProperty) {
  auto v1 = g_undirected.add_vertex();
  rondo::graph::property p("new_label", "new_color");
  g_undirected.set_property(v1, p);

  auto prop = g_undirected.get_property(v1);
  EXPECT_EQ(prop.label, "new_label");
  EXPECT_EQ(prop.color, "new_color");
}

TEST_F(GraphTest, ToJson) {
  auto v1 = g_undirected.add_vertex(rondo::graph::property("node1", "red"));
  auto v2 = g_undirected.add_vertex(rondo::graph::property("node2", "blue"));
  g_undirected.add_edge(v1, v2, 3.5);

  auto json = g_undirected.to_json();

  EXPECT_EQ(json["directed"], false);
  EXPECT_EQ(json["nodes"].size(), 2);
  EXPECT_EQ(json["edges"].size(), 2); // Since it's undirected, there should be two edges (v1-v2 and v2-v1)

  // ... more checks if needed
}

TEST_F(GraphTest, FromJson) {
  std::string json_content = R"(
  {
    "directed": false,
    "nodes": [
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
  EXPECT_DOUBLE_EQ(g_undirected.get_weight(1, 2), 3.5);
}

TEST_F(GraphTest, DfsTraversal) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2);
  g_directed.add_edge(v1, v3);
  g_directed.add_edge(v2, v4);

  std::vector<rondo::vertex> visited_order;
  g_directed.dfs(v1, [&visited_order](rondo::vertex &v) {
    visited_order.push_back(v);
  });

  std::vector<rondo::vertex> expected_order = {v1, v2, v4, v3};
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

  std::vector<rondo::vertex> visited_order;
  g_directed.bfs(v1, [&visited_order](rondo::vertex &v) {
    visited_order.push_back(v);
  });

  std::vector<rondo::vertex> expected_order = {v1, v2, v3, v4};
  EXPECT_EQ(visited_order, expected_order);
}

TEST_F(GraphTest, DijkstraShortestPath) {
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  g_directed.add_edge(v1, v2, 1);
  g_directed.add_edge(v2, v3, 1);
  g_directed.add_edge(v3, v4, 1);
  g_directed.add_edge(v1, v4, 10);

  auto result = g_directed.dijkstra(v1);

  std::vector<rondo::vertex> path_to_v4 = result.path_to(v4);
  std::vector<rondo::vertex> expected_path = {v1, v2, v3, v4};
  EXPECT_EQ(path_to_v4, expected_path);
  EXPECT_DOUBLE_EQ(result.distances[v4], 3);
}

TEST_F(GraphTest, BellmanFordShortestPath) {
  // Create a directed graph
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();

  // Add edges with weights
  g_directed.add_edge(v0, v1, 6);
  g_directed.add_edge(v0, v2, 7);
  g_directed.add_edge(v1, v2, 8);
  g_directed.add_edge(v1, v3, 5);
  g_directed.add_edge(v1, v4, -4);
  g_directed.add_edge(v2, v3, -3);
  g_directed.add_edge(v2, v4, 9);
  g_directed.add_edge(v3, v1, -2);
  g_directed.add_edge(v4, v0, 2);
  g_directed.add_edge(v4, v3, 7);

  // Compute shortest paths from v0 using Bellman-Ford
  auto result = g_directed.bellman_ford(v0);

  // Check distances
  EXPECT_DOUBLE_EQ(result.distances[v0], 0);
  EXPECT_DOUBLE_EQ(result.distances[v1], 2);
  EXPECT_DOUBLE_EQ(result.distances[v2], 7);
  EXPECT_DOUBLE_EQ(result.distances[v3], 4);
  EXPECT_DOUBLE_EQ(result.distances[v4], -2);

  // Check predecessors
  EXPECT_EQ(result.predecessors[v0], rondo::graph::null_vertex());
  EXPECT_EQ(result.predecessors[v1], v3);
  EXPECT_EQ(result.predecessors[v2], v0);
  EXPECT_EQ(result.predecessors[v3], v2);
  EXPECT_EQ(result.predecessors[v4], v1);
}

TEST_F(GraphTest, BellmanFordEarlyExit) {
  // Create a directed graph with a cycle
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();

  g_directed.add_edge(v0, v1, 4);
  g_directed.add_edge(v1, v2, -6);
  g_directed.add_edge(v2, v0, 3);

  // Compute shortest paths from v0 using Bellman-Ford
  auto result = g_directed.bellman_ford(v0);

  // Check distances
  EXPECT_DOUBLE_EQ(result.distances[v0], 0);
  EXPECT_DOUBLE_EQ(result.distances[v1], 4);
  EXPECT_DOUBLE_EQ(result.distances[v2], -2);

  // Check predecessors
  EXPECT_EQ(result.predecessors[v0], rondo::graph::null_vertex());
  EXPECT_EQ(result.predecessors[v1], v0);
  EXPECT_EQ(result.predecessors[v2], v1);
}

TEST_F(GraphTest, BellmanFordNegativeCycle) {
  // Create a directed graph with a negative cycle
  auto v0 = g_directed.add_vertex();
  auto v1 = g_directed.add_vertex();
  auto v2 = g_directed.add_vertex();
  auto v3 = g_directed.add_vertex();
  auto v4 = g_directed.add_vertex();
  auto v5 = g_directed.add_vertex();

  g_directed.add_edge(v0, v1, 5);
  g_directed.add_edge(v0, v2, 4);
  g_directed.add_edge(v1, v2, -2);
  g_directed.add_edge(v1, v3, 1);
  g_directed.add_edge(v2, v3, 2);
  g_directed.add_edge(v2, v4, 1);
  g_directed.add_edge(v2, v5, 4);
  g_directed.add_edge(v3, v1, -1);
  g_directed.add_edge(v3, v5, 3);
  g_directed.add_edge(v4, v5, 4);

  // Compute shortest paths from v0 using Bellman-Ford
  EXPECT_THROW({
    g_directed.bellman_ford(v0);
  }, std::runtime_error);
}

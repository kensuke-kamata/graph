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
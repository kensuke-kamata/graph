#pragma once

#include <nlohmann/json.hpp>

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace rondo {
using vertex = size_t;
using weight = double_t;

class graph {
 public:
  struct property {
    std::string label = "";
    std::string color = "";

    property(const std::string &label = "", const std::string &color = "")
      : label(label), color(color) {}
  };

  using neighbor  = std::pair<vertex, weight>;
  using neighbors = std::vector<neighbor>;

 private:
  std::map<vertex, neighbors> adjacencies = {};
  std::map<vertex, property>  properties  = {};
  vertex next = std::numeric_limits<vertex>::max();
  bool directed = false;

 public:
  graph(bool directed = false) : directed(directed) {}

  void clear() {
    adjacencies.clear();
    properties.clear();
    next = std::numeric_limits<vertex>::max();
  }

  size_t size() const {
    return adjacencies.size();
  }

  bool empty() const {
    return adjacencies.empty();
  }

  const neighbors &operator[](const vertex &v) const {
    return adjacencies.at(v);
  }

  const property &operator()(const vertex &v) const {
    return properties.at(v);
  }

  const neighbors &get_neighbors(const vertex &v) const {
    return adjacencies.at(v);
  }

  const property &get_property(const vertex &v) const {
    return properties.at(v);
  }

  weight get_weight(const vertex &from, const vertex &to) const {
    if (adjacencies.find(from) == adjacencies.end()) {
      throw std::runtime_error("vertex: " + std::to_string(from) + " not found");
    }
    auto &neighbors = adjacencies.at(from);
    auto it = std::find_if(neighbors.begin(), neighbors.end(),
                          [&to](const neighbor &n) {
      return n.first == to;
    });
    if (it == neighbors.end()) {
      throw std::runtime_error("edge: " + std::to_string(from) + " -> " + std::to_string(to) + " not found");
    }
    return it->second;
  }

  void set_property(const vertex &v, const property &p) {
    if (properties.find(v) != properties.end()) {
      properties[v] = p;
    }
  }

  bool has_edge(const vertex &from, const vertex &to) const {
    if (adjacencies.find(from) == adjacencies.end()) {
      return false;
    }
    auto &neighbors = adjacencies.at(from);
    auto it = std::find_if(neighbors.begin(), neighbors.end(),
                          [&to](const neighbor &n) {
      return n.first == to;
    });
    return it != neighbors.end();
  }

  vertex add_vertex(const property &p = property()) {
    next++;
    if (next == std::numeric_limits<vertex>::max()) {
      throw std::runtime_error("vertex: overflow");
    }
    adjacencies[next] = {};
    properties[next]  = p;
    return next;
  }

  vertex add_vertex(const vertex &v, const property &p = property()) {
    if (adjacencies.find(v) != adjacencies.end()) {
      properties[v] = p;
    }
    adjacencies[v] = {};
    properties[v]  = p;
    return v;
  }

  void add_edge(const vertex &from, const vertex &to, const weight &w = 1) {
    if (properties.find(from) == properties.end()) {
      add_vertex(from);
    }
    if (properties.find(to) == properties.end()) {
      add_vertex(to);
    }
    adjacencies[from].emplace_back(std::make_pair(to, w));
    if (!directed) {
      adjacencies[to].emplace_back(std::make_pair(from, w));
    }
  }

  bool remove_edge(const vertex &from, const vertex &to) {
    bool removed = remove_helper(from, to);
    if (!directed && removed) {
      remove_helper(to, from);
    }
    return removed;
  }

  void from_json(const std::string &file) {
    std::ifstream fs(file);
    if (!fs.is_open()) {
      throw std::runtime_error("unable to open: " + file);
    }

    nlohmann::json j;
    fs >> j;

    this->directed = j["directed"].get<bool>();
    auto nodes     = j["nodes"];
    auto edges     = j["edges"];

    for (auto &node : nodes) {
      add_vertex(node["id"], property(node["label"], node["color"]));
    }

    for (auto &edge : edges) {
      add_edge(edge["from"], edge["to"], edge.value("weight", 1.0));
    }
  }

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["directed"] = this->directed;

    // Serialize nodes
    auto nodes = nlohmann::json::array();
    for (const auto &pair : this->properties) {
      nlohmann::json node;
      node["id"]    = pair.first;
      node["label"] = pair.second.label;
      node["color"] = pair.second.color;
      nodes.push_back(node);
    }
    j["nodes"] = nodes;

    // Serialize edges
    auto edges = nlohmann::json::array();
    for (const auto &pair : this->adjacencies) {
      for (const auto &neighbor : pair.second) {
        nlohmann::json edge;
        edge["from"]   = pair.first;
        edge["to"]     = neighbor.first;
        edge["weight"] = neighbor.second;
        edges.push_back(edge);
      }
    }
    j["edges"] = edges;
    return j;
  }

 private:
  bool remove_helper(const vertex &from, const vertex &to) {
    bool removed = false;
    // Find and remove the edge from 'from' to 'to'.
    auto &neighbors = adjacencies[from];
    auto it = std::find_if(neighbors.begin(), neighbors.end(),
                          [&to](const neighbor &n) {
      return n.first == to;
    });
    if (it != neighbors.end()) {
      neighbors.erase(it);
      removed = true;
    }
    return removed;
  }
};
}  // namespace rondo

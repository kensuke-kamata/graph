#pragma once

#include <nlohmann/json.hpp>

#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace rondo {
class graph {
 public:
  using vertex   = size_t;
  using weight   = double_t;
  using edge     = std::pair<vertex, weight>;
  using edges    = std::vector<edge>;
  using function = std::function<void(vertex &)>;

  template <typename T>
  using matrix = std::map<vertex, std::map<vertex, T>>;

  static constexpr vertex VERTEX_END = std::numeric_limits<vertex>::max();
  static constexpr weight WEIGHT_INF = std::numeric_limits<weight>::infinity();

  struct property {
    std::string label = "";
    std::string color = "";

    property(const std::string &label = "", const std::string &color = "")
      : label(label), color(color) {}
  };

 private:
  std::map<vertex, edges>    vertices   = {};
  std::map<vertex, property> properties = {};
  vertex next = VERTEX_END;
  bool directed = false;

 public:
  graph(bool directed = false) : directed(directed) {}

  void clear() {
    vertices.clear();
    properties.clear();
    next = VERTEX_END;
  }

  size_t size() const {
    return vertices.size();
  }

  bool empty() const {
    return vertices.empty();
  }

  const edges &operator[](const vertex &v) const {
    return vertices.at(v);
  }

  const property &operator()(const vertex &v) const {
    return properties.at(v);
  }

  const edges &get_edges(const vertex &v) const {
    return vertices.at(v);
  }

  const property &get_property(const vertex &v) const {
    return properties.at(v);
  }

  std::optional<weight> get_weight(const vertex &from, const vertex &to) const {
    auto it = vertices.find(from);
    if (it == vertices.end()) {
      return std::nullopt;
    }
    auto &edges = it->second;
    for (const auto &edge : edges) {
      if (edge.first == to) {
        return edge.second;
      }
    }
    return  std::nullopt;
  }

  void set_property(const vertex &v, const property &p) {
    if (properties.find(v) != properties.end()) {
      properties[v] = p;
    }
  }

  bool has_edge(const vertex &from, const vertex &to) const {
    if (vertices.find(from) == vertices.end()) {
      return false;
    }
    auto &edges = vertices.at(from);
    auto it = std::find_if(edges.begin(), edges.end(),
                          [&to](const edge &e) {
      return e.first == to;
    });
    return it != edges.end();
  }

  vertex add_vertex(const property &p = property()) {
    next++;
    if (next == VERTEX_END) {
      throw std::runtime_error("vertex: overflow");
    }
    vertices[next]   = {};
    properties[next] = p;
    return next;
  }

  vertex add_vertex(const vertex &v, const property &p = property()) {
    if (vertices.find(v) != vertices.end()) {
      return v;
    }
    vertices[v]   = {};
    properties[v] = p;
    return v;
  }

  void add_edge(const vertex &from, const vertex &to, const weight &w = 1.0) {
    if (vertices.find(from) == vertices.end()) {
      add_vertex(from);
    }
    if (vertices.find(to) == vertices.end()) {
      add_vertex(to);
    }
    vertices.at(from).emplace_back(std::make_pair(to, w));
    if (!directed) {
      vertices.at(to).emplace_back(std::make_pair(from, w));
    }
  }

  bool remove_edge(const vertex &from, const vertex &to) {
    bool removed = remove_helper(from, to);
    if (!directed && removed) {
      remove_helper(to, from);
    }
    return removed;
  }

  void from_json(const std::string &path) {
    std::ifstream fs(path);
    if (!fs.is_open()) {
      throw std::runtime_error("unable to open: " + path);
    }

    nlohmann::json j;
    fs >> j;

    directed = j["directed"].get<bool>();
    auto vertices = j["vertices"];
    auto edges    = j["edges"];

    for (auto &v : vertices) {
      add_vertex(v["id"].get<vertex>(), property(
        v["label"].get<std::string>(),
        v["color"].get<std::string>()));
    }

    for (auto &edge : edges) {
      add_edge(edge["from"].get<vertex>(),
                edge["to"].get<vertex>(),
                edge["weight"].get<weight>());
    }
  }

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["directed"] = directed;

    // Serialize nodes
    auto vs = nlohmann::json::array();
    for (const auto &pair : properties) {
      nlohmann::json v;
      v["id"]    = pair.first;
      v["label"] = pair.second.label;
      v["color"] = pair.second.color;
      vs.push_back(v);
    }
    j["vertices"] = vs;

    // Serialize edges
    auto es = nlohmann::json::array();
    for (const auto &pair : vertices) {
      for (const auto &edge : pair.second) {
        nlohmann::json e;
        e["from"]   = pair.first;
        e["to"]     = edge.first;
        e["weight"] = edge.second;
        es.push_back(e);
      }
    }
    j["edges"] = es;
    return j;
  }

  void dfs(vertex &start, const function &f) {
    std::map<vertex, bool> visited;
    dfs_helper(start, visited, f);
  }

  void bfs(vertex &start, const function &f) {
    auto it = vertices.find(start);
    if (it == vertices.end()) {
      return;
    }

    std::map<vertex, bool> visited;
    std::queue<vertex> q;

    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
      auto &v = q.front();
      q.pop();

      f(v);

      auto &edges = vertices.at(v);
      for (auto &edge : edges) {
        if (!visited[edge.first]) {
          visited[edge.first] = true;
          q.push(edge.first);
        }
      }
    }
  }

  struct result {
    std::map<vertex, weight> distances    = {};
    std::map<vertex, vertex> predecessors = {};

    std::vector<vertex> path_to(const vertex &end) {
      std::vector<vertex> path;
      if (predecessors.find(end) == predecessors.end()) {
        return path;
      }
      for (auto it = end; it != VERTEX_END; it = predecessors.at(it)) {
        path.emplace_back(it);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
  };

  result dijkstra(const vertex &start) const {
    std::priority_queue<std::pair<weight, vertex>,
                        std::vector<std::pair<weight, vertex>>,
                        std::greater<std::pair<weight, vertex>>> pq;

    result res;
    for (const auto &pair : vertices) {
      res.distances[pair.first]    = WEIGHT_INF;
      res.predecessors[pair.first] = VERTEX_END;
    }

    res.distances[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
      auto [d, u] = pq.top();
      pq.pop();

      if (d > res.distances[u]) { continue; }

      const auto &edges = vertices.at(u);
      for (const auto &edge : edges) {
        auto v = edge.first;
        auto w = edge.second;
        auto alt = res.distances[u] + w;
        if (alt < res.distances[v]) {
          res.distances[v]    = alt;
          res.predecessors[v] = u;
          pq.emplace(alt, v);
        }
      }
    }
    return res;
  }

  result bellman_ford(const vertex &start) const {
    result res;
    for (const auto &pair : vertices) {
      res.distances[pair.first]    = WEIGHT_INF;
      res.predecessors[pair.first] = VERTEX_END;
    }
    res.distances[start] = 0;

    for (size_t i = 0; i < vertices.size(); i++) {
      auto updated = false;
      for (const auto &pair : vertices) {
        auto u = pair.first;
        for (const auto &edge : pair.second) {
          auto v = edge.first;
          auto w = edge.second;
          if (res.distances[u] + w < res.distances[v]) {
            res.distances[v]    = res.distances[u] + w;
            res.predecessors[v] = u;
            updated = true;
          }
        }
      }
      if (!updated) { break; }
      if (i == vertices.size() - 1 && updated) {
        throw std::runtime_error("negative cycle detected");
      }
    }
    return res;
  }

  struct floyd_warshall_result {
    matrix<weight> dist = {};
    matrix<vertex> pred = {};

    std::optional<weight> distance(const vertex &from, const vertex &to) const {
      if (dist.find(from) == dist.end() ||
          dist.at(from).find(to) == dist.at(from).end()) {
        return std::nullopt;
      }
      return dist.at(from).at(to);
    }

    std::vector<vertex> path(const vertex &from, const vertex &to) {
      std::vector<vertex> p;
      if (pred.find(from) == pred.end() ||
          pred.at(from).find(to) == pred.at(from).end()) {
        return p;
      }

      auto iter = to;
      while (iter != from) {
        p.emplace_back(iter);
        iter = pred.at(from).at(iter);
      }
      p.emplace_back(from);
      std::reverse(p.begin(), p.end());
      return p;
    }
  };

  floyd_warshall_result floyd_warshall() const {
    floyd_warshall_result res;
    for (const auto &pair : vertices) {
      auto u = pair.first;
      for (const auto &edge : pair.second) {
        auto v = edge.first;
        auto w = edge.second;
        res.dist[u][v] = w;
        res.pred[u][v] = u;
      }
      res.dist[u][u] = 0;
    }

    for (const auto &k_pair : vertices) {
      for (const auto &i_pair : vertices) {
        for (const auto &j_pair : vertices) {
          auto k = k_pair.first, i = i_pair.first, j = j_pair.first;
          if (res.dist.at(i).find(k) != res.dist.at(i).end() &&
              res.dist.at(k).find(j) != res.dist.at(k).end()) {
            auto alt = res.dist.at(i).at(k) + res.dist.at(k).at(j);
            if (res.dist.at(i).find(j) == res.dist.at(i).end() ||
                alt < res.dist[i][j]) {
              res.dist[i][j] = alt;
              res.pred[i][j] = res.pred[k][j];
            }
          }
        }
      }
    }

    for (const auto &pair : vertices) {
      auto u = pair.first;
      if (res.dist[u][u] < 0) {
        throw std::runtime_error("negative cycle detected");
      }
    }

    return res;
  }

 private:
  bool remove_helper(const vertex &from, const vertex &to) {
    bool removed = false;
    auto vit = vertices.find(from);
    if (vit == vertices.end()) {
      return removed;
    }
    // Find and remove the edge from 'from' to 'to'.
    auto &edges = vit->second;
    auto eit = std::find_if(edges.begin(), edges.end(), [&to](const edge &e) {
      return e.first == to;
    });
    if (eit != edges.end()) {
      edges.erase(eit);
      removed = true;
    }
    return removed;
  }

  void dfs_helper(vertex &v, std::map<vertex, bool> &visited, const function &f) {
    auto it = vertices.find(v);
    if (it == vertices.end()) {
      return;
    }

    visited[v] = true;
    f(v);

    auto &edges = vertices.at(v);
    for (auto &edge : edges) {
      if (!visited[edge.first]) {
        dfs_helper(edge.first, visited, f);
      }
    }
  }
};
}  // namespace rondo

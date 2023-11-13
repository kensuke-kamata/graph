#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
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
  using edge     = std::pair<vertex, vertex>;
  using function = std::function<void(const vertex &)>;

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
  std::map<vertex, property> vertices = {};
  std::vector<edge>          edges    = {};
  std::map<edge, weight>     weights  = {};

  vertex next = VERTEX_END;
  bool directed = false;

 public:
  graph(bool directed = false) : directed(directed) {}

  void clear() {
    vertices.clear();
    next = VERTEX_END;
  }

  size_t size() const {
    return vertices.size();
  }

  bool empty() const {
    return vertices.empty();
  }

  std::vector<edge> operator[](const vertex &v) const {
    std::vector<edge> ret;
    std::copy_if(edges.begin(), edges.end(), std::back_inserter(ret),
                 [&v](const edge &e) {
      return e.first == v;
    });
    return ret;
  }

  std::optional<property> operator()(const vertex &v) const {
    auto it = vertices.find(v);
    if (it == vertices.end()) {
      return std::nullopt;
    }
    return vertices.at(v);
  }

  std::vector<edge> get_edges(const vertex &v) const {
    std::vector<edge> ret;
    std::copy_if(edges.begin(), edges.end(), std::back_inserter(ret),
                 [&v](const edge &e) {
      return e.first == v;
    });
    return ret;
  }

  std::optional<property> get_property(const vertex &v) const {
    auto it = vertices.find(v);
    if (it == vertices.end()) {
      return std::nullopt;
    }
    return vertices.at(v);
  }

  std::optional<weight> get_weight(const edge &e) const {
    auto it = weights.find(e);
    if (it == weights.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<weight> get_weight(const vertex &from, const vertex &to) const {
    auto it = weights.find(edge(from, to));
    if (it == weights.end()) {
      return std::nullopt;
    }
    return  it->second;
  }

  void set_property(const vertex &v, property p) {
    auto it = vertices.find(v);
    if (it != vertices.end()) {
      vertices[v] = p;
    }
  }

  bool has_vertex(const vertex &v) const {
    return vertices.find(v) != vertices.end();
  }

  bool has_edge(const vertex &from, const vertex &to) const {
    return std::find(edges.begin(), edges.end(), edge(from, to)) != edges.end();
  }

  vertex add_vertex(property p = property()) {
    next++;
    while (vertices.find(next) != vertices.end()) {
      next++;
    }
    if (next == VERTEX_END) {
      throw std::runtime_error("vertex: overflow");
    }
    vertices[next] = p;
    return next;
  }

  vertex add_vertex(const vertex &v, property p = property()) {
    if (vertices.find(v) != vertices.end()) {
      throw std::runtime_error("vertex: already exists");
    }
    if (v == VERTEX_END) {
      throw std::runtime_error("vertex: overflow");
    }
    vertices[v] = p;
    return v;
  }

  void add_edge(const edge &e, const weight &w = 1.0) {
    add_edge(e.first, e.second, w);
  }

  void add_edge(const vertex &from, const vertex &to, const weight &w = 1.0) {
    if (!has_vertex(from)) {
      add_vertex(from);
    }
    if (!has_vertex(to)) {
      add_vertex(to);
    }
    add_edge_helper(from, to, w);
    if (!directed) {
      add_edge_helper(to, from, w);
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
    for (const auto &pair : vertices) {
      nlohmann::json v;
      v["id"]    = pair.first;
      v["label"] = pair.second.label;
      v["color"] = pair.second.color;
      vs.push_back(v);
    }
    j["vertices"] = vs;

    // Serialize edges
    auto es = nlohmann::json::array();
    for (const auto &edge : edges) {
      nlohmann::json e;
      e["from"]   = edge.first;
      e["to"]     = edge.second;
      e["weight"] = get_weight(edge).value();
      es.push_back(e);
    }
    j["edges"] = es;
    return j;
  }

  void dfs(const vertex &start, const function &f) {
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

      auto edges = get_edges(v);
      for (auto &edge : edges) {
        if (!visited[edge.second]) {
          visited[edge.second] = true;
          q.push(edge.second);
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

      auto edges = get_edges(u);
      for (const auto &edge : edges) {
        auto v = edge.second;
        auto w = get_weight(u, v).value();
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
      for (const auto &edge : edges) {
        auto u = edge.first;
        auto v = edge.second;
        auto w = get_weight(u, v).value();
        auto alt = res.distances[u] + w;
        if (alt < res.distances[v]) {
          res.distances[v]    = alt;
          res.predecessors[v] = u;
          updated = true;
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
      for (const auto &edge : get_edges(u)) {
        auto v = edge.second;
        auto w = get_weight(u, v).value();
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

  graph mst() {
    auto start = vertices.begin()->first;
    return mst_helper(start);
  }

  graph mst(const vertex &start) {
    if (vertices.find(start) == vertices.end()) {
      return graph();
    }
    return mst_helper(start);
  }

 private:
  void add_edge_helper(const vertex &from, const vertex &to, const weight &w) {
    auto from_to = edge(from, to);
    edges.emplace_back(from_to);
    weights[from_to] = w;
  }

  bool remove_helper(const vertex &from, const vertex &to) {
    bool removed = false;
    auto from_to = edge(from, to);
    auto it = std::find(edges.begin(), edges.end(), from_to);
    if (it != edges.end()) {
      edges.erase(it);
      weights.erase(from_to);
      removed = true;
    }
    return removed;
  }

  void dfs_helper(const vertex &v, std::map<vertex, bool> &visited, const function &f) {
    auto it = vertices.find(v);
    if (it == vertices.end()) {
      return;
    }

    visited[v] = true;
    f(v);

    auto edges = get_edges(v);
    for (auto &edge : edges) {
      if (!visited[edge.second]) {
        dfs_helper(edge.second, visited, f);
      }
    }
  }

  graph mst_helper(const vertex &start) {
    if (directed) {
      throw std::runtime_error("mst: directed graph");
    }

    graph g;

    if (empty()) {
      return g;
    }

    std::map<vertex, bool> visited;
    for (const auto &pair : vertices) {
      visited[pair.first] = false;
    }

    auto cmp = [](const std::pair<weight, edge>& left,
                  const std::pair<weight, edge>& right) {
      return left.first > right.first;
    };
    std::priority_queue<std::pair<weight, edge>,
                        std::vector<std::pair<weight, edge>>,
                        decltype(cmp)> pq(cmp);

    visited[start] = true;
    g.add_vertex(start, vertices.at(start));

    for (const auto &edge : get_edges(start)) {
      pq.emplace(get_weight(edge).value(), edge);
    }

    while (!pq.empty()) {
      auto p = pq.top();
      auto w = p.first;
      auto u = p.second.first;
      auto v = p.second.second;
      pq.pop();

      if (visited[v]) { continue; }

      visited[v] = true;
      g.add_vertex(v, vertices.at(v));
      g.add_edge(u, v, w);

      for (const auto &edge : get_edges(v)) {
        if (!visited[edge.second]) {
          pq.emplace(get_weight(edge).value(), edge);
        }
      }
    }
    return g;
  }
};
}  // namespace rondo

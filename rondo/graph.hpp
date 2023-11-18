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
  using capacity = size_t;
  using flow     = size_t;

  using edge = std::pair<vertex, vertex>;
  using path = std::map<vertex, vertex>; // e.g. path parent; then parent[v] = u means u -> v

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

  using vertices   = std::map<vertex, property>;
  using edges      = std::vector<edge>;
  using weights    = std::map<edge, weight>;
  using capacities = std::map<edge, capacity>;

 private:
  vertices   vertices_   = {};
  edges      edges_      = {};
  weights    weights_    = {};
  capacities capacities_ = {};

  vertex next_ = VERTEX_END;
  bool directed_ = false;

 public:
  graph(bool directed = false) : directed_(directed) {}

  void clear() {
    vertices_.clear();
    next_ = VERTEX_END;
  }

  size_t size() const {
    return vertices_.size();
  }

  bool empty() const {
    return vertices_.empty();
  }

  edges operator[](const vertex &v) const {
    edges ret;
    std::copy_if(edges_.begin(), edges_.end(), std::back_inserter(ret),
                 [&v](const edge &e) {
      return e.first == v;
    });
    return ret;
  }

  std::optional<property> operator()(const vertex &v) const {
    auto it = vertices_.find(v);
    if (it == vertices_.end()) {
      return std::nullopt;
    }
    return vertices_.at(v);
  }

  vertices get_vertices() const {
    return vertices_;
  }

  edges get_edges(const vertex &v) const {
    edges ret;
    std::copy_if(edges_.begin(), edges_.end(), std::back_inserter(ret),
                 [&v](const edge &e) {
      return e.first == v;
    });
    return ret;
  }

  std::optional<property> get_property(const vertex &v) const {
    auto it = vertices_.find(v);
    if (it == vertices_.end()) {
      return std::nullopt;
    }
    return vertices_.at(v);
  }

  std::optional<weight> get_weight(const edge &e) const {
    auto it = weights_.find(e);
    if (it == weights_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<weight> get_weight(const vertex &from, const vertex &to) const {
    auto it = weights_.find(edge(from, to));
    if (it == weights_.end()) {
      return std::nullopt;
    }
    return  it->second;
  }

  std::optional<capacity> get_capacity(const edge &e) const {
    auto it = capacities_.find(e);
    if (it == capacities_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<capacity> get_capacity(const vertex &from, const vertex &to) const {
    auto it = capacities_.find(edge(from, to));
    if (it == capacities_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  void set_property(const vertex &v, property p) {
    auto it = vertices_.find(v);
    if (it != vertices_.end()) {
      vertices_[v] = p;
    }
  }

  bool has_vertex(const vertex &v) const {
    return vertices_.find(v) != vertices_.end();
  }

  bool has_edge(const vertex &from, const vertex &to) const {
    return std::find(edges_.begin(), edges_.end(), edge(from, to)) != edges_.end();
  }

  vertex add_vertex(property p = property()) {
    next_++;
    while (vertices_.find(next_) != vertices_.end()) {
      next_++;
    }
    if (next_ == VERTEX_END) {
      throw std::runtime_error("vertex: overflow");
    }
    vertices_[next_] = p;
    return next_;
  }

  vertex add_vertex(const vertex &v, property p = property()) {
    if (vertices_.find(v) != vertices_.end()) {
      throw std::runtime_error("vertex: already exists");
    }
    if (v == VERTEX_END) {
      throw std::runtime_error("vertex: overflow");
    }
    vertices_[v] = p;
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
    if (!directed_) {
      add_edge_helper(to, from, w);
    }
  }

  void add_edge(const vertex &from, const vertex &to, const capacity &c) {
    if (!has_vertex(from)) {
      add_vertex(from);
    }
    if (!has_vertex(to)) {
      add_vertex(to);
    }
    add_edge_helper(from, to, c);
    if (!directed_) {
      add_edge_helper(to, from, c);
    }
  }

  bool remove_edge(const vertex &from, const vertex &to) {
    bool removed = remove_helper(from, to);
    if (!directed_ && removed) {
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

    directed_ = j["directed"].get<bool>();
    auto vs = j["vertices"];
    auto es = j["edges"];

    for (auto &v : vs) {
      add_vertex(v["id"].get<vertex>(), property(
        v["label"].get<std::string>(),
        v["color"].get<std::string>()));
    }

    for (auto &e : es) {
      add_edge(e["from"].get<vertex>(),
               e["to"].get<vertex>(),
               e["weight"].get<weight>());
    }
  }

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["directed"] = directed_;

    // Serialize nodes
    auto vs = nlohmann::json::array();
    for (const auto &pair : vertices_) {
      nlohmann::json v;
      v["id"]    = pair.first;
      v["label"] = pair.second.label;
      v["color"] = pair.second.color;
      vs.push_back(v);
    }
    j["vertices"] = vs;

    // Serialize edges
    auto es = nlohmann::json::array();
    for (const auto &edge : edges_) {
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
    auto it = vertices_.find(start);
    if (it == vertices_.end()) {
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
    for (const auto &pair : vertices_) {
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
    for (const auto &pair : vertices_) {
      res.distances[pair.first]    = WEIGHT_INF;
      res.predecessors[pair.first] = VERTEX_END;
    }
    res.distances[start] = 0;

    for (size_t i = 0; i < vertices_.size(); i++) {
      auto updated = false;
      for (const auto &edge : edges_) {
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
      if (i == vertices_.size() - 1 && updated) {
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
    for (const auto &pair : vertices_) {
      auto u = pair.first;
      for (const auto &edge : get_edges(u)) {
        auto v = edge.second;
        auto w = get_weight(u, v).value();
        res.dist[u][v] = w;
        res.pred[u][v] = u;
      }
      res.dist[u][u] = 0;
    }

    for (const auto &k_pair : vertices_) {
      for (const auto &i_pair : vertices_) {
        for (const auto &j_pair : vertices_) {
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

    for (const auto &pair : vertices_) {
      auto u = pair.first;
      if (res.dist[u][u] < 0) {
        throw std::runtime_error("negative cycle detected");
      }
    }

    return res;
  }

  graph mst() {
    auto start = vertices_.begin()->first;
    return mst_helper(start);
  }

  graph mst(const vertex &start) {
    if (vertices_.find(start) == vertices_.end()) {
      return graph();
    }
    return mst_helper(start);
  }

 private:
  flow bottleneck(const vertex &s, const vertex &t, capacities &residuals, path &parent) {
    flow b = std::numeric_limits<flow>::max();
    for (auto v = t; v != s; v = parent.at(v)) {
      auto u = parent.at(v);
      b = std::min(b, residuals.at(edge(u, v)));
    }
    return b;
  }

  edges get_edges(const vertex &v, capacities &residuals) {
    edges es;
    for (const auto &e : residuals) {
      if (e.first.first == v) {
        es.emplace_back(e.first);
      }
    }
    return es;
  }

  path augment(const vertex &s, const vertex &t, capacities &residuals) {
    // Use breadth-first search to find an augmenting path
    std::map<vertex, bool> visited;
    std::queue<vertex> queue;

    queue.push(s);
    visited[s] = true;

    path parent = {};
    while(!queue.empty()) {
      auto u = queue.front();
      queue.pop();
      if (u == t) {
        return parent;
      }
      for (const auto &e : get_edges(u, residuals)) {
        auto v  = e.second;
        if (!visited[v] && residuals.at(e) > 0) {
          visited[v] = true;
          parent[v]  = u; // Record the path
          queue.push(v);
        }
      }
    }
    return parent;
  }

  capacities gen_residuals() {
    capacities residuals = {};
    for (const auto &e : edges_) {
      auto c = get_capacity(e);
      if (c.has_value()) {
        residuals[e] = c.value();
        auto re = edge(e.second, e.first);
        if (residuals.find(re) == residuals.end()) {
          residuals[re] = 0; // Reverse edge
        }
      }
    }
    return residuals;
  }

 public:
  // Ford-Fulkerson max flow algorithm using DFS
  std::optional<flow> max_flow(const vertex &s, const vertex &t) {
    if (!directed_) {
      return std::nullopt;
    }
    if (!has_vertex(s) || !has_vertex(t)) {
      return  std::nullopt;
    }
    capacities residuals = gen_residuals();
    flow result = 0;
    while (true) {
      path parent = augment(s, t, residuals);
      if (parent.find(t) == parent.end()) {
        break;
      }
      flow b = bottleneck(s, t, residuals, parent);
      for (auto v = t; v != s; v = parent.at(v)) {
        auto u = parent.at(v);
        residuals.at(edge(u, v)) -= b; // Decrease residual capacity of forward edge
        residuals.at(edge(v, u)) += b; // Increase residual capacity of reverse edge
      }
      result += b;
    }
    return result;
  }

 private:
  void add_edge_helper(const vertex &from, const vertex &to, const weight &w) {
    auto from_to = edge(from, to);
    edges_.emplace_back(from_to);
    weights_[from_to] = w;
  }

  void add_edge_helper(const vertex &from, const vertex &to, const capacity &c) {
    auto from_to = edge(from, to);
    edges_.emplace_back(from_to);
    capacities_[from_to] = c;
  }

  bool remove_helper(const vertex &from, const vertex &to) {
    bool removed = false;
    auto from_to = edge(from, to);
    auto it = std::find(edges_.begin(), edges_.end(), from_to);
    if (it != edges_.end()) {
      edges_.erase(it);
      weights_.erase(from_to);
      removed = true;
    }
    return removed;
  }

  void dfs_helper(const vertex &v, std::map<vertex, bool> &visited, const function &f) {
    auto it = vertices_.find(v);
    if (it == vertices_.end()) {
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
    if (directed_) {
      throw std::runtime_error("mst: directed graph");
    }

    graph g;

    if (empty()) {
      return g;
    }

    std::map<vertex, bool> visited;
    for (const auto &pair : vertices_) {
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
    g.add_vertex(start, vertices_.at(start));

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
      g.add_vertex(v, vertices_.at(v));
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

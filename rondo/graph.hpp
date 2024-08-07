#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace std {
template <>
struct hash<std::pair<size_t, size_t>> {
  size_t operator()(const std::pair<size_t, size_t> &p) const {
    auto hash1 = std::hash<size_t>{}(p.first);
    auto hash2 = std::hash<size_t>{}(p.second);
    return hash1 ^ (hash2 << 1);
  }
};
}

namespace rondo {
class graph {
 public:
  using vertex   = size_t;
  using weight   = double_t;
  using capacity = size_t;
  using flow     = size_t;
  using edge     = std::pair<vertex, vertex>;
  using path     = std::unordered_map<vertex, vertex>; // e.g. path parent; then parent[v] = u means u -> v
  using function = std::function<void(const vertex &)>;

  template <typename T>
  using matrix = std::unordered_map<vertex, std::unordered_map<vertex, T>>;

  static constexpr vertex VERTEX_END = std::numeric_limits<vertex>::max();
  static constexpr weight WEIGHT_INF = std::numeric_limits<weight>::infinity();

  struct property {
    std::string label = "";
    std::string color = "";

    property(const std::string &label = "", const std::string &color = "")
      : label(label), color(color) {}
  };

  using vertices   = std::unordered_map<vertex, property>;
  using edges      = std::unordered_set<edge>;
  using weights    = std::unordered_map<edge, weight>;
  using capacities = std::unordered_map<edge, capacity>;

 private:
  vertices   vertices_   = {};
  edges      edges_      = {};
  weights    weights_    = {};
  capacities capacities_ = {};

  vertex next_ = 0;
  bool directed_ = false;

 public:
  mutable std::shared_mutex mutex_;

  graph(bool directed = false) : directed_(directed) {}

  graph(const graph &other)
    : vertices_(other.vertices_), edges_(other.edges_), weights_(other.weights_),
      capacities_(other.capacities_), next_(other.next_), directed_(other.directed_) {}

  graph &operator=(const graph &other) {
    if (this != &other) {
      std::unique_lock lock_this(mutex_);
      std::unique_lock lock_other(other.mutex_);
      vertices_   = other.vertices_;
      edges_      = other.edges_;
      weights_    = other.weights_;
      capacities_ = other.capacities_;
      next_       = other.next_;
      directed_   = other.directed_;
    }
    return *this;
  }

  void clear() {
    std::unique_lock lock(mutex_);
    vertices_.clear();
    edges_.clear();
    weights_.clear();
    capacities_.clear();
    next_ = 0;
  }

  size_t size() const {
    std::shared_lock lock(mutex_);
    return vertices_.size();
  }

  bool empty() const {
    std::shared_lock lock(mutex_);
    return vertices_.empty();
  }

  edges operator[](const vertex &v) const {
    std::shared_lock lock(mutex_);
    edges ret;
    std::copy_if(edges_.begin(), edges_.end(), std::inserter(ret, ret.end()), [&v](const edge &e) {
      return e.first == v;
    });
    return ret;
  }

  std::optional<property> operator()(const vertex &v) const {
    std::shared_lock lock(mutex_);
    auto it = vertices_.find(v);
    if (it == vertices_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  vertices get_vertices() const {
    std::shared_lock lock(mutex_);
    return vertices_;
  }

  edges get_edges(const vertex &v) const {
    std::shared_lock lock(mutex_);
    edges ret;
    std::copy_if(edges_.begin(), edges_.end(), std::inserter(ret, ret.end()), [&v](const edge &e) {
      return e.first == v;
    });
    return ret;
  }

  std::optional<property> get_property(const vertex &v) const {
    std::shared_lock lock(mutex_);
    auto it = vertices_.find(v);
    if (it == vertices_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<weight> get_weight(const edge &e) const {
    std::shared_lock lock(mutex_);
    auto it = weights_.find(e);
    if (it == weights_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<weight> get_weight(const vertex &from, const vertex &to) const {
    std::shared_lock lock(mutex_);
    auto it = weights_.find(edge(from, to));
    if (it == weights_.end()) {
      return std::nullopt;
    }
    return  it->second;
  }

  std::optional<capacity> get_capacity(const edge &e) const {
    std::shared_lock lock(mutex_);
    auto it = capacities_.find(e);
    if (it == capacities_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<capacity> get_capacity(const vertex &from, const vertex &to) const {
    std::shared_lock lock(mutex_);
    auto it = capacities_.find(edge(from, to));
    if (it == capacities_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  void set_property(const vertex &v, property p) {
    std::unique_lock lock(mutex_);
    auto it = vertices_.find(v);
    if (it != vertices_.end()) {
      vertices_[v] = p;
    }
  }

  bool has_vertex(const vertex &v) const {
    std::shared_lock lock(mutex_);
    return vertices_.find(v) != vertices_.end();
  }

  bool has_edge(const vertex &from, const vertex &to) const {
    std::shared_lock lock(mutex_);
    return std::find(edges_.begin(), edges_.end(), edge(from, to)) != edges_.end();
  }

  vertex add_vertex(property p = property()) {
    std::unique_lock lock(mutex_);
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
    std::unique_lock lock(mutex_);
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

 private:
  void add_edge_helper(const vertex &from, const vertex &to, const weight &w) {
    std::unique_lock lock(mutex_);
    auto from_to = edge(from, to);
    edges_.emplace(from_to);
    weights_[from_to] = w;
  }

  void add_edge_helper(const vertex &from, const vertex &to, const capacity &c) {
    std::unique_lock lock(mutex_);
    auto from_to = edge(from, to);
    edges_.emplace(from_to);
    capacities_[from_to] = c;
  }

 public:
  bool remove_edge(const vertex &from, const vertex &to) {
    std::unique_lock lock(mutex_);
    bool removed = remove_helper(from, to);
    if (!directed_ && removed) {
      remove_helper(to, from);
    }
    return removed;
  }

 private:
  bool remove_helper(const vertex &from, const vertex &to) {
    bool removed = false;
    auto from_to = edge(from, to);
    auto it = std::find(edges_.begin(), edges_.end(), from_to);
    if (it != edges_.end()) {
      edges_.erase(it);
      weights_.erase(from_to);
      capacities_.erase(from_to);
      removed = true;
    }
    return removed;
  }

 public:
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
    std::shared_lock lock(mutex_);
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
    std::shared_lock lock(mutex_);
    std::unordered_map<vertex, bool> visited;
    dfs_helper(start, visited, f);
  }

 private:
  void dfs_helper(const vertex &v, std::unordered_map<vertex, bool> &visited, const function &f) {
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

 public:
  void bfs(vertex &start, const function &f) {
    std::shared_lock lock(mutex_);
    auto it = vertices_.find(start);
    if (it == vertices_.end()) {
      return;
    }

    std::unordered_map<vertex, bool> visited;
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

  static std::vector<vertex> path_to(const std::unordered_map<vertex, std::pair<weight, vertex>> &map, const vertex &end) {
    std::vector<vertex> path;
    if (map.find(end) == map.end()) {
      return path;
    }
    for (auto it = end; it != VERTEX_END; it = map.at(it).second) {
      path.emplace_back(it);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::unordered_map<vertex, std::pair<weight, vertex>> dijkstra(const vertex &start) const {
    std::priority_queue<
      std::pair<weight, vertex>,
      std::vector<std::pair<weight, vertex>>,
      std::greater<std::pair<weight, vertex>>
    > pq;
    std::unordered_map<vertex, std::pair<weight, vertex>> res;

    std::shared_lock lock(mutex_);
    for (const auto &pair : vertices_) {
      res[pair.first].first  = WEIGHT_INF;
      res[pair.first].second = VERTEX_END;
    }
    res[start].first = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
      auto [d, u] = pq.top();
      pq.pop();
      if (d > res[u].first) {
        continue;
      }
      auto edges = get_edges(u);
      for (const auto &edge : edges) {
        auto v = edge.second;
        auto w = get_weight(u, v).value();
        auto alt = res[u].first + w;
        if (alt < res[v].first) {
          res[v].first  = alt;
          res[v].second = u;
          pq.emplace(alt, v);
        }
      }
    }
    return res;
  }

  std::unordered_map<vertex, std::pair<weight, vertex>> bellman_ford(const vertex &start) const {
    std::unordered_map<vertex, std::pair<weight, vertex>> res;

    std::shared_lock lock(mutex_);
    for (const auto &pair : vertices_) {
      res[pair.first].first  = WEIGHT_INF;
      res[pair.first].second = VERTEX_END;
    }
    res[start].first = 0;

    for (size_t i = 0; i < vertices_.size(); i++) {
      auto updated = false;
      for (const auto &edge : edges_) {
        auto u = edge.first;
        auto v = edge.second;
        auto w = get_weight(u, v).value();
        auto alt = res[u].first + w;
        if (alt < res[v].first) {
          res[v].first  = alt;
          res[v].second = u;
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

  static std::optional<weight> get_distance(const matrix<weight> &dist, const vertex &from, const vertex &to) {
    if (dist.find(from) == dist.end() ||
        dist.at(from).find(to) == dist.at(from).end()) {
      return std::nullopt;
    }
    return dist.at(from).at(to);
  }

  static std::vector<vertex> get_path(const matrix<vertex> &pred, const vertex &from, const vertex &to) {
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

  std::pair<matrix<weight>, matrix<vertex>> floyd_warshall() const {
    std::pair<matrix<weight>, matrix<vertex>> res;
    std::shared_lock lock(mutex_);
    for (const auto &pair : vertices_) {
      auto u = pair.first;
      for (const auto &edge : get_edges(u)) {
        auto v = edge.second;
        auto w = get_weight(u, v).value();
        res.first[u][v] = w;
        res.second[u][v] = u;
      }
      res.first[u][u] = 0;
    }
    for (const auto &k_pair : vertices_) {
      for (const auto &i_pair : vertices_) {
        for (const auto &j_pair : vertices_) {
          auto k = k_pair.first, i = i_pair.first, j = j_pair.first;
          if (res.first.at(i).find(k) != res.first.at(i).end() &&
              res.first.at(k).find(j) != res.first.at(k).end()) {
            auto alt = res.first.at(i).at(k) + res.first.at(k).at(j);
            if (res.first.at(i).find(j) == res.first.at(i).end() ||
                alt < res.first[i][j]) {
              res.first[i][j] = alt;
              res.second[i][j] = res.second[k][j];
            }
          }
        }
      }
    }
    for (const auto &pair : vertices_) {
      auto u = pair.first;
      if (res.first[u][u] < 0) {
        throw std::runtime_error("negative cycle detected");
      }
    }
    return res;
  }

  graph mst() {
    std::shared_lock lock(mutex_);
    if (vertices_.empty()) {
      return graph();
    }
    auto start = vertices_.begin()->first;
    return mst_helper(start);
  }

  graph mst(const vertex &start) {
    std::shared_lock lock(mutex_);
    if (vertices_.find(start) == vertices_.end()) {
      return graph();
    }
    return mst_helper(start);
  }

 private:
  graph mst_helper(const vertex &start) {
    if (directed_) {
      throw std::runtime_error("mst: directed graph");
    }
    graph g;
    if (empty()) {
      return g;
    }
    std::unordered_map<vertex, bool> visited;
    for (const auto &pair : vertices_) {
      visited[pair.first] = false;
    }
    auto cmp = [](const std::pair<weight, edge>& left,
                  const std::pair<weight, edge>& right) {
      return left.first > right.first;
    };
    std::priority_queue<
      std::pair<weight, edge>,
      std::vector<std::pair<weight, edge>>,
      decltype(cmp)
    > pq(cmp);

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

 public:
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
  capacities gen_residuals() {
    std::shared_lock lock(mutex_);
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

  edges get_edges(const vertex &v, capacities &residuals) {
    edges es;
    for (const auto &e : residuals) {
      if (e.first.first == v) {
        es.emplace(e.first);
      }
    }
    return es;
  }

  path augment(const vertex &s, const vertex &t, capacities &residuals) {
    std::unordered_map<vertex, bool> visited;
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
        auto v = e.second;
        if (!visited[v] && residuals.at(e) > 0) {
          visited[v] = true;
          parent[v]  = u; // Record the path
          queue.push(v);
        }
      }
    }
    return parent;
  }

  flow bottleneck(const vertex &s, const vertex &t, capacities &residuals, path &parent) {
    flow b = std::numeric_limits<flow>::max();
    for (auto v = t; v != s; v = parent.at(v)) {
      auto u = parent.at(v);
      b = std::min(b, residuals.at(edge(u, v)));
    }
    return b;
  }
};
}  // namespace rondo

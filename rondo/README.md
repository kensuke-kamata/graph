# `rondo::graph`

## Member Variables
- `vertices_`: A map of vertices, using vertex IDs as keys to store the properties of each vertex.
- `edges_`: A list of edges, where each edge is represented as a pair of vertices (`std::pair<vertex, vertex>`).
- `weights_`: A map of edge weights, using edges as keys to store the weight of each edge.
- `capacities_`: A map of edge capacities, using edges as keys to store the capacity of each edge.
- `next_`: Manages the next vertex ID to be used.
- `directed_`: Indicates whether the graph is directed.

## Member Functions

### Constructor:
- `graph(bool directed = false)`: Initializes the graph, specifying whether it is directed or undirected.

### Operator Overloads:
- `std::vector<edge> operator[](const vertex &v) const`: Returns the edges that originate from the specified vertex.
- `std::optional<property> operator()(const vertex &v) const`: Returns the property of the specified vertex if it exists.

```c++
rondo::graph g;
auto edges_from_v g[v]
for (const auto &e : edges_from_v) {
    std::cout << "Edge from " << e.first << " to " << e.second << std::endl;
}
```

```c++
rondo::graph g;
auto prop_of_v = g(v)
if (prop_of_v) {
    std::cout << "Vertex label: " << prop->label << ", color: " << prop->color << std::endl;
} else {
    std::cout << "Vertex not found." << std::endl;
}
```

### Vertex Operations:
- `add_vertex(property p = property())`: Adds a new vertex.
- `add_vertex(const vertex &v, property p = property())`: Adds a vertex with the specified ID.
- `has_vertex(const vertex &v) const`: Checks if a vertex with the specified ID exists.

### Edge Operations:
- `add_edge(const edge &e, const weight &w = 1.0)`: Adds a new edge.
- `add_edge(const vertex &from, const vertex &to, const weight &w = 1.0)`: Adds an edge between the specified vertices.
- `has_edge(const vertex &from, const vertex &to) const`: Checks if an edge exists between the specified vertices.
- `remove_edge(const vertex &from, const vertex &to)`: Removes the edge between the specified vertices.

### Property Operations:
- `get_property(const vertex &v) const`: Retrieves the property of the specified vertex.
- `set_property(const vertex &v, property p)`: Sets the property of the specified vertex.

### Input/Output:
- `from_json(const std::string &path)`: Loads the graph from a JSON file.
- `to_json() const`: Exports the graph to JSON format.

## Algorithms

### Depth-First Search (DFS)
```c++
dfs(const vertex &start, const function &f);
```

DFS is a traversal algorithm that explores as far as possible along each branch before backtracking. It is particularly useful for tasks like finding connected components, detecting cycles, and performing topological sorts. The algorithm marks the current vertex `v` as visited and recursively visits all unvisited vertices that are directly connected to the `v`.

```c++
graph g;
// Adding vertices and edges to the graph
// ...
g.dfs(start_vertex, [](const graph::vertex &v) {
    std::cout << "Visited vertex: " << v << std::endl;
});
```

### Breadth-First Search (BFS)
```c++
bfs(vertex &start, const function &f);
```

BFS is a traversal algorithm that explores all vertices at the present depth level before moving on to vertices at the next level. It is particularly useful to finding the shortest path in unweighted graphs, level-order traversal, and for tasks that require visiting nodes layer by layer. The algorithm begins by marking the start vertex as visitged and pushing it onto the queue. It then enters a loop where it processes each vertex in the queue: marking its neighbors as visited, and adding them to the queue for futher processing.

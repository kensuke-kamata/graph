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

### Dijkstra Algorithm
The Dijkstra's algorithm is used to find the shortest path from a starting vertex to all other vertices in a graph. This algorithm is particularly effective for graphs with positive weights. It references all edges in a breadth-first manner while using a priority queue to account for edge weights to find the shortest path.

**Initialization**
- Set the distance to each vertex to infinity (`WEIGHT_INF`), except for the starting vertex (`start`), which is set to 0.
- Create a map to record predecessors (`predecessors`), setting the predecessor of every vertex to a special value (e.g., `VERTEX_END`).

**Priority Queue**
- Use a priority queue (`priority_queue`) to always retrieve the vertex with the shortest distance.
- This queue stores pairs of distance and vertex (`std::pair<weight, vertex>`).

**Main Loop**
- Extract vertices from the queue and examine all edges extending from those vertices.
- If a new distance is shorter than the known distance, update the distance and predecessor, then add the new distance and vertex to the queue.

**Termination Condition**
- Once the queue is empty, the shortest paths to all vertices are determined.

**Computational Complexity**
- The initial loop to construct the priority queue performs insertions $V$ times, resulting in a complexity of $O(V \log V)$.
- The subsequent `while` loop visits each edge once and is called at most $E$ times, resulting in a complexity of $O(E \log V)$.
- The overall complexity is $O((V + E) \log V)$.

### Bellman-Ford Algorithm
The Bellman-Ford algorithm is used to find the shortest paths from a single source vertex to all other vertices in a weighted graph. It can handle graphs with negative weights and can detect negative weight cycles.

**Initialization**
- Set the distance to each vertex to infinity (`WEIGHT_INF`), except for the starting vertex (`start`), which is set to 0.
- Set the predecessor of each vertex to a special value (`VERTEX_END`).

**Relaxation**
- Iterate over all edges $|V| - 1$ times, where $|V|$ is the number of vertices.
- Update the distance to the destination vertex for each edge if a shorter path is found.
- Terminate early if no updates occur in an iteration, indicating that the shortest paths have been found.

> The shortest path in a graph consists of at most $|V| - 1$ edges, provided the path does not exceed the total number of vertices $|V|$. This is true for simple paths (paths without cycles). The distance to each vertex on the shortest path can be accurately updated with at most $|V| - 1$ edge relaxation operations.

**Negative Cycle Detection**
- After `|V| - 1` iterations, if any distance can still be updated, a negative weight cycle exists.

**Computational Complexity**
- Nested loops result in $O(V*E)$
- Slower than Dijkstra's algorithm, which is ($O((V + E) \log V)$)

### Floyd-Warshall Algorithm
The Floyd-Warshall algorithm finds the shortest paths between all pairs of vertices in a weighted graph, which may include negative weights but must not contain negative weight cycles. It computes the shortest distances and paths between all vertices simultaneously, making it suitable for dense graphs. It is ideal in scenarios where pre-computed shortest paths are needed for all locations.

> Implementing single-source shortest path algorithms (like Dijkstra's or Bellman-Ford) for all vertices can yield results similar to the Floyd-Warshall algorithm.

Floyd-Warshall is a type of dynamic programming algorithm, memorizing the (computed) shortest distances and predecessors for each pair of vertices in two matrices.

**Initialization**
- Initialize the distance matrix `dist`:
  - `dist[i][i]` is set to 0 (self-loop).
  - `dist[i][j]` is set to the weight of edge `(i, j)`.
  - If no edge exists, set it to a large value like infinity.
- Initialize the predecessor matrix `pred`:
  - `pred[i][j]` is set to the vertex `i`.

**Dynamic Programming**
- Use three nested loops to test all vertex pairs `i, j` with each intermediate vertex `k` one by one.
- Check if `dist[i][j]` can be updated via `dist[i][k] + dist[k][j]`.
- If an update is possible, set `dist[i][j]` to the new distance and update `pred[i][j]` to `pred[k][j]`.

**Negative Cycle Detection**
- If any diagonal element `dist[i][i]` becomes negative, a negative weight cycle exists.

**Computational Complexity**
- The three nested loops result in a complexity of $O(V^3)$.

### Prim's Algorithm

Prim's Algorithm is designed to find the minimum spanning tree (MST) in a connected, undirected graph using a greedy approach without reversing previous decisions. The MST is a subset of the edges that connects all vertices with the minimum possible total edge weight, without forming any cycles. The algorithm incrementally adds edges to the spanning tree T until the MST is complete.

**Initialization**
- Start with a selected vertex (arbitrarily chosen if not specified).
- Initialize a priority queue to hold the edges, sorted by their weights.
- Mark the selected vertex as visited and add all its edges to the priority queue.

**Process**
While the priority queue is not empty:
- Remove the edge with the smallest weight from the priority queue.
- If the edge connects to an unvisited vertex:
  - Add the vertex to the MST and mark it as visited.
  - Add the vertex to the growing spanning tree.
  - Insert all new edges from this vertex into the priority queue, considering only those that connect to unvisited vertices.

**Edge Selection**
The algorithm selects edges in increasing order of weight, ensuring that no cycles are formed. This is guaranteed by only considering edges that connect to unvisited vertices.

**Computational Complexity**
- Using a priority queue leads to a complexity of $O((V+E)\log V)$, where V is the number of vertices and E is the number of edges.

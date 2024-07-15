# `rondo::graph`

The Rondo Graph Library, `rondo::graph`, is a header-only C++ library for creating and manipulating graphs. It leverages modern C++17 features and the popular `nlohmann::json` library for JSON serialization and deserialization of graph structures.

## Installation

Since Rondo Graph Library is header-only, it can be included directly into C++ projects. Simply add the `graph.hpp` file to your project's include path.

To integrate Rondo Graph Library into your CMake project, add the following to your `CMakeLists.txt`:

```cmake
add_subdirectory(path_to_rondo_graph)
target_link_libraries(your_target_name INTERFACE rondo_graph)
```

Replace `path_to_rondo_graph` with the actual path to the Rondo Graph Library in your project structure.

## Usage

Here's a basic example of using the Rondo Graph Library:

```cpp
#include <rondo/graph.hpp>

int main() {
  rondo::graph my_graph;
  auto v1 = my_graph.add_vertex({"Label1", "Blue"});
  auto v2 = my_graph.add_vertex({"Label2", "Red"});
  my_graph.add_edge(v1, v2, 1.0);

  // Your graph logic here

  return 0;
}
```

## Testing

To enable and run the unit tests, configure the build with the `BUILD_RONDO_GRAPH_TEST` option set to `ON`. Follow these steps to compile and execute the tests:

```bash
mkdir build
cd build
cmake .. -D BUILD_RONDO_GRAPH_TEST=ON
make
ctest
```

## Acknowledgements

- Thanks to the developers of the `nlohmann::json` library for providing an excellent JSON handling library for modern C++.
- This project uses GoogleTest for unit testing.

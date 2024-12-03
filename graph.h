#pragma once
// Implementation of various graph algorithms

#include <iostream>
#include <vector>
#include <utility>
#include <limits>
#include <queue>

using namespace std;

const int INFINITY = numeric_limits<int>::max();


/*******************************************************************************************
Graph Representation:

    The graph is represented using an adjacency list (adjacency_list), 
    which is a vector of vector of pair<int, int>.
    Each pair<int, int> represents a connected vertex and the weight of the edge.

Adding Edges:
    The addEdge method adds an edge from from_vertex to to_vertex with the given weight.
    If the graph is undirected it also adds the edge from to_vertex to from_vertex.
********************************************************************************************/
class Graph
{
public:
    Graph(int num_vertices) : num_vertices(num_vertices), adjacency_list(num_vertices) {}

    void addEdge(int from_vertex, int to_vertex, int weight, bool undirected = true)
    {
        adjacency_list[from_vertex].push_back(make_pair(to_vertex, weight));
        if (undirected)
        {
            adjacency_list[to_vertex].push_back(make_pair(from_vertex, weight));
        }
    }


/*
Djikstra's algorithm
The algorithm uses a simple linear search to find the unvisited vertex with the smallest tentative distance in each iteration.

Algorithm:

Initialization:
    A distances vector is initialized with INFINITY for all vertices.
    A visited vector is used to track whether a vertex has been processed.
    The distance to the source_vertex is set to 0.
Main Loop:
    For each vertex in the graph:
        Find the Unvisited Vertex with the Smallest Distance:
        Iterate over all vertices to find the unvisited vertex with the minimum tentative distance (min_distance).
        Termination Condition:
        If no such vertex is found (i.e., current_vertex == -1), all reachable vertices have been visited, and the algorithm terminates.
        Mark the Vertex as Visited:
        The selected current_vertex is marked as visited.
        Update Distances:
            For each neighbor of current_vertex, update the distance if a shorter path is found.
            Only unvisited neighbors are considered.
*/

    vector<int> dijkstra(int source_vertex) const
    {
        vector<int> distances(num_vertices, INFINITY);
        vector<bool> visited(num_vertices, false);
        distances[source_vertex] = 0;

        for (int count = 0; count < num_vertices; ++count)
        {
            int min_distance = INFINITY;
            int current_vertex = -1;

            // Find the unvisited vertex with the smallest distance
            for (int vertex = 0; vertex < num_vertices; ++vertex)
            {
                if (!visited[vertex] && distances[vertex] <= min_distance)
                {
                    min_distance = distances[vertex];
                    current_vertex = vertex;
                }
            }

            if (current_vertex == -1)
            {
                break; // All reachable vertices have been visited
            }

            visited[current_vertex] = true;

            // Update distances of adjacent vertices
            for (size_t edge_index = 0; edge_index < adjacency_list[current_vertex].size(); ++edge_index)
            {
                int neighbor_vertex = adjacency_list[current_vertex][edge_index].first;
                int edge_weight = adjacency_list[current_vertex][edge_index].second;

                if (!visited[neighbor_vertex] && distances[neighbor_vertex] > distances[current_vertex] + edge_weight)
                {
                    distances[neighbor_vertex] = distances[current_vertex] + edge_weight;
                }
            }
        }

        return distances;
    }

/*
Prim's MST
    in_mst Vector - Keeps track of whether a vertex has been included in the MST.
    Priority Queue - Used to select the edge with the smallest weight connecting the MST to a new vertex.
    Key Values - The key array stores the minimum weight edge that connects each vertex to the MST.
Algorithm Flow:
    Start from the start_vertex.
    At each step, pick the smallest edge connecting a vertex in the MST to a vertex outside the MST.
    Update the key values of adjacent vertices if a smaller edge is found.
    Continue until all vertices are included in the MST.
*/
    int MinimumSpanningTreePrim(int start_vertex)
    {
        vector<bool> in_mst(num_vertices, false);
        vector<int> key(num_vertices, INFINITY);
        int total_cost = 0;

        // Priority queue to hold pairs of (cost, vertex)
        priority_queue<pair<int, int>,vector<pair<int, int>>,greater<pair<int, int>>> priority_queue;

        // Start from the start_vertex with cost 0
        priority_queue.push(make_pair(0, start_vertex));
        key[start_vertex] = 0;

        while (!priority_queue.empty())
        {
            pair<int, int> item = priority_queue.top();
            priority_queue.pop();

            int current_vertex = item.second;

            // If the vertex is already included in MST, skip it
            if (in_mst[current_vertex])
            {
                continue;
            }

            // Include the vertex in MST
            in_mst[current_vertex] = true;
            total_cost += item.first;

            // Iterate over adjacent vertices
            for (size_t edge_index = 0; edge_index < adjacency_list[current_vertex].size(); ++edge_index)
            {
                int neighbor_vertex = adjacency_list[current_vertex][edge_index].first;
                int edge_weight = adjacency_list[current_vertex][edge_index].second;

                if (!in_mst[neighbor_vertex] && key[neighbor_vertex] > edge_weight)
                {
                    key[neighbor_vertex] = edge_weight;
                    priority_queue.push(make_pair(edge_weight, neighbor_vertex));
                }
            }
        }

        return total_cost;
    }
private:
    int num_vertices; // Number of vertices
    vector<vector<pair<int, int>>> adjacency_list; // Adjacency list
};


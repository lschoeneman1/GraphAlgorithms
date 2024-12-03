//******************************************************************** 
// Overview:  Read a graph from a file, and compute shortest path using 
//            djikstra's algorithm
// Author: Larry Schoeneman
//******************************************************************** 

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <limits>
#include "graph.h"

int main()
{
    int num_vertices, num_edges; // Number of vertices and edges
    string filename;

    cout << "Enter the filename containing the graph details: ";
    cin >> filename;

    ifstream input_file(filename);
    if (!input_file)
    {
        cerr << "Error: Unable to open file " << filename << endl;
        return 1;
    }

    input_file >> num_vertices >> num_edges;
    Graph graph(num_vertices);

    // Read edges from the file
    for (int edge_index = 0; edge_index < num_edges; ++edge_index)
    {
        int from_vertex, to_vertex, weight; // Edge from from_vertex to to_vertex with weight
        input_file >> from_vertex >> to_vertex >> weight;
        // Assuming 0-based indexing for vertices
        graph.addEdge(from_vertex, to_vertex, weight);
    }

    int source_vertex;
    input_file >> source_vertex;

    input_file.close();

    vector<int> distances = graph.dijkstra(source_vertex);

    // Output distances
    for (int vertex_index = 0; vertex_index < num_vertices; ++vertex_index)
    {
        cout << "Distance from " << source_vertex << " to " << vertex_index << " is ";
        if (distances[vertex_index] == INFINITY)
        {
            cout << "INF" << endl;
        }
        else
        {
            cout << distances[vertex_index] << endl;
        }
    }

    return 0;
}
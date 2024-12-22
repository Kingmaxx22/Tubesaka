#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <climits>
#include <map>
using namespace std;

// Graph representation using adjacency list
class Graph {
public:
    // Adjacency list to store graph edges and weights
    map<string, vector<pair<string, int>>> adj;

    // Function to add an edge between two nodes with a given weight
    void addEdge(string u, string v, int weight) {
        adj[u].push_back({v, weight});
        adj[v].push_back({u, weight}); // Assuming undirected graph
    }

    // Function to print the adjacency list of the graph
    void printGraph() {
        for (auto &[key, neighbors] : adj) {
            cout << key << ": ";
            for (auto &[neighbor, weight] : neighbors) {
                cout << "(" << neighbor << ", " << weight << ") ";
            }
            cout << endl;
        }
    }
};

// DFS Iterative to find path between start and goal nodes
void DFSIterative(Graph &graph, string start, string goal) {
    stack<string> s; // Stack to manage traversal order
    map<string, bool> visited; // Map to track visited nodes
    map<string, string> parent; // Map to reconstruct the path

    s.push(start); // Start the DFS from the start node
    visited[start] = true; // Mark the start node as visited

    while (!s.empty()) {
        string node = s.top(); // Get the top node from the stack
        s.pop();

        // Check if we have reached the goal node
        if (node == goal) {
            cout << "Path found using DFS: ";
            string current = goal;
            // Backtrack to reconstruct the path
            while (current != start) {
                cout << current << " <- ";
                current = parent[current];
            }
            cout << start << endl;
            return;
        }

        // Explore all neighbors of the current node
        for (auto &[neighbor, weight] : graph.adj[node]) {
            if (!visited[neighbor]) { // If neighbor is not visited
                s.push(neighbor); // Push it onto the stack
                visited[neighbor] = true; // Mark as visited
                parent[neighbor] = node; // Set the parent for path reconstruction
            }
        }
    }

    // If no path is found
    cout << "No path found using DFS." << endl;
}

// Helper function for recursive Dijkstra's algorithm
void DijkstraRecursive(map<string, vector<pair<string, int>>> &adj, map<string, int> &dist, map<string, bool> &visited, string current) {
    visited[current] = true; // Mark the current node as visited

    // Update distances to neighbors
    for (auto &[neighbor, weight] : adj[current]) {
        if (dist[current] + weight < dist[neighbor]) {
            dist[neighbor] = dist[current] + weight; // Update the distance if shorter path is found
        }
    }

    // Find the next node with the smallest distance that is not visited
    string nextNode = "";
    int minDist = INT_MAX;

    for (auto &[node, d] : dist) {
        if (!visited[node] && d < minDist) {
            minDist = d;
            nextNode = node;
        }
    }

    // Recur for the next node if it exists
    if (!nextNode.empty()) {
        DijkstraRecursive(adj, dist, visited, nextNode);
    }
}

// Dijkstra's algorithm to find the shortest path between start and goal nodes
void Dijkstra(Graph &graph, string start, string goal) {
    map<string, int> dist; // Map to store shortest distances from start
    map<string, bool> visited; // Map to track visited nodes

    // Initialize distances and visited status
    for (auto &[node, _] : graph.adj) {
        dist[node] = INT_MAX; // Set all distances to infinity
        visited[node] = false; // Mark all nodes as not visited
    }

    dist[start] = 0; // Distance to the start node is 0

    // Start the recursive Dijkstra algorithm
    DijkstraRecursive(graph.adj, dist, visited, start);

    // Output the shortest path distance to the goal node
    if (dist[goal] != INT_MAX) {
        cout << "Shortest path using Dijkstra: " << dist[goal] << endl;
    } else {
        cout << "No path found using Dijkstra." << endl;
    }
}

int main() {
    Graph graph;

    // Sample dataset representing roads around Telkom Bandung
    graph.addEdge("Telkom", "ITB", 5);
    graph.addEdge("Telkom", "UNPAD", 10);
    graph.addEdge("Telkom", "Gedung Sate", 8);
    graph.addEdge("ITB", "Gedung Sate", 3);
    graph.addEdge("Gedung Sate", "UNPAD", 2);

    // Adding 7 new edges to the graph
    graph.addEdge("Telkom", "Cihampelas", 4);
    graph.addEdge("Cihampelas", "Dago", 6);
    graph.addEdge("Dago", "ITB", 2);
    graph.addEdge("Cihampelas", "Gedung Sate", 7);
    graph.addEdge("Gedung Sate", "Braga", 5);
    graph.addEdge("Braga", "Telkom", 9);
    graph.addEdge("Dago", "Braga", 8);

    // Print the graph to visualize the connections
    graph.printGraph();

    string start = "Telkom";
    string goal = "UNPAD";

    // Perform DFS to find a path
    DFSIterative(graph, start, goal);
    // Perform Dijkstra's algorithm to find the shortest path
    Dijkstra(graph, start, goal);

    return 0;
}

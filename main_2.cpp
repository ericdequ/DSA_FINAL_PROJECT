#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <forward_list>
#include <climits>
using namespace std;

/*
NOTES for team members:
 - Create interface/menu in main
 - Add way to stop algorithms (maybe add end node in parameter?)
 - Make graph with data in main
    - Add more information of each node (latitude and longitude)
*/

class NodeList {
public:
    // Node Object
    struct Node {
        pair<int, int> coords; // Store latitude and longitude
        int nodeID; // Store nodeID;
        
        // Constructor
        Node(int latitude, int longitude, int id) : coords(latitude, longitude), nodeID(id) {}
    };
    
    // Methods
    void addNode(int lati, int longi, int id);
    
    // Vector Storing Information
    vector<Node*> listNodes;
};

// Add node to the vector
void NodeList::addNode(int lati, int longi, int id) {
    Node* tempNode = new Node(lati, longi, id);
    listNodes.push_back(tempNode);
}

// Adjacency List
class GraphL {
private:
    vector<forward_list<pair<int, int>>> adjList; // Outer vector: from, Inner vector: pair of to and weight
    int numVertices = 0;
public:
    // Methods
    void insertEdge(int from, int to, int weight); // Add edges to graph
    bool isEdge(int from, int to); // Check if edge exists
    int getWeight(int from, int to); // Get weight of edge
    vector<int> getAdjacent(int vertex); // Get list of vertices connected
    //Graph Algorithms
    vector<int> shortestPath(int src);
    void bfs(int src);
};

//Note: Based on the way our data is stored, we need a way to stop this bfs or it will continue
//forever until we are out of data points. For instance, we should add a parameter that stores 
//what type of place the user is looking for, ie: food, library etc, and then stop once we have found
//one or two nodes that fall under that category.
//TODO: Update this function.
void GraphL::bfs(int src) {
    set<int> visited;
    queue<int> q;
    visited.insert(src);
    q.push(src);

    while (!q.empty()) {
        int u = q.front();
        cout << u << " ";
        q.pop();
        forward_list<pair<int,int>> neighbors = adjList[u];
        for (pair<int,int> v : neighbors) {
            if (visited.count(v.first) == 0) {
                visited.insert(v.first);
                q.push(v.first);
            }
        }
    }
}

vector<int> GraphL::shortestPath(int src) {
    vector<int> distances(numVertices, INT_MAX);
    vector<bool> visited(numVertices, false);
    priority_queue<pair<int, int>, vector<pair<int, int> >, greater<pair<int, int> > > pq;
    distances[src] = 0;
    pq.push(make_pair(0, src));

    distances[src] = 0;
    pq.push(make_pair(0, src));
    while (!pq.empty())
    {
        pair<int, int> top = pq.top();
        pq.pop();
        int dist = top.first;
        int node = top.second;
        if (dist > distances[node])
            continue;
        else {
            forward_list<pair<int, int>> list = adjList[top.first];
            for (auto it = list.begin(); it != list.end(); it++) {
                pair<int, int> v = *it;
                if (distances[v.first] > distances[node] + v.second) {
                    distances[v.first] = distances[node] + v.second;
                    pq.push(make_pair(distances[v.first], v.first));
                }
            }
        }
    }
    return distances;
}


void GraphL::insertEdge(int from, int to, int weight) {
    // Check if graph needs to be resized for new from vertex
    if (adjList.size() <= from)
        adjList.resize(from + 1);

    if (adjList[from].empty()) { // If no edges found at from vertex
        pair<int, int> tempP(to, weight);
        adjList[from].push_front(tempP);
        numVertices++;
    }
    else { // If list is not empty (Edges are present)
        auto it = adjList[from].begin();
        for (it; it != adjList[from].end(); it++) {
            if ((*it).first == to) // If there is already an edge
                break;
        }
        if (it == adjList[from].end()) { // If no edge to the to vertex found
            pair<int, int> tempP(to, weight);
            adjList[from].push_front(tempP);
            numVertices++;
        }
    }
}

bool GraphL::isEdge(int from, int to) {
    if (adjList.size() <= from) // If graph is too small
        return false;

    for (auto it = adjList[from].begin(); it != adjList[from].end(); ++it) {
        if ((*it).first == to) // If there exists an edge
            return true;
    }

    return false;
}

int GraphL::getWeight(int from, int to) {
    if (adjList.size() <= from) // If graph is too small
        return 0;

    for (auto it = adjList[from].begin(); it != adjList[from].end(); ++it) {
        if ((*it).first == to) // If there exists an edge
            return (*it).second;
    }

    return 0;
}

vector<int> GraphL::getAdjacent(int vertex) {
    vector<int> returnVec; // Create vector to return
    if (adjList.size() <= vertex) // If graph is too small
        return returnVec;

    // Iterate through all edges of vertex
    for (auto it = adjList[vertex].begin(); it != adjList[vertex].end(); ++it)
        returnVec.push_back((*it).first);

    return returnVec; // Return the vector of vertices
}

// Adjacency Matrix
class GraphM {
private:
    vector<vector<int>> adjList; // Outer vector: from, Inner vector: weight at edge
    int numVertices = 0;
public:
    // Methods
    void insertEdge(int from, int to, int weight); // Add edges to graph
    bool isEdge(int from, int to); // Check if edge exists
    int getWeight(int from, int to); // Get weight of edge
    vector<int> getAdjacent(int vertex); // Get list of vertices connected
    //Algorithms
    vector<int> shortestPath(int src);
    int minDistance(vector<int>& dist, vector<bool>& visited); 
    void bfs(int src);

};

//Note: Based on the way our data is stored, we need a way to stop this bfs or it will continue
//forever until we are out of data points. For instance, we should add a parameter that stores 
//what type of place the user is looking for, ie: food, library etc, and then stop once we have found
//one or two nodes that fall under that category.
//TODO: Update this function.
void GraphM::bfs(int src) {
    set<int> visited;
    queue<int> q;
    visited.insert(src);
    q.push(src);
   
    while (!q.empty()) {
        int u = q.front();
        cout << u << " ";
        q.pop();
        vector<int> neighbors = adjList[u];
        for (int v : neighbors) {
            if (visited.count(v) == 0) {
                visited.insert(v);
                q.push(v);
            }
        }
    }
}

int GraphM::minDistance(vector<int>& dist, vector<bool>& visited) {
    int min = INT_MAX;
    int minIndex = 0;
    for (int i = 0; i < dist.size(); i++) {
        if (visited[i] == false && dist[i] <= min) {
            min = dist[i];
            minIndex = i;
        }
    }
    return minIndex;
}

//Code adapted from: https://www.geeksforgeeks.org/c-program-for-dijkstras-shortest-path-algorithm-greedy-algo-7/
vector<int> GraphM::shortestPath(int src) {
    vector<int> dist(numVertices, INT_MAX);
    vector<bool> visited(numVertices, false);
    dist[src] = 0;

    for (int i = 0; i < adjList[src].size(); i++) {
        int u = minDistance(dist, visited);
        visited[u] = true;

        for (int v = 0; v < adjList[src].size(); i ++) {
            if (!visited[v] && adjList[u][v] && dist[u] != INT_MAX && dist[u] + adjList[u][v] < dist[v])
                dist[v] = dist[u] + adjList[u][v];
        }
    }
    return dist;
}


void GraphM::insertEdge(int from, int to, int weight) {
    // Check if graph needs to be resized for new from vertex
    if (adjList.size() <= from)
        adjList.resize(from + 1);

    if (adjList[from].empty()) { // If no edges found at from vertex
        adjList[from].resize(to + 1);
        adjList[from][to] = weight; // Insert edge
        numVertices++;
    }
    else { // If list is not empty (Edges are present)
        auto it = adjList[from].begin();
        int counter = 0;
        for (it; it != adjList[from].end(); ++it) {
            if (*it != 0 && counter == to) // If there is already an edge
                break;
            counter++;
        }
        if (it == adjList[from].end()) { // If no edge to the to vertex found
            // Check if graph needs to be resized for new to vertex
            if (adjList[from].size() <= to)
                adjList[from].resize(to + 1);

            adjList[from][to] = weight; // Insert edge
            numVertices++;
        }
    }
}

bool GraphM::isEdge(int from, int to) {
    if (adjList.size() <= from) // If graph is too small
        return false;

    if (adjList[from].size() <= to) // If from vertex's vector is too small
        return false;

    return true;
}

int GraphM::getWeight(int from, int to) {
    if (adjList.size() <= from) // If graph is too small
        return 0;

    if (adjList[from].size() <= to) // If from vertex's vector is too small
        return 0;

    return adjList[from][to];
}

vector<int> GraphM::getAdjacent(int vertex) {
    vector<int> returnVec; // Create vector to return
    if (adjList.size() <= vertex) // If graph is too small
        return returnVec;

    // Iterate through all edges of vertex
    for (int i = 0; i < adjList[vertex].size(); ++i) {
        if (adjList[vertex][i] != 0)
            returnVec.push_back(i);
    }

    return returnVec; // Return the vector of vertices
}


int main() {
    // Test Cases for above methods
    /*
    GraphL testGraphL;
    testGraphL.insertEdge(0, 1, 15);
    testGraphL.insertEdge(0, 1, 10); // Test duplicate addition
    testGraphL.insertEdge(0, 2, 25);
    if (testGraphL.isEdge(0, 1) == true) // Check if edge exists
        cout << "true" << endl;
    if (testGraphL.isEdge(0, 3) == false) // Check if edge doesn't exist
        cout << "false" << endl;
    cout << testGraphL.getWeight(0, 1) << endl; // Check for valid weight
    cout << testGraphL.getWeight(99, 99) << endl; // Check for invalid weight

    GraphM testGraphM;
    testGraphM.insertEdge(0, 1, 15);
    testGraphM.insertEdge(0, 1, 10); // Test duplicate addition
    testGraphM.insertEdge(0, 2, 25);
    if (testGraphM.isEdge(0, 2) == true) // Check if edge exists
        cout << "true" << endl;
    if (testGraphM.isEdge(0, 3) == false) // Check if edge doesn't exist
        cout << "false" << endl;
    cout << testGraphM.getWeight(0, 2) << endl; // Check for valid weight
    cout << testGraphM.getWeight(99, 99) << endl; // Check for invalid weight
    */

    return 0;
}
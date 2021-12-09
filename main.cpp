#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <forward_list>
#include <climits>
#include<sstream>
#include <fstream>
#include <chrono>
#pragma once
using namespace std;

struct EdgeData {
    string to = 0;
    string from = 0;
    string weight = 0;
};

struct NodeData {
    int id = 0;
    int OSID = 0;
    double latitude = 0.000000;
    double longitude = 0.000000;
};

void getEdgeDataAfterConstruction(vector<vector<string>>& allEdges) {
    std::fstream file("EdgeData_Construction.txt");
    std::string line;
    if (file) {
        bool b = true;
        getline(file, line);
        while (getline(file, line)) {
            vector<string> allparts;
            int pos = 0;
            while ((pos = line.find(',')) != string::npos) {
                allparts.push_back(line.substr(0, pos));
                line.erase(0, pos + 1);
            }

            allparts.push_back(line);
            allEdges.push_back(allparts);
        }
        file.close();
    }
    else {
        cout << "File couldnt open" << endl;
    }
}

void nodeDataAfterConstruction(vector<vector<string>>& allnodes) { // built as seperate file so adding extra node data only requires changing this function and node class
    std::fstream file("NodeData_Construction.txt");
    std::string line;
    if (file) {
        getline(file, line);
        while (getline(file, line)) {
            vector<string> allparts;
            int pos = 0;
            while ((pos = line.find(',')) != string::npos) {
                allparts.push_back(line.substr(0, pos));
                line.erase(0, pos + 1);
            }
            allparts.push_back(line);
            allnodes.push_back(allparts);
        }
        file.close();
    }
    else {
        cout << "File couldnt open" << endl;
    }
}


void getEdgeDataBeforeConstruction(vector<vector<string>>& allEdges) {
    std::fstream file("AllEdges.csv");
    std::string line;
    if (file) {
        bool b = true;
        getline(file, line);
        while (getline(file, line)) {
            vector<string> allparts;
            int pos = 0;
            while ((pos = line.find(',')) != string::npos) {
                allparts.push_back(line.substr(0, pos));
                line.erase(0, pos + 1);
            }


            allparts.push_back(line);
            allEdges.push_back(allparts);
        }
        file.close();
    }
    else {
        cout << "File couldnt open" << endl;
    }
}

void nodeDataBeforeConstruction(vector<vector<string>>& allnodes) { // built as seperate file so adding extra node data only requires changing this function and node class
    std::fstream file("AllNodes.csv");
    std::string line;
    if (file) {
        getline(file, line);
        while (getline(file, line)) {
            vector<string> allparts;
            int pos = 0;
            while ((pos = line.find(',')) != string::npos) {
                allparts.push_back(line.substr(0, pos));
                line.erase(0, pos + 1);
            }
            allparts.push_back(line);
            allnodes.push_back(allparts);
        }
        file.close();
    }
    else {
        cout << "File couldnt open" << endl;
    }
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
    void bfs(int src, int destination);
};

void GraphL::bfs(int src, int destination) {
    set<int> visited;
    queue<int> q;
    visited.insert(src);
    q.push(src);

    while (!q.empty()) {
        int u = q.front();
        cout << u << " ";
        if (u == destination)
        {
            cout << endl;
            break;
        }
        q.pop();
        forward_list<pair<int, int>> neighbors = adjList[u];
        for (pair<int, int> v : neighbors) {
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
    while (!pq.empty())
    {
        pair<int, int> top = pq.top();
        pq.pop();
        int dist = top.first;
        int node = top.second;
        if (dist > distances[node])
            continue;
        else {
            forward_list<pair<int, int>> list = adjList[node];
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
    vector<int> shortestPath2(int src);
    int minDistance(vector<int>& dist, vector<bool>& visited);
    void bfs(int src);

};

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
        for (int v = 0; v < adjList[src].size(); v++) {
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

    cout << "Reading in geolocation data for UF campus with and without construction" << endl;
    cout << "......................................................................." << endl;

    vector<vector<string>> allEdgesBefore;
    vector<vector<string>> allnodesBefore;
    getEdgeDataBeforeConstruction(allEdgesBefore);
    cout << "Amount of edges before accounting for construction on campus: " << allEdgesBefore.size() << endl;
    nodeDataBeforeConstruction(allnodesBefore);
    cout << "Amount of nodes before accounting for construction on campus: " << allnodesBefore.size() << endl << endl;
    cout << endl;

    vector<vector<string>> allEdgesAfter;
    vector<vector<string>> allnodesAfter;
    getEdgeDataAfterConstruction(allEdgesAfter);
    cout << "Amount of edges after accounting for construction on campus: " << allEdgesAfter.size() << endl;
    nodeDataAfterConstruction(allnodesAfter);
    cout << "Amount of nodes after accounting for construction on campus: " << allnodesAfter.size() << endl;
    cout << endl;

    cout << "Total edges Removed: " << (allEdgesBefore.size() - allEdgesAfter.size()) << endl;
    cout << "Total nodes Removed: " << (allnodesBefore.size() - allnodesAfter.size()) << endl;
    cout << endl;


    GraphL adjacenyList_dense;
    GraphM adjacenyMatrix_dense;
    cout << "Building a dense graph (no construction) with adjaceny list and matrix representation" << endl;
    for (int i = 0; i < allEdgesBefore.size(); i++)
    {
        vector<string> graphData = allEdgesBefore[i];
        adjacenyList_dense.insertEdge(stoi(graphData[1]), stoi(graphData[2]), stoi(graphData[0]));
        adjacenyMatrix_dense.insertEdge(stoi(graphData[1]), stoi(graphData[2]), stoi(graphData[0]));
    }
    cout << "Finished building dense graphs." << endl; 


    GraphL adjacenyList_sparse;
    GraphM adjacenyMatrix_sparse;
    cout << "Building a sparse graph (with construction) with adjaceny list and matrix representation" << endl;
    for (int i = 0; i < allEdgesAfter.size(); i++)
    {
        vector<string> graphData = allEdgesAfter[i];
        adjacenyList_sparse.insertEdge(stoi(graphData[0]), stoi(graphData[1]), stoi(graphData[2]));
        adjacenyMatrix_sparse.insertEdge(stoi(graphData[0]), stoi(graphData[1]), stoi(graphData[2]));
    }
    cout << "Finished building sparse graphs." << endl;

    cout << "Please choose your starting and ending point (Press Enter After Each)." << endl;
    int src;
    int end;
    cin >> src;
    cin >> end;

    auto start_time = std::chrono::steady_clock::now();
    vector<int> dist = adjacenyList_dense.shortestPath(src);
    auto end_time = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout << "Your shortest path with construction from " << src << " to " << end << " is " << dist[end] << " meters."<< endl;
    cout << "Computing your shortest without any construction took " << ms.count() << " miliseconds for the adjaceny list." << endl;


    start_time = std::chrono::steady_clock::now();
    vector<int> dist2 = adjacenyMatrix_dense.shortestPath(src);
    end_time = std::chrono::steady_clock::now();
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout << "Your shortest path with construction from " << src << " to " << end << " is " << dist2[end] << " meters." << endl;
    cout << "Computing your shortest path with construction took " << ms.count() << " miliseconds for the adjaceny matrix." << endl; 

    start_time = std::chrono::steady_clock::now();
    vector<int> dist3 = adjacenyList_sparse.shortestPath(src);
    end_time = std::chrono::steady_clock::now();
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout << "Your shortest path without construction from " << src << " to " << end << " is " << dist3[end] << " meters." << endl;
    cout << "Computing your shortest path with construction took " << ms.count() << " miliseconds for the adjaceny list." << endl;

    start_time = std::chrono::steady_clock::now();
    vector<int> dist4 = adjacenyMatrix_sparse.shortestPath(src);
    end_time = std::chrono::steady_clock::now();
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout << "Your shortest path without construction from " << src << " to " << end << " is " << dist4[end] << " meters." << endl;
    cout << "Computing your shortest path with construction took " << ms.count() << " miliseconds for the adjaceny matrix." << endl;   
    return 0;
}

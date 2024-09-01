/*
* Dhaval Atul Patel - Programming Assignment 3
*/
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>



using namespace std;

/************************************   DSP Algorithm Class  *********************************************/
class Graph {
public:
    
    map<string, map<string, int>> adjList; // (Map) -> (Key, Map) -> (Neighbor - weight - Neighbor)

    void addEdge(const string& u, const string& v, int weight) {
        adjList[u][v] = weight;
        adjList[v][u] = weight; 
    }

    void printDA(map<string, string> parent)
    {
        string current1 = "MountDoom";
        vector<string> path1;
        cout << "Shortest path from Hobbiton to MountDoom (Dijkstra's algorithm): ";
        while (current1 != "Hobbiton") {
            path1.push_back(current1);
            current1 = parent.at(current1);
        }
        path1.push_back("Hobbiton");
        for(int i=path1.size()-1; i>=0; i--)
        {
            cout << path1[i];
            if(i!=0)
            {
                cout << " -> ";
            }
        }

        cout << endl << endl;
    }
    
    map<string, int> dijkstra(const string& startVertex) {
        map<string, int> distance;
        map<string, string> parent;
        priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> pq;

        
        for (const auto& pair : adjList) {
            distance[pair.first] = numeric_limits<int>::max();
        }

        
        distance[startVertex] = 0;

       
        pq.push({0, startVertex});

        while (!pq.empty()) {
            string currentVertex = pq.top().second;
            int currentDistance = pq.top().first;
            pq.pop();

            
            if (currentDistance > distance[currentVertex]) {
                continue;
            }

            
            for (const auto& neighbor : adjList[currentVertex]) {
                string nextVertex = neighbor.first;
                int weight = neighbor.second;
                int newDistance = currentDistance + weight;

                
                if (newDistance < distance[nextVertex]) {
                    distance[nextVertex] = newDistance;
                    parent[nextVertex] = currentVertex;
                    pq.push({newDistance, nextVertex});
                }
            }
        }

        /*********************  Prints Dijkstra's Algorithm Output  ***********************/
    
        printDA(parent);

        /*************************  Prints Graph of Middle Earth Output ***********************************/
        cout << "Graph of MiddleEarth: " << endl << endl;
        for (const auto& vertex : adjList) {
            cout << vertex.first << ": ";
            for (const auto& edge : vertex.second) {
                cout << edge.first << ", ";
            }
            cout << endl;
        }
        cout << endl;
        /************************************************************/

        return distance;
    }
};

/*************************   Helper Functions of BFS Algorithm    **************************************/
void addEdges(map<string, vector<string>>& adjList, string firstElement, string secondElement)
{
    adjList[firstElement].push_back(secondElement);
}

void printBFS(vector<string> shortestPath, string startingNode, string endingNode)
{
    cout << "Shortest path from " << startingNode << " to " << endingNode << " (Breadth First Search): ";
    if (!shortestPath.empty()) {
        for (const string& vertex : shortestPath) {
            cout << vertex << " -> ";
        }
        cout << "\b\b\b  " << endl;
    } else {
        cout << "No path found." << endl;
    }
}
vector<string> BFS(map<string, vector<string>>& adjList, string startingNode, string endingNode)
{
    queue<string> q;
    set<string> visited;
    map<string, string> parent;

    visited.insert(startingNode);
    q.push(startingNode);

    while(!q.empty())
    {
        string currentNode = q.front();
        q.pop();

        if (currentNode == endingNode) {
            vector<string> shortestPath;
            while (currentNode != startingNode) {
                shortestPath.push_back(currentNode);
                currentNode = parent[currentNode];
            }
            shortestPath.push_back(startingNode);
            
            vector<string> shortestPathRev;
            for(int i = shortestPath.size() - 1; i >= 0; i--)
            {
                shortestPathRev.push_back(shortestPath[i]);
            }

            return shortestPathRev;
        }
        
        for(const auto& neighbor : adjList[currentNode])
        {
            if(visited.find(neighbor) == visited.end())
            {
                visited.insert(neighbor);
                parent[neighbor] = currentNode;
                q.push(neighbor);
            }
        }
    }

      return {};
}

/************************   Prims Algorithm   ***************************************/

void primMST(map<string, map<string, int>>& graph, const string& startVertex) {
    priority_queue<pair<int, string>, vector<pair<int, string>>, greater<pair<int, string>>> pq; // Stores pair<int, string> in a vector container and implements logic of Min Heap
    set<string> visted;
    map<string, string> parent;
    unordered_map<string, int> key;

    for (const auto &vertex : graph)
    {
        key[vertex.first] = numeric_limits<int>::max();
        parent[vertex.first] = "Null";
    }

    key[startVertex] = 0;

    pq.push({0, startVertex});
    
    while(!pq.empty())
    {
        pair<int,string> topElement = pq.top();
        int weight = topElement.first;
        string location = topElement.second;

        pq.pop();

        if(visted.find(location) != visted.end())
        {
            continue;
        }
        
        visted.insert(location);

        for(const auto& pair : graph.at(location))
        {
            string neighbor = pair.first;
            int weight = pair.second;

            if(weight < key[neighbor])
            {
                parent[neighbor] = location;
                key[neighbor] = weight;
                pq.push({weight, neighbor});
            }
        }
    }

    for(const auto&pair : parent)
    {
        string child = pair.first;
        string parent = pair.second;
        cout << child << ": \"" << parent << "\"" << endl;
    }
}



int main() {
    
    // This is for BFS
    map<string, vector<string>> adjList;

    ifstream inputFile("MiddleEarthEdges.txt");

    string currentLine;

    while(getline(inputFile, currentLine)) 
    {
        stringstream ss(currentLine);
        string firstElement;
        string secondElement;
        getline(ss, firstElement, ',');
        getline(ss, secondElement, ',');

        addEdges(adjList, firstElement, secondElement);
    }

    inputFile.close();

    string startingNode = "Hobbiton";
    string endingNode = "MountDoom";
    vector<string> shortestPath = BFS(adjList, startingNode, endingNode);
    
    printBFS(shortestPath, startingNode, endingNode);

    /***************************** Dijkstra Algorithm & Prints info about Middle Earth *************************************/

    Graph graph;

    ifstream inputFile1("MiddleEarthEdges.txt");

    string currentLine1;

    while(getline(inputFile1, currentLine1)) 
    {
        stringstream ss(currentLine1);
        string firstElement;
        string secondElement;
        string weightStr;
       
        getline(ss, firstElement, ',');
        getline(ss, secondElement, ',');
        getline(ss, weightStr);
        int weightInt = stoi(weightStr);

        graph.addEdge(firstElement, secondElement, weightInt);
    }

    inputFile1.close();

    
    map<string, int> shortestDistances = graph.dijkstra("Hobbiton");


    // Calls Prim Method
    primMST(graph.adjList, "Hobbiton");


    return 0;
}

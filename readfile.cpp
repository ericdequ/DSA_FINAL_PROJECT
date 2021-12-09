#include<cstdlib>
#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <forward_list>
#include <climits>
#include<sstream>
#include <fstream>
#include<string>
using namespace std;



struct EdgeData{
    string to = 0;
    string from = 0;
    string weight = 0;

    void showEdgeData(){
        cout<<"The Edge From Node Id"<<from<<", To Node Id "<<to<<" is "<<weight<<" meters long"<<endl; 
    }

};

struct NodeData{
    int id = 0;
    int OSID = 0;
    double latitude = 0.000000;
    double longitude = 0.000000;
};

void getEdgeDataAfterConstruction(vector<vector<string>> &allEdges){
    std::fstream file("edge_Data(Complete).txt");
    std::string line; 
    if(file){
        bool b = true;
        getline(file,line);
        while (getline(file,line)){
        vector<string> allparts;
        int pos = 0;
        while ((pos = line.find(',')) != string::npos){
            allparts.push_back(line.substr(0,pos));
            line.erase(0,pos + 1);
        }
            
            allparts.push_back(line);
            allEdges.push_back(allparts);
        }
        file.close();
    }else{
        cout<<"File couldnt open"<<endl;
    }   
}

void nodeDataAfterConstruction(vector<vector<string>> &allnodes){ // built as seperate file so adding extra node data only requires changing this function and node class
    std::fstream file("Node_Data_NO_RESTRAUNT.txt");
    std::string line; 
    if(file){
        getline(file,line);
        while (getline(file,line)){
        vector<string> allparts;
        int pos = 0;
        while ((pos = line.find(',')) != string::npos){
            allparts.push_back(line.substr(0,pos));
            line.erase(0,pos + 1);
        }
            allparts.push_back(line);
            allnodes.push_back(allparts);
        }
        file.close();
    }else{
        cout<<"File couldnt open"<<endl;
    }   
}


void getEdgeDataBeforeConstruction(vector<vector<string>> &allEdges){
    std::fstream file("alledges.csv");
    std::string line; 
    if(file){
        bool b = true;
        getline(file,line);
        while (getline(file,line)){
        vector<string> allparts;
        int pos = 0;
        while ((pos = line.find(',')) != string::npos){
            allparts.push_back(line.substr(0,pos));
            line.erase(0,pos + 1);
        }
        
            
            allparts.push_back(line);
            allEdges.push_back(allparts);
        }
        file.close();
    }else{
        cout<<"File couldnt open"<<endl;
    }   
}

void nodeDataBeforeConstruction(vector<vector<string>> &allnodes){ // built as seperate file so adding extra node data only requires changing this function and node class
    std::fstream file("allnodes.csv");
    std::string line; 
    if(file){
        getline(file,line);
        while (getline(file,line)){
        vector<string> allparts;
        int pos = 0;
        while ((pos = line.find(',')) != string::npos){
            allparts.push_back(line.substr(0,pos));
            line.erase(0,pos + 1);
        }
            allparts.push_back(line);
            allnodes.push_back(allparts);
        }
        file.close();
    }else{
        cout<<"File couldnt open"<<endl;
    }
}


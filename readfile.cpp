#include<cstdlib>
#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <forward_list>
#include <climits>
#include<sstream>
#include<string>
using namespace std;

struct EdgeData{
    int to = 0;
    int from = 0;
    int weight = 0;
}

void getdata(){
    std::ifstream file("alledges.csv");
    std::string str; 
    while (std::getline(file, str))
    {
        vector<string> result;
        stringstream s_stream(str); //create string stream from the string
        cout<<"str = "<<str<<endl;
        while(s_stream.good()) {
            string substr;
            getline(s_stream, substr, ','); //get first string delimited by comma
            cout<<"substr = "<<substr<<endl;
        }
        cout<<endl<<endl;
    }
}

void nodeData(){ // built as seperate file so adding extra node data only requires changing this function and node class

}





#include<iostream>
#include<unordered_map>
#include<unordered_set>
#include<string>
#include<vector>
#include<queue>
#include<limits>
#include<fstream>
#include<stack>
#include<sstream>

#include"pQueue.h"

using namespace std;

const int NUM_RESULTS = 1000;
const char ABSTRACT_MARKER = 'i';
const char KEYWORD_MARKER = 'C';
const char SEMANTIC_MARKER = 'T';

struct forwardEdge{
  forwardEdge(string e, float w):prevName(e), pathWeight(w){}
  string prevName;
  float pathWeight;
  bool operator<(const forwardEdge& other) const {
    return this->pathWeight < other.pathWeight;
  }
};

struct graphNode{
  graphNode():name("NULL"), prevPathNode("NULL"){}
  graphNode(string n):
    name(n),
    prevPathNode(n)
  {}
  unordered_map<string,float> edges;
  string prevPathNode;
  string name;
};

typedef unordered_map<string,graphNode> graphMap;

void addNode(string name, graphMap& graph){
  if(graph.find(name)==graph.end()){
    graphNode newNode(name);
    graph[name] = newNode;
  }
}

void constructGraph(string fileName, graphMap& graph){
  fstream inFile(fileName.c_str(),ios::in);
  string start, end;
  float weight;
  while(inFile>>start>>end>>weight){
    addNode(start, graph);
    graph[start].edges[end] = weight;
    addNode(end, graph);
    graph[end].edges[start] = weight;
  }
  inFile.close();
}

vector<string> getPath(string startName, string endName, graphMap& graph){
  if(graph.find(endName) != graph.end()){
    stack<string> stk;
    string cName = endName;
    while(cName != startName){
      stk.push(cName);
      cName = graph[cName].prevPathNode;
    }
    vector<string> res;
    res.push_back(startName);
    while(!stk.empty()){
      res.push_back(stk.top());
      stk.pop();
    }
    return res;
  }
  return vector<string>();
}

unordered_set<string>& getMoreAbstracts(unordered_set<string>& abstracts, graphMap& graph, int resultSize){
  pQueue<string, float> q;
  for(string name:abstracts){
    q.push(name, 0);
  }

  while(!q.empty() && abstracts.size() < resultSize){
    pair<string, float>cPair = q.pop();

    string& currName = cPair.first;
    float pathWeight = cPair.second;

    graphNode& cNode = graph[currName];
    abstracts.insert(currName);
    for(auto edge : cNode.edges){
      //if the edge points to an abstract we havn't seen before
      if(edge.first[0] == ABSTRACT_MARKER
          && abstracts.find(edge.first) == abstracts.end()){
        q.push(edge.first,pathWeight + edge.second);
      }
    }
  }
  return abstracts;
}

//runs a 1:n shortest path query all at same time. Returns paths
vector<vector<string> > runDijkstra(string start, const vector<string>& end, graphMap& graph){
  cout << "Running Dijkstra's, looking for "<<end.size() << " goals." << endl;
  unordered_set<string> goals;
  unordered_set<string> visited;
  for(string s : end){
    goals.insert(s);
  }

  pQueue<string,forwardEdge> q;
  q.push(start,forwardEdge(start, 0));
  visited.insert(start);

  while(!q.empty() && goals.size() > 0){
    pair<string, forwardEdge> cPair = q.pop();

    string& prevName = cPair.second.prevName;
    string& currName = cPair.first;
    float pathWeight = cPair.second.pathWeight;

    graphNode& cNode = graph[currName];
    cNode.prevPathNode = prevName;

    //update goals
    goals.erase(cNode.name);
    visited.insert(currName);

    for(pair<string,float> edge : cNode.edges){
      if(visited.find(edge.first) == visited.end()){
        forwardEdge fe(currName, edge.second + pathWeight);
        q.push(edge.first, fe); //relies on push updating min
      }
    }
  }
  vector<vector<string> > paths;
  for(string eName : end){
    paths.push_back(getPath(start, eName, graph));
  }
  return paths;
}

unordered_set<string>& getOverlapAbstracts(vector<string> path, graphMap& graph, unordered_set<string>& abstracts){
  //in this section we are going to try and catch any overlapping abstracts
  //cerr << "Getting overlap data for " << path[0] << "->" << path[path.size()-1] << endl;
  for(int i = 0 ; i < path.size()-1; i++){
    //get two adjacent path nodes
    string nodeA = path[i];
    string nodeB = path[i+1];
    //if the nodes are both keywords
    if(nodeA[0] == KEYWORD_MARKER && nodeB[0] == KEYWORD_MARKER){
      //optimization, check smaller against larger
      unordered_map<string,float>*  childrenLarge;
      unordered_map<string,float>*  childrenSmall;
      if(graph[nodeA].edges.size() < graph[nodeB].edges.size()){
        childrenLarge = & graph[nodeB].edges;
        childrenSmall = & graph[nodeA].edges;
      }else{
        childrenLarge = & graph[nodeA].edges;
        childrenSmall = & graph[nodeB].edges;
      }
      //we want an upper bound on these abstracts
      int intersectionCount = 100;
      for(auto pair: *childrenSmall){
        //if there is a shared abstract between the two
        if(intersectionCount > 0 && pair.first[0] == ABSTRACT_MARKER && childrenLarge->find(pair.first) != childrenLarge->end()){
          abstracts.insert(pair.first);
          intersectionCount--;
        }
      }
    }
  }
  return abstracts;
}

int main (int argc, char** argv){
  if(argc < 5){
    cerr << "Must supply graph_file, tid_file, start_tid, and out_file command line arguments"<<endl;
  } else {
    string graphFileName = argv[1];
    string startName = argv[3];
    vector<string> endNames;
    fstream tidFile(argv[2],ios::in);
    fstream outFile(argv[4], ios::out);
    string id;
    while(tidFile >> id) endNames.push_back(id);
    tidFile.close();
    cerr << "Found TIDS:" << endNames.size() << endl;

    graphMap graph;
    cerr << "Constructing Graph" << endl;
    constructGraph(graphFileName, graph);
    cerr << "Found " << graph.size() << " nodes" << endl;
    cerr << "Starting Traversal" << endl;
    vector<vector<string> > paths = runDijkstra(startName, endNames, graph);
    cerr << "Got Paths:" << paths.size() << endl;
    for(vector<string> path : paths){
      if(path.size() > 1){
        unordered_set<string> abstracts;

        outFile << "PATH: ";
        for(string name : path){
          if(name[0] == ABSTRACT_MARKER)
            abstracts.insert(name);
          outFile << name << " ";
        }
        outFile << endl;
        abstracts = getMoreAbstracts(abstracts, graph, NUM_RESULTS);
        abstracts = getOverlapAbstracts(path, graph, abstracts);

        outFile << "RELATED: ";
        for(string abstract : abstracts){
          outFile << abstract << " ";
        }
        outFile << endl;
      }
    }
    outFile.close();
  }

  return 0;
}

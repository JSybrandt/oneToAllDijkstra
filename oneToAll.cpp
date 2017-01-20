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

using namespace std;

const int NUM_RESULTS = 1000;
const char ABSTRACT_MARKER = 'i';
const char KEYWORD_MARKER = 'C';
const char SEMANTIC_MARKER = 'T';

struct forwardEdge{
  forwardEdge(string s, string e, float w):startName(s), endName(e), pathWeight(w){}
  string startName, endName;
  float pathWeight;
  bool operator<(const forwardEdge& other) const {
    return this->pathWeight > other.pathWeight;
  }
};

struct graphNode{
  graphNode():name("NULL"), pathWeight(-1), prevPathNode("NULL"){}
  graphNode(string n):
    name(n),
    pathWeight(numeric_limits<float>::max()),
    prevPathNode(n)
  {}
  unordered_map<string,float> edges;
  float pathWeight;
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

unordered_set<string> getMoreAbstracts(unordered_set<string> initialAbstracts, graphMap& graph, int resultSize){

  //cerr << "Extending Abstract Set" << endl;
  unordered_set<string> visitedNodes = initialAbstracts;
  priority_queue<forwardEdge> q;
  for(string name:initialAbstracts){
    q.push(forwardEdge(name,name,0));
  }

  while(!q.empty() && visitedNodes.size() < resultSize){
    forwardEdge cEdge = q.top();
    q.pop();
    graphNode& cNode = graph[cEdge.endName];
    visitedNodes.insert(cNode.name);
    //cerr << "Visiting:" << cNode.name << ":" << cEdge.pathWeight << endl;
    for(auto edge : cNode.edges){
      //if the edge points to an abstract we havn't seen before
      if(edge.first[0] == ABSTRACT_MARKER
          && visitedNodes.find(edge.first) == visitedNodes.end()){
        q.push(forwardEdge(cNode.name,edge.first,cEdge.pathWeight + edge.second));
      }
    }
  }
  return visitedNodes;
}

//runs a 1:n shortest path query all at same time. Returns paths
vector<vector<string> > runDijkstra(string start, const vector<string>& end, graphMap& graph){
  unordered_set<string> goals;
  for(string s : end){
    goals.insert(s);
  }
  priority_queue<forwardEdge> q;
  q.push(forwardEdge(start, start, 0));
  while(!q.empty() && goals.size() > 0){
    forwardEdge cEdge = q.top();
    q.pop();
    //cerr << "Traverse:" << cEdge.endName << ":" << cEdge.pathWeight << endl;
    graphNode& cNode = graph[cEdge.endName];
    if(cNode.pathWeight > cEdge.pathWeight){
      cNode.pathWeight = cEdge.pathWeight;
      cNode.prevPathNode = cEdge.startName;
      //update goals
      goals.erase(cNode.name);
      for(pair<string,float> edge : cNode.edges){
        forwardEdge fe(cNode.name, edge.first, edge.second + cNode.pathWeight);
        q.push(fe);
      }
    }
  }
  vector<vector<string> > paths;
  for(string eName : end){
    paths.push_back(getPath(start, eName, graph));
  }
  return paths;
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
    //cerr << "Found TIDS:" << endNames.size() << endl;

    graphMap graph;
    //cerr << "Constructing Graph" << endl;
    constructGraph(graphFileName, graph);
    //cerr << "Found " << graph.size() << " nodes" << endl;
    //cerr << "Starting Traversal" << endl;
    vector<vector<string> > paths = runDijkstra(startName, endNames, graph);
    //cerr << "Got Paths:" << paths.size() << endl;
    for(vector<string> path : paths){
      unordered_set<string> abstracts;
      outFile << "PATH: ";
      for(string name : path){
        if(name[0] == ABSTRACT_MARKER)
          abstracts.insert(name);
        outFile << name << " ";
      }
      //in this section we are going to try and catch any overlapping abstracts
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
          for(auto pair: *childrenSmall){
            //if there is a shared abstract between the two
            if(pair.first[0] == ABSTRACT_MARKER && childrenLarge->find(pair.first) != childrenLarge->end()){
              abstracts.insert(pair.first);
            }
          }
        }
      }
      outFile << endl;
      outFile << "RELATED: ";
      for(string abstract : getMoreAbstracts(abstracts, graph, NUM_RESULTS)){
        outFile << abstract << " ";
      }
      outFile << endl;
    }
    outFile.close();
  }

  return 0;
}

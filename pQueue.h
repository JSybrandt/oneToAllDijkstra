#pragma once

#include<unordered_map>
#include<algorithm>
#include<iostream>
#include<stack>

using std::string;
using std::unordered_map;
using std::swap;
using std::cout;
using std::endl;
using std::stack;

struct pNode{
  pNode(string n, float w):
    val(w),
    name(n),
    left(nullptr),
    right(nullptr),
    parent(nullptr){}

  ~pNode(){
    removePointers();
  }
  void removePointers(){
//cout<<"Erasing:"<<name<<endl;
    if(left)
      left->parent = nullptr;
    if(right)
      right->parent = nullptr;
    if(parent)
      if(parent->left == this)
        parent->left = nullptr;
      else
        parent->right = nullptr;
    left = right = parent = nullptr;
  }
  float val;
  string name;
  pNode *left, *right, *parent;

};

struct pQueue{
  pQueue(){ head = nullptr;}
  ~pQueue(){
    for(auto p : nameRefs)
      delete p.second;
  }
  pNode* merge(pNode* a, pNode* b){
    if(!a){
      return b;
    }
    if(!b){
      return a;
    }
    if(b->val < a->val){
      swap(a,b);
    }
    swap(a->left, a->right);
    a->right = merge(b, a->right);
    a->right-> parent = a;
    return a;
  }
  void insert(string name, float val){
    pNode* node = new pNode(name,val);
    nameRefs[name] = node;
    head = merge(head,node);
  }

  void del(string name){
    pNode* node = nameRefs[name];
    pNode* left = node->left;
    pNode* right = node->right;
    if(node == head)
      head = nullptr;
    delete node;
    head = merge(head,merge(left,right));
    nameRefs.erase(name);
  }
  void change(string name, float val){
    del(name);
    insert(name, val);
  }
  std::pair<string,float> popMin(){
    if(head){
      string ret = head->name;
      float v = head->val;
      del(ret);
      return std::make_pair(ret, v);
    } else {
      return std::make_pair("",-1);
    }
  }
  bool isEmpty(){
    return head == nullptr;
  }
  void printStructure(pNode* node, int tabLevel){
    for(int i = 0; i < tabLevel; i++)
      cout <<"\t";
    if(node){
      cout << node->name << ":" << node->val << endl;
      printStructure(node->left, tabLevel+1);
      printStructure(node->right, tabLevel+1);
    }else{
      cout << "*" << endl;
    }
  }
  pNode* head;
  unordered_map<string, pNode*> nameRefs;
};

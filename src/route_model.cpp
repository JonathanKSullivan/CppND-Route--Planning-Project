#include "route_model.h"
#include <iostream>
#include <vector>

using std::vector;

RouteModel::Node& RouteModel::FindClosestNode(float x, float y){
  RouteModel::Node node;
  node.x = x;
  node.y = y;
  
  float min_dist = std::numeric_limits<float>::max();
  float dist;
  int closest_idx;
  
  for(const RouteModel::Road& road : Roads()){
    if(road.type != Model::Road::Type::Footway){
      for(const int index : Ways()[road.way].nodes){
        dist = node.distance(SNodes()[index]);
        if(dist < min_dist){
          closest_idx = index;
          min_dist = dist;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int counter(0);
  for(int i = 0; i < this->Nodes().size(); i++){
     m_Nodes.push_back(RouteModel::Node(i, this, this->Nodes()[i]));
  }
  
  CreateNodeToRoadHashmap();
}

void RouteModel::Node::FindNeighbors(){
  for(const Model::Road* road : parent_model->node_to_road[this->index]){
    RouteModel::Node* neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if(neighbor){
      this->neighbors.push_back(neighbor);
    }
  }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(
  std::vector<int> node_indices){
  RouteModel::Node* closest_node = nullptr;
  RouteModel::Node node;
  for(int node_index : node_indices){
    RouteModel::Node node = parent_model->SNodes()[node_index];
    if(node.visited == false && this->distance(node) != 0.0 ){
      if(closest_node == nullptr || (this->distance(node) < this->distance(*closest_node))){
        closest_node = &parent_model->SNodes()[node_index];
      }
    }
  }
  return closest_node;
}

std::unordered_map<int, vector<const Model::Road*>>
      RouteModel::GetNodeToRoadMap(){
        return node_to_road;
      }
void RouteModel::CreateNodeToRoadHashmap(){
  for(const Model::Road& road : Model::Roads()){
    if(road.type != Model::Road::Type::Footway){
      for(int node_idx : Ways()[road.way].nodes){
        if(node_to_road.find(node_idx) == 
           node_to_road.end()){
          node_to_road[node_idx] = 
          vector<const Model::Road*>();
        }
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

vector<RouteModel::Node>& RouteModel::SNodes(){
  return this->m_Nodes;
}
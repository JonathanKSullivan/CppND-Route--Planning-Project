#include "route_planner.h"
#include <algorithm>
#include <vector>

bool RoutePlanner::Compare(const RouteModel::Node *first, const RouteModel::Node *second){
    // f_value is determined by adding g_value to h_value
    return (first->g_value + first->h_value) < (second->g_value + second->h_value);
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
  start_x /= 100;
  start_y /= 100;
  end_x /= 100;
  end_y /= 100;
  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(
  RouteModel::Node* current_node){  
  RouteModel::Node* parent;
  
  std::vector<RouteModel::Node> path_found = {*current_node};
  while(current_node->parent){
    parent = current_node->parent;
    this->distance += current_node->distance(*parent);
    path_found.push_back(*parent);
    current_node = parent;
  }
  this->distance *= m_Model.MetricScale();
  return path_found;
}

void RoutePlanner::AStarSearch(){
  this->start_node->visited = true;
  open_list.push_back(this->start_node);
  RouteModel::Node* current_node = nullptr;
  while(open_list.size() > 0){
    RouteModel::Node* new_node = this->NextNode();
    new_node->parent = current_node;
    current_node = new_node;
    if(current_node->distance(*this->end_node) == 0){
      m_Model.path = this->ConstructFinalPath(end_node);
      return;
    }else{
      this->AddNeighbors(current_node);
    }
  }
  
}

float RoutePlanner::CalculateHValue(
  const RouteModel::Node* node){
  return node->distance(*this->end_node);
}

RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort open_list by f value
    std::sort(open_list.begin(), open_list.end(), Compare);

    // Temporary variable to hold the position of the nearest node
    RouteModel::Node* first = open_list.front();

    // Remove node from open nodes to avoid revisiting it
    open_list.erase(open_list.begin());

    return first;
}

void RoutePlanner::AddNeighbors(RouteModel::Node* current_node){
  current_node->FindNeighbors();
  for(RouteModel::Node* neighbor : current_node->neighbors){
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = this->CalculateHValue(neighbor);
    neighbor->visited = true;
    open_list.push_back(neighbor);
  }
}
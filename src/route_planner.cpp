#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


// Calcuates the h-value heuristic (euclidean distance as calculated by RouteModel::Node::distance)
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}


// Expands the current node by adding all unvisited neighbors to the open list and setting their parent,
// h_value, g_value, and visited attributes. Adds all unvisited neighbours to the open_list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto neighbour : current_node->neighbors) {    
        neighbour->parent = current_node;
        neighbour->h_value = CalculateHValue(neighbour);
        neighbour->g_value = current_node->g_value + current_node->distance(*neighbour);
        neighbour->visited = true;
        open_list.emplace_back(neighbour);
    }
}

// Comparator that sorts by descending g-value
bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
    float f_a = a->g_value + a->h_value;
    float f_b = b->g_value + b->h_value;

    return f_a > f_b;
}

// Sorts list be descending g-value
void SortNodes(std::vector<RouteModel::Node*> *list) {
    sort(list->begin(), list->end(), Compare);
}


// Sorts the open_list and returns the optimal node (lowest g-value in the open_list)
RouteModel::Node *RoutePlanner::NextNode() {
    SortNodes(&open_list);
    auto closest_neighbour = open_list.back();
    open_list.pop_back();
    return closest_neighbour;
}


// Construct the path from the current_node to the starting node by traversing parents
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    do {
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    } while (current_node != start_node);
    path_found.emplace_back(*start_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// Runs A* Search on the given start_node and end_node
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    // Initialize A* Search with start node
    start_node->visited = true;
    start_node->g_value = 0;
    start_node->h_value = CalculateHValue(start_node);
    open_list.emplace_back(start_node);

    while (!open_list.empty()) {
        current_node = NextNode();
        if (current_node == end_node) {
             m_Model.path = ConstructFinalPath(end_node);
            return;
        }
        AddNeighbors(current_node);
    }
}
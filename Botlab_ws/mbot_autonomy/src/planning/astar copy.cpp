#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    // -----------------------
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    
    // startNode->g_cost = 0;
    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    
    // If start == goal
    if (startNode == goalNode) {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        path.path_length = path.path.size();
        return path;

    }

    PriorityQueue open; 
    open.push(startNode);
    std::vector<Node*> closed; // discovery

    while(!open.empty()) {
        Node* n = open.pop();
        std::vector<Node*> kids = expand_node(n, distances, params);
        // std::cout << "size of kids: " << kids.size() << std::endl;

        for (auto kid : kids) {
            kid->h_cost = h_cost(kid, goalNode, distances);
            kid->g_cost = n->g_cost + g_cost(n, kid, distances, params);
            printf("is this get called???????\n");
            if (kid == goalNode) 
            {
                printf("Goal if found!");
                found_path = true;
                break;
            }
            else if (!is_in_list(kid, closed)) {
                open.push(kid);
                std::cout << "Open is increased by one" << std::endl;
            }
        }
        closed.push_back(n);
        std::cout << "size closed: " << closed.size() << std::endl;
    }


    if (found_path)
    {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();
    return path;

}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    h_cost = abs(goal->cell.x - from->cell.x) + abs(goal->cell.y - from->cell.y);
    
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    g_cost = from->g_cost + 1/distances(from->cell.x, from->cell.y); //Need to change for obstacle distance
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    for (int n = 0; n < 8; ++n)
    {
        // printf("testing::");
        cell_t adjacentCell(node->cell.x + xDeltas[n], node->cell.y + yDeltas[n]);
        // Check if neighbor is in the grid and if it is obstacle

        if (distances.isCellInGrid(adjacentCell.x, adjacentCell.y) && distances.operator()(adjacentCell.x, adjacentCell.y) > 0)
        {
            // float distance = node->parent->g_cost;
            // printf("testing done");
            // if (n < 4) distance += 1.0;
            // else distance += 1.414;

            Node* adjacentNode = new Node(adjacentCell.x, adjacentCell.y);
            // Update neigbor parent and costs
            adjacentNode->parent = node;
            children.push_back(adjacentNode);
        }
        // printf("\n");
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    Node* node = goal_node;
    while (node != start_node) {
        printf("Test");
        path.push_back(node);
        node = node->parent;
    }
    
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    for (auto node : nodes) {
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = node->cell.x * distances.cellsPerMeter();
        pose.y = node->cell.y * distances.cellsPerMeter();
        pose.theta = atan2(pose.y, pose.x);
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    // add the statrting node
    new_node_path.push_back(nodePath[0]);
    
    for (size_t i = 1; i++; i < nodePath.size()) {
        Node* currentNode = nodePath[i];
        Node* endOfNewPath = *new_node_path.end();
        // if the rest nodes are not the same as the last node 
        // of the new_node_path, add it to the new_node_path
        // if (currentNode->theta != endOfNewPath->theta) {
        //     new_node_path.push_back(currentNode);
        // }
    }
    return new_node_path;

}

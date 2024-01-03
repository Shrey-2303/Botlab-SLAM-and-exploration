#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);

    PriorityQueue open;
    std::vector<Node*> closed;
    std::vector<Node*> SearchedSet;
    startNode->h_cost = h_cost(startNode, goalNode, distances);
    startNode->g_cost = 0;
    open.push(startNode);
    closed.push_back(startNode);
    SearchedSet.push_back(startNode);
    while (!open.empty())
    {
        Node* currentNode = open.pop();
        if(currentNode->cell.x == goalNode->cell.x && currentNode->cell.y == goalNode->cell.y)
        {
            found_path = true;
            break;
        }
        closed.push_back(currentNode);

        const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
        const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

        for (int n = 0; n < 8; ++n) { 
            int newX = currentNode->cell.x + xDeltas[n];
            int newY = currentNode->cell.y + yDeltas[n];
            if (distances.isCellInGrid(newX,newY) && distances.operator()(newX, newY) > 0.2 ) 
            {
                Node* neighbor = new Node(newX, newY);
                if (is_in_list(neighbor, closed)) {
                    continue;
                }
                if(is_in_list(neighbor,SearchedSet))
                {
                    neighbor = get_from_list(neighbor,SearchedSet);
                }

                double newGcost = currentNode->g_cost + g_cost(currentNode, neighbor, distances, params);

                bool isMember = false;

                if(!is_in_list(neighbor,SearchedSet))
                {
                    neighbor->g_cost = newGcost;
                    neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                    neighbor->parent = currentNode;
                    open.push(neighbor);
                    SearchedSet.push_back(neighbor);
                }
                else if(neighbor->g_cost > newGcost)
                {
                    neighbor->g_cost = newGcost;
                    neighbor->parent = currentNode;
                    open.push(neighbor);
                }
            }
        }

    }

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        auto nodePath = extract_node_path(get_from_list(goalNode,SearchedSet), startNode);
        nodePath = prune_node_path(nodePath,distances);
        // nodePath = prune_node_path(nodePath);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
        path.path_length = path.path.size();
        printf("[A*] find a path with length %d\n", path.path_length);
        return path;
    }

    else 
    {
        printf("[A*] Didn't find a path%ld\n",path.path.size());
        path.path_length = path.path.size();
        return path;
    }
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    int dx = fabs(goal->cell.x - from->cell.x);
    int dy = fabs(goal->cell.y - from->cell.y);
    h_cost = (dx + dy) * distances.metersPerCell();
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    if (abs(goal->cell.x - from->cell.x) + abs(goal->cell.y - from->cell.y) == 2)
        g_cost = distances.metersPerCell()*2.2 + 1/distances.operator()(goal->cell.x, goal->cell.y);
    else
        g_cost = distances.metersPerCell() + 1/distances.operator()(goal->cell.x, goal->cell.y);
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

        if (distances.isCellInGrid(adjacentCell.x, adjacentCell.y) && distances.operator()(adjacentCell.x, adjacentCell.y) > 0.2)
        {

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
    while (node != nullptr) {
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
    
    for(auto node : nodes)
    {
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = node->cell.x * distances.metersPerCell() + distances.originInGlobalFrame().x;
        pose.y = node->cell.y * distances.metersPerCell() + distances.originInGlobalFrame().x;
        pose.theta = atan2(pose.y, pose.x);
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (node->cell.x == item->cell.x && node->cell.y == item->cell.y) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (node->cell.x == n->cell.x && node->cell.y == n->cell.y) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath, const ObstacleDistanceGrid& distances)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    float theta12;
    float theta13;
    float threshold_distance = 0.3;
    // first_theta = atan2(nodePath[0]->cell.y, nodePath[0]->cell.x);
    new_node_path.push_back(nodePath[0]);

    for (size_t i = 1; i < nodePath.size() - 1; ++i) {
        float lastX = new_node_path.back()->cell.x;
        float lastY = new_node_path.back()->cell.y;
        // get theta of next node
        theta12 = atan2((nodePath[i]->cell.y - lastY), (nodePath[i]->cell.x - lastX));
        theta13 = atan2((nodePath[i+1]->cell.y - lastY), (nodePath[i+1]->cell.x - lastX));
        // if next theta is different than the previous theta, add to new_node_path

        double distance = std::hypot(lastX - nodePath[i]->cell.x, lastY - nodePath[i]->cell.y) * distances.metersPerCell();
        // if (theta12 != theta13 && distance > threshold_distance) {
        //     new_node_path.push_back(nodePath[i]);
        // }
        // if (theta12 != theta13) {
        //     new_node_path.push_back(nodePath[i]);
        // }
        new_node_path.push_back(nodePath[i]);
    }

    // for (size_t i = 0; i < nodePath.size(); ++i) {
    //     // get theta of next node
    //     second_theta = atan2(nodePath[i]->cell.y, nodePath[i]->cell.x);
    //     // if next theta is different than the previous theta, add to new_node_path
    //     if (first_theta != second_theta) {
    //         new_node_path.push_back(nodePath[i]);
    //         first_theta = second_theta;
    //     }
    // }
    return new_node_path;
}

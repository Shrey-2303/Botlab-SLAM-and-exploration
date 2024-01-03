#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(14),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}


void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    
}

int sign2(float x){
    return (x>0) ? 1: ((x<0) ? -1 : 0);
}
std::vector<Point<int>> bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    std::vector<Point<int>> cells;
    Point<float> raystart = global_position_to_grid_position(ray.origin,map);
    Point<float> rayend;
    rayend.x = (ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + raystart.x;
    rayend.y = (ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + raystart.y;

    int x0 = static_cast<int>(raystart.x);
    int y0 = static_cast<int>(raystart.y);
    int x1 = static_cast<int>(rayend.x);
    int y1 = static_cast<int>(rayend.y);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = sign2(x1 - x0);
    int sy = sign2(y1 - y0);
    int err = dx - dy, e2;
    int x = x0;
    int y = y0;

    std::vector<Point<int>> points;
    points.push_back(Point<int>(x,y));

    while((x != x1) || (y != y1)) {
        e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <=dx) {
            err += dx;
            y += sy;
        }
        points.push_back(Point<int>(x,y));
    }

    return points;    
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    MovingLaserScan MovingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;

    adjusted_ray_t ray;
    for (unsigned i = 0; i < MovingScan.size(); i+=ray_stride_)
    {
        ray = MovingScan.at(i);
        //super simple youtube heuristic
        if (ray.range < 5.5)
        {
            Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                                ray.origin.y + ray.range * std::sin(ray.theta));
            auto rayEnd = global_position_to_grid_cell(endpoint, map);
            // if (map.isCellInGrid(rayCell.x, rayCell.y)) scanScore += 1;
            if (map.logOdds(rayEnd.x, rayEnd.y) > 0.0)
            {
                scanScore += NormalPdf(0.0);
                // scanScore += map.logOdds(rayEnd.x, rayEnd.y);
                // scanScore += 1.0;
            }
            else
            {
                // Point<int> nearest_occupied = gridBFS(endpoint, map);
                // double distance = map.cellsPerMeter()*std::sqrt(std::pow(nearest_occupied.x - endpoint.x,2) + std::pow(nearest_occupied.y - endpoint.y,2));
                // scanScore += NormalPdf(distance);
            }
        }
    }
    //std::cout << scanScore << std::endl;

    return scanScore; // Placeholder
    }






double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

#include <map>
#include <queue>
std::vector<Point<int>> gridAdjacents(const Point<int> end_point)
{
    std::vector<Point<int>> pts;
    pts.push_back({end_point.x-1,end_point.y-1});
    pts.push_back({end_point.x-1,end_point.y});
    pts.push_back({end_point.x-1,end_point.y+1});

    pts.push_back({end_point.x,end_point.y-1});
    pts.push_back({end_point.x,end_point.y+1});

    pts.push_back({end_point.x+1,end_point.y-1});
    pts.push_back({end_point.x+1,end_point.y});
    pts.push_back({end_point.x+1,end_point.y+1});

    return pts;

}
Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point.
    std::map<int,std::map<int,bool>> discovery;
    std::queue<Point<int>> queue;

    queue.push(Point<int>(end_point.x, end_point.y));
    Point<int> cell = Point<int>(end_point.x-100,end_point.y-100);

    unsigned iter = 0;
    double distance;
    while (!queue.empty())
    {
        // std::cout << "Iter: " << iter << std::endl;
        if ((++iter) >= 5) break;
        cell = queue.front();
        queue.pop();
        std::vector<Point<int>> adjacents = gridAdjacents(cell);
        bool find_x, find_y;
        for (const auto& adjacent : adjacents)
        {
            if (find_x = (discovery.find(adjacent.x) == discovery.end())) 
                discovery.insert({adjacent.x,std::map<int,bool>()});

            if (find_y = (discovery.at(adjacent.x).find(adjacent.y) == discovery.at(adjacent.x).end())) 
                discovery.at(adjacent.x).insert({adjacent.y,true});
            

            // if find_x is false and find_y is false, then it already exists
            if (!find_x && !find_y) continue;
            distance = std::sqrt(std::pow(adjacent.x - end_point.x,2) + std::pow(adjacent.y - end_point.y,2)) * map.cellsPerMeter();
            if (map.logOdds(adjacent.x, adjacent.y) > 0.0) return adjacent;
            else if (distance > 5.0) return adjacent;
            else 
            {
                queue.push(adjacent);

            }
        }
    }

    return cell; // Placeholder
    
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    return Point<float>(0.0f, 0.0f); // Placeholder
    
}

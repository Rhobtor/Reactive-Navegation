#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <queue>
#include <unordered_map>

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_node") {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&PathPlannerNode::octomapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PathPlannerNode initialized!");
    }

private:
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        auto octree = dynamic_cast<octomap::OcTree*>(tree);
        if (!octree) {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast Octomap to OcTree.");
            return;
        }

        octomap::point3d start(0.0, 0.0, 0.0);
        octomap::point3d goal(5.0, 5.0, 0.0);

        auto path = findPath(*octree, start, goal);
        RCLCPP_INFO(this->get_logger(), "Found path with %zu points.", path.size());
    }

    std::vector<octomap::point3d> findPath(octomap::OcTree& tree, const octomap::point3d& start, const octomap::point3d& goal) {
        struct Node {
            octomap::point3d point;
            double cost;
            double heuristic;
            double totalCost() const { return cost + heuristic; }
        };

        auto cmp = [](const Node& a, const Node& b) { return a.totalCost() > b.totalCost(); };
        std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open_set(cmp);

        std::unordered_map<octomap::point3d, octomap::point3d, octomap::point3d_hash> came_from;
        std::unordered_map<octomap::point3d, double, octomap::point3d_hash> cost_so_far;

        open_set.push({start, 0.0, (goal - start).norm()});
        cost_so_far[start] = 0.0;

        while (!open_set.empty()) {
            auto current = open_set.top().point;
            open_set.pop();

            if (current == goal) break;

            for (auto neighbor : getNavigableRegions(tree, current, 1.0)) {
                double new_cost = cost_so_far[current] + (neighbor - current).norm();
                if (cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor]) {
                    cost_so_far[neighbor] = new_cost;
                    open_set.push({neighbor, new_cost, (goal - neighbor).norm()});
                    came_from[neighbor] = current;
                }
            }
        }

        // Reconstruir el camino
        std::vector<octomap::point3d> path;
        for (octomap::point3d at = goal; at != start; at = came_from[at]) {
            path.push_back(at);
        }
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    std::vector<octomap::point3d> getNavigableRegions(octomap::OcTree& tree, const octomap::point3d& pos, double radius) {
        std::vector<octomap::point3d> navigable_points;

        for (auto it = tree.begin_leafs_bbx(pos - octomap::point3d(radius, radius, radius),
                                            pos + octomap::point3d(radius, radius, radius)); it != tree.end_leafs_bbx(); ++it) {
            if (!tree.isNodeOccupied(*it)) {
                navigable_points.push_back(it.getCoordinate());
            }
        }

        return navigable_points;
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}

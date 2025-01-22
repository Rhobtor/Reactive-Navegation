#include "visualization.hpp"


visualization::visualization() {}

visualization_msgs::msg::Marker visualization::PointColor(octomap::point3d point, int iter, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "punto_aleatorio";
    marker.id = iter;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Establece el nuevo color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0; // No transparente

    return marker;
}

visualization_msgs::msg::Marker visualization::createLineMarker(octomap::point3d start, octomap::point3d end, int iteracion) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = rclcpp::Clock().now();
    line_marker.ns = "line";
    line_marker.id = iteracion;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;

    // Escala
    line_marker.scale.x = 0.05; // Ancho de la línea

    // Color
    line_marker.color.r = 0.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;

    // Puntos de inicio y fin
    geometry_msgs::msg::Point start_point;
    start_point.x = start.x();
    start_point.y = start.y();
    start_point.z = start.z();
    line_marker.points.push_back(start_point);

    geometry_msgs::msg::Point end_point;
    end_point.x = end.x();
    end_point.y = end.y();
    end_point.z = end.z();
    line_marker.points.push_back(end_point);

    return line_marker;
}

visualization_msgs::msg::Marker visualization::deleteAllPoints() {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "punto_aleatorio";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;

    return marker;
}

visualization::~visualization() {}
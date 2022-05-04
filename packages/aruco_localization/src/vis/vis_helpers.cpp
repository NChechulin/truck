#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

#include "vis/vis_helpers.hpp"
#include "msgs/msgs_helpers.hpp"

#include <iostream>

namespace rosaruco {
    visualization_msgs::msg::Marker get_marker(const Transform &t, int id, double size) {
        std::vector<tf2::Vector3> points = {
            {0, 0, 0},
            {0, size, 0},
            {size, size, 0},
            {size, 0, 0},
        };

        std::transform(points.begin(), points.end(), points.begin(), 
            [&t](const tf2::Vector3 &v) {
                return t(v);
            }
        );

        points.push_back(points[0]);

        visualization_msgs::msg::Marker marker;

        std::transform(points.begin(), points.end(), back_inserter(marker.points),
            [](const tf2::Vector3& v) {
                return geometry_msgs::msg::Point().set__x(v[0]).set__y(v[1]).set__z(v[2]);
            }
        );

        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.color.a = 1.0;
        marker.color.g = 1.0;
        marker.scale.x = 0.1;

        return marker;
    }

    visualization_msgs::msg::Marker get_label(int id, const tf2::Vector3 &p, double size) {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.text = "id: " + std::to_string(id);
        marker.id = (id + 1) << 10;

        toMsg(p, marker.pose.position);

        marker.color.a = 1.0;
        marker.color.g = 1.0;
        marker.color.r = 1.0;
        marker.color.b = 1.0;
        marker.scale.z = size;
        return marker;
    }

    void add_labeled_marker(std::vector<visualization_msgs::msg::Marker> &markers, const Transform &t, int id, double size) {
        markers.push_back(get_marker(t, id, size));
        markers.push_back(get_label(id, t(tf2::Vector3(size / 2, size / 2, 0)), size / 5));
    }
}

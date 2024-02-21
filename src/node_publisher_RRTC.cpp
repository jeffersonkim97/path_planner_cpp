#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rrtc.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class RRTCNodePublisher : public rclcpp::Node{
    public:
        RRTCNodePublisher()
        : Node("rrtc_node_publisher"), count_(0){
            publisher_line_list_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_nodes", 10);
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&RRTCNodePublisher::rrtc_callback, this)
            );
        }

    private:
        void rrtc_callback() {

            rrtc::RRTC rrtc;

            int max_iter = rrtc.max_iter;

            for (int i = 0; i < max_iter; i++)
            {
                rrtc::Node *q = rrtc.randomSample();

                if (q){
                    rrtc::Node *qnear = rrtc.find_neighbor(q->position, i);
                    if (rrtc.distance(q->position, qnear->position) > rrtc.step_size) {
                        Vector2f qnew_pos = rrtc.extend(q, qnear);
                        rrtc::Node *qnew = new rrtc::Node;
                        qnew->position = qnew_pos;
                        rrtc.add(qnear, qnew, i);
                    };
                };

                if (rrtc.reached()){
                    cout << "End Position: " << rrtc.endPos << endl;
                    cout << "Last RRT Node: " << rrtc.lastNode->position << endl;
                    cout << "Destination Reached \n";
                    break;
                };
            }

            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp.sec = 0;
            marker.header.stamp.nanosec = 0;
            marker.header.frame_id="map";
            marker.ns = "while_line";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = 0;
            marker.pose = geometry_msgs::msg::Pose();
            marker.scale.set__x(0.08);
            marker.color.a = 1.0;
            marker.color.b = 1.0;
            marker.color.g = 1.0;
            marker.color.r = 1.0;
            visualization(&marker, rrtc.rootStart, 0);
            RCLCPP_INFO(this->get_logger(), "Publishing");
            publisher_line_list_->publish(marker);

        };

        void visualization(visualization_msgs::msg::Marker *marker, rrtc::Node *parent, int depth){
            // std::cout << std::string("    ", depth) << "vis parent: " << parent << " children: " << parent->children.size() << " depth: "<< depth << std::endl;

            auto point_parent = geometry_msgs::msg::Point();
            point_parent.x = parent->position.x();
            point_parent.y = parent->position.y();
            point_parent.z = 0;

            for (size_t i=0; i < parent->children.size(); i++){
                // std::cout << "i: " << i <<std::endl;
                rrtc::Node *child = parent->children[i];

                // std::cout << std::string("    ", depth) << "child: " << child << " parent: " << parent << std::endl;
                if (child == parent) {
                    continue;
                }
                auto point_child = geometry_msgs::msg::Point();
                point_child.x = child->position.x();
                point_child.y = child->position.y();
                point_child.z = 0;
                marker->points.push_back(point_parent);
                marker->points.push_back(point_child);
                visualization(marker, child, depth + 1);
            }

        };
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_line_list_;

        size_t count_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTCNodePublisher>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et

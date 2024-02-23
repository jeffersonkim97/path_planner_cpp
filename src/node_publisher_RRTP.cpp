#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rrtp.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class RRTPNodePublisher : public rclcpp::Node{
    public:
        RRTPNodePublisher()
        : Node("rrtp_node_publisher"), count_(0){
            publisher_line_list_ = this->create_publisher<visualization_msgs::msg::Marker>("rrtp_nodes", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&RRTPNodePublisher::rrtp_callback, this)
            );
            }

    private:
        void rrtp_callback() {

            rrtp::RRTP rrtp;

            int max_iter = rrtp.max_iter;
            // int max_iter = 20;

            for (int i = 0; i < max_iter; i++)
            {
                rrtp::Node *q = rrtp.randomSample();
                if (q){
                    rrtp::Node *qnear = rrtp.find_neighbor(q->vertex, i);
                    if (rrtp.distance(q->vertex, qnear->vertex) > rrtp.step_size) {
                        rrtp::Vertex qnew_pos = rrtp.extend(q, qnear);
                        rrtp::Node *qnew = new rrtp::Node;
                        qnew->vertex = qnew_pos;
                        rrtp.add(qnear, qnew, i);
                    };
                };
                
                if (rrtp.reached(i)){
                    std::cout << "End Position: " << rrtp.endPos << std::endl;
                    std::cout << "Destination Reached \n" << std::endl;
                    break;
                };
            }

            // start tree
            auto markerStart = visualization_msgs::msg::Marker();
            markerStart.header.stamp.sec = 0;
            markerStart.header.stamp.nanosec = 0;
            markerStart.header.frame_id="map";
            markerStart.ns = "red_line";
            markerStart.id = 0;
            markerStart.type = visualization_msgs::msg::Marker::LINE_LIST;
            markerStart.action = 0;
            markerStart.pose = geometry_msgs::msg::Pose();
            markerStart.scale.set__x(0.08);
            markerStart.color.a = 1.0;
            markerStart.color.b = 0.0;
            markerStart.color.g = 0.0;
            markerStart.color.r = 1.0;
            visualization(&markerStart, rrtp.rootStart, 0);

            // goal tree
            auto markerGoal = visualization_msgs::msg::Marker();
            markerGoal.header.stamp.sec = 0;
            markerGoal.header.stamp.nanosec = 0;
            markerGoal.header.frame_id="map";
            markerGoal.ns = "blue_line";
            markerGoal.id = 0;
            markerGoal.type = visualization_msgs::msg::Marker::LINE_LIST;
            markerGoal.action = 0;
            markerGoal.pose = geometry_msgs::msg::Pose();
            markerGoal.scale.set__x(0.08);
            markerGoal.color.a = 1.0;
            markerGoal.color.b = 1.0;
            markerGoal.color.g = 0.0;
            markerGoal.color.r = 0.0;
            for (int j = 0; j < rrtp.n; j++){
                visualization(&markerGoal, rrtp.nodesGoal[j], 0);
            }
            RCLCPP_INFO(this->get_logger(), "Publishing");
            publisher_line_list_->publish(markerStart);
            publisher_line_list_->publish(markerGoal);

        };

        void visualization(visualization_msgs::msg::Marker *marker, rrtp::Node *parent, int depth){
            auto point_parent = geometry_msgs::msg::Point();
            point_parent.x = parent->vertex.position.x();
            point_parent.y = parent->vertex.position.y();
            point_parent.z = parent->vertex.time;

            for (size_t i=0; i < parent->children.size(); i++){
                rrtp::Node *child = parent->children[i];

                if (child == parent) {
                    continue;
                }
                auto point_child = geometry_msgs::msg::Point();
                point_child.x = child->vertex.position.x();
                point_child.y = child->vertex.position.y();
                point_child.z = child->vertex.time;
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
    rclcpp::spin(std::make_shared<RRTPNodePublisher>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rrtc3D.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class RRTCNodePublisher : public rclcpp::Node{
    public:
        RRTCNodePublisher()
        : Node("rrtc_node_publisher"), count_(0){
            publisher_line_list_ = this->create_publisher<visualization_msgs::msg::Marker>("rrtc_nodes", 10);
            timer_ = this->create_wall_timer(
                5000ms, std::bind(&RRTCNodePublisher::rrtc_callback, this)
            );
            }

    private:
        void rrtc_callback() {

            rrtc3D::RRTC3D rrtc;

            // int max_iter = rrtc.max_iter;
            int max_iter = 4;

            // TODO
            // Fix time sampling -> cannot sample time correctly from respective trees
            // Add obstacles

            // std::cout << "begin iteration" << std::endl;

            for (int i = 0; i < max_iter; i++)
            {
                std::cout << "i: " << i << std::endl;
                rrtc3D::Node *q = rrtc.randomSample(i);
                std::cout << "Random Sample" << std::endl;
                std::cout << "qrand: " << q->vertex.position.x() << ", " << q->vertex.position.y() << ", " << q->vertex.time << std::endl;
                if (q){
                    rrtc3D::Node *qnear = rrtc.find_neighbor(q->vertex, i);
                    std::cout << "Neighbor Found: " << qnear->vertex.position.x() << ", " << qnear->vertex.position.y() << ", " << qnear->vertex.time << std::endl;
                    std::cout << "Distance to point: " << rrtc.distance(q->vertex, qnear->vertex) << std::endl;
                    std::cout << "Bigger than stepsize?: " << (rrtc.distance(q->vertex, qnear->vertex) > rrtc.step_size) << std::endl;
                    if (rrtc.distance(q->vertex, qnear->vertex) > rrtc.step_size) {
                        rrtc3D::Vertex qnew_pos = rrtc.extend(q, qnear, i);
                        std::cout << "Extended" << std::endl;
                        std::cout << "qnew: " << qnew_pos.position.x() << ", " << qnew_pos.position.y() << ", " << qnew_pos.time << std::endl;
                        rrtc3D::Node *qnew = new rrtc3D::Node;
                        qnew->vertex = qnew_pos;
                        rrtc.add(qnear, qnew, i);
                        std::cout << "Added" << std::endl;

                        // bool checkReachable = rrtc.reachable(qnear->vertex, qnew->vertex, i);
                        // std::cout << "IS it reachable?: " << checkReachable << std::endl;
                    } else{
                        rrtc.add(qnear, q, i);
                        std::cout << "Added" << std::endl;
                        bool checkReachable = rrtc.reachable(qnear->vertex, q->vertex, i);
                        std::cout << "IS it reachable?: " << checkReachable << std::endl;
                    };
                };
                
                if (rrtc.reached(i)){
                    std::cout << "End Position: " << rrtc.endPos << std::endl;
                    std::cout << "Destination Reached \n" << std::endl;
                    break;
                };
            }

            std::cout << "--------------------------------------------------------" << std::endl;
            // Parent:
            float parent_x = rrtc.rootGoal->vertex.position.x();
            float parent_y = rrtc.rootGoal->vertex.position.y();
            float parent_z = rrtc.rootGoal->vertex.time;
            std::cout << "Parent: " << parent_x << ", " << parent_y << ", " << parent_z << std::endl;

            // Child:
            for (size_t k=0; k < rrtc.rootGoal->children.size(); k++){
                rrtc3D::Node *child = rrtc.rootGoal->children[k];
                float child_x = child->vertex.position.x();
                float child_y = child->vertex.position.y();
                float child_z = child->vertex.time;

                std::cout << "Child " << k << ": " << child_x << ", " << child_y << ", " << child_z << std::endl;
            }
            std::cout << "========================================================" << std::endl;

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
            visualization(&markerStart, rrtc.rootStart, 0);

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
            visualization(&markerGoal, rrtc.rootGoal, 0);
            RCLCPP_INFO(this->get_logger(), "Publishing");
            publisher_line_list_->publish(markerStart);
            publisher_line_list_->publish(markerGoal);

        };

        void visualization(visualization_msgs::msg::Marker *marker, rrtc3D::Node *parent, int depth){
            auto point_parent = geometry_msgs::msg::Point();
            point_parent.x = parent->vertex.position.x();
            point_parent.y = parent->vertex.position.y();
            point_parent.z = parent->vertex.time;

            for (size_t i=0; i < parent->children.size(); i++){
                rrtc3D::Node *child = parent->children[i];

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
    rclcpp::spin(std::make_shared<RRTCNodePublisher>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et

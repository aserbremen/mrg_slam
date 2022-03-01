// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <visualization_msgs/MarkerArray.h>

#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <hdl_graph_slam/markers_publisher.hpp>


namespace hdl_graph_slam {

void
MarkersPublisher::onInit( ros::NodeHandle& nh, ros::NodeHandle& mt_nh, ros::NodeHandle& private_nh )
{
    markers_pub  = mt_nh.advertise<visualization_msgs::MarkerArray>( "/hdl_graph_slam/markers", 16 );
    map_frame_id = private_nh.param<std::string>( "map_frame_id", "map" );
    own_name     = private_nh.param<std::string>( "own_name", "atlas" );

    // colors from pyplot tableu palette (https://matplotlib.org/3.1.0/gallery/color/named_colors.html)
    color_blue.r   = 31.0 / 255.0;
    color_blue.g   = 119.0 / 255.0;
    color_blue.b   = 180.0 / 255.0;
    color_blue.a   = 1.0;
    color_orange.r = 255.0 / 255.0;
    color_orange.g = 127.0 / 255.0;
    color_orange.b = 14.0 / 255.0;
    color_orange.a = 1.0;
    color_green.r  = 44.0 / 255.0;
    color_green.g  = 160.0 / 255.0;
    color_green.b  = 44.0 / 255.0;
    color_green.a  = 1.0;
    color_red.r    = 214.0 / 255.0;
    color_red.g    = 39.0 / 255.0;
    color_red.b    = 40.0 / 255.0;
    color_red.a    = 1.0;
    color_purple.r = 148.0 / 255.0;
    color_purple.g = 103.0 / 255.0;
    color_purple.b = 189.0 / 255.0;
    color_purple.a = 1.0;
    color_brown.r  = 140.0 / 255.0;
    color_brown.g  = 86.0 / 255.0;
    color_brown.b  = 75.0 / 255.0;
    color_brown.a  = 1.0;
    color_pink.r   = 227.0 / 255.0;
    color_pink.g   = 119.0 / 255.0;
    color_pink.b   = 194.0 / 255.0;
    color_pink.a   = 1.0;
    color_olive.r  = 188.0 / 255.0;
    color_olive.g  = 189.0 / 255.0;
    color_olive.b  = 34.0 / 255.0;
    color_olive.a  = 1.0;
    color_cyan.r   = 23.0 / 255.0;
    color_cyan.g   = 190.0 / 255.0;
    color_cyan.b   = 207.0 / 255.0;
    color_cyan.a   = 1.0;
    color_black.r  = 0;
    color_black.g  = 0;
    color_black.b  = 0;
    color_black.a  = 1.0;
    color_white.r  = 1.0;
    color_white.g  = 1.0;
    color_white.b  = 1.0;
    color_white.a  = 1.0;
    color_gray.r   = 0.5;
    color_gray.g   = 0.5;
    color_gray.b   = 0.5;
    color_gray.a   = 1.0;
}


void
MarkersPublisher::publish( std::shared_ptr<GraphSLAM>& graph_slam, const std::vector<KeyFrame::Ptr>& keyframes,
                           const std::vector<Edge::Ptr>& edges, const KeyFrame::ConstPtr& last_keyframe,
                           const std::vector<std::pair<KeyFrame::ConstPtr, geometry_msgs::Pose> >& others_last_kf_and_pose,
                           double loop_detector_distance_thresh, const GlobalIdGenerator& gid_gen )
{
    enum {
        MARKER_NODES,
        MARKER_LAST_NODES,
        // MARKER_IMU,
        MARKER_MAIN_EDGES,
        MARKER_SPHERE,
        MARKER_MISC_EDGES,
        __NUM_MARKERS__,
    };
    auto                            stamp   = ros::Time::now();
    RobotId                         own_rid = gid_gen.getRobotId( own_name );
    visualization_msgs::MarkerArray markers;
    markers.markers.resize( __NUM_MARKERS__ );

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[MARKER_NODES];
    traj_marker.header.frame_id             = map_frame_id;
    traj_marker.header.stamp                = stamp;
    traj_marker.ns                          = "nodes";
    traj_marker.id                          = MARKER_NODES;
    traj_marker.type                        = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.3;

    /*
    visualization_msgs::Marker& imu_marker = markers.markers[MARKER_IMU];
    imu_marker.header                      = traj_marker.header;
    imu_marker.ns                          = "imu";
    imu_marker.id                          = MARKER_IMU;
    imu_marker.type                        = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.3;
    */

    traj_marker.points.resize( keyframes.size() );
    traj_marker.colors.resize( keyframes.size() );
    for( int i = 0; i < keyframes.size(); i++ ) {
        Eigen::Vector3d pos     = keyframes[i]->node->estimate().translation();
        traj_marker.points[i].x = pos.x();
        traj_marker.points[i].y = pos.y();
        traj_marker.points[i].z = pos.z();

        /*
        double p                = static_cast<double>( i ) / keyframes.size();
        traj_marker.colors[i].r = 1.0 - p;
        traj_marker.colors[i].g = p;
        traj_marker.colors[i].b = 0.0;
        traj_marker.colors[i].a = 1.0;
        */

        if( gid_gen.getRobotId( keyframes[i]->gid ) == own_rid ) {
            traj_marker.colors[i] = color_blue;
        } else {
            traj_marker.colors[i] = color_cyan;
        }

        /*
        if( keyframes[i]->acceleration ) {
            Eigen::Vector3d      pos = keyframes[i]->node->estimate().translation();
            geometry_msgs::Point point;
            point.x = pos.x();
            point.y = pos.y();
            point.z = pos.z();

            std_msgs::ColorRGBA color;
            color = color_brown;

            imu_marker.points.push_back( point );
            imu_marker.colors.push_back( color );
        }
        */
    }

    visualization_msgs::Marker& last_node_marker = markers.markers[MARKER_LAST_NODES];
    last_node_marker.header.frame_id             = map_frame_id;
    last_node_marker.header.stamp                = stamp;
    last_node_marker.ns                          = "last_nodes";
    last_node_marker.id                          = MARKER_LAST_NODES;
    last_node_marker.type                        = visualization_msgs::Marker::SPHERE_LIST;

    last_node_marker.pose.orientation.w = 1.0;
    last_node_marker.scale.x = last_node_marker.scale.y = last_node_marker.scale.z = 0.5;

    last_node_marker.points.resize( others_last_kf_and_pose.size() + 2 );
    last_node_marker.colors.resize( others_last_kf_and_pose.size() + 2 );

    for( size_t i = 0; i <= others_last_kf_and_pose.size(); i++ ) {
        Eigen::Vector3d pos;
        if( i == others_last_kf_and_pose.size() ) {
            pos                        = last_keyframe->node->estimate().translation();
            last_node_marker.colors[i] = color_blue;
        } else {
            pos                        = others_last_kf_and_pose[i].first->node->estimate().translation();
            last_node_marker.colors[i] = color_cyan;
        }

        last_node_marker.points[i].x = pos.x();
        last_node_marker.points[i].y = pos.y();
        last_node_marker.points[i].z = pos.z();
    }
    // origin
    last_node_marker.points.back().x = 0;
    last_node_marker.points.back().y = 0;
    last_node_marker.points.back().z = 0;
    last_node_marker.colors.back()   = color_olive;

    // main edges
    {
        visualization_msgs::Marker& main_edge_marker = markers.markers[MARKER_MAIN_EDGES];
        main_edge_marker.header.frame_id             = map_frame_id;
        main_edge_marker.header.stamp                = stamp;
        main_edge_marker.id                          = MARKER_MISC_EDGES;
        main_edge_marker.ns                          = "main_edges";
        main_edge_marker.type                        = visualization_msgs::Marker::LINE_LIST;

        main_edge_marker.pose.orientation.w = 1.0;
        main_edge_marker.scale.x            = 0.07;
        main_edge_marker.points.resize( edges.size() * 2 );
        main_edge_marker.colors.resize( edges.size() * 2 );

        size_t i = 0;
        for( const auto& edge : edges ) {
            const g2o::EdgeSE3*   edge_se3 = edge->edge;
            const g2o::VertexSE3* v1       = dynamic_cast<const g2o::VertexSE3*>( edge_se3->vertices()[0] );
            const g2o::VertexSE3* v2       = dynamic_cast<const g2o::VertexSE3*>( edge_se3->vertices()[1] );
            const auto&           pt1      = v1->estimate().translation();
            const auto&           pt2      = v2->estimate().translation();

            main_edge_marker.points[i * 2].x     = pt1.x();
            main_edge_marker.points[i * 2].y     = pt1.y();
            main_edge_marker.points[i * 2].z     = pt1.z();
            main_edge_marker.points[i * 2 + 1].x = pt2.x();
            main_edge_marker.points[i * 2 + 1].y = pt2.y();
            main_edge_marker.points[i * 2 + 1].z = pt2.z();


            if( gid_gen.getRobotId( edge->gid ) == own_rid ) {
                if( edge->type == Edge::TYPE_ODOM ) {
                    main_edge_marker.colors[i * 2] = main_edge_marker.colors[i * 2 + 1] = color_red;
                } else {
                    main_edge_marker.colors[i * 2] = main_edge_marker.colors[i * 2 + 1] = color_purple;
                }
            } else {
                if( edge->type == Edge::TYPE_ODOM ) {
                    main_edge_marker.colors[i * 2] = main_edge_marker.colors[i * 2 + 1] = color_orange;
                } else {
                    main_edge_marker.colors[i * 2] = main_edge_marker.colors[i * 2 + 1] = color_pink;
                }
            }

            i++;
        }
    }

    // misc edges
    {
        visualization_msgs::Marker& misc_edge_marker = markers.markers[MARKER_MISC_EDGES];
        misc_edge_marker.header.frame_id             = map_frame_id;
        misc_edge_marker.header.stamp                = stamp;
        misc_edge_marker.id                          = MARKER_MISC_EDGES;
        misc_edge_marker.ns                          = "misc_edges";
        misc_edge_marker.type                        = visualization_msgs::Marker::LINE_LIST;

        misc_edge_marker.pose.orientation.w = 1.0;
        misc_edge_marker.scale.x            = 0.05;
        misc_edge_marker.points.resize( graph_slam->graph->edges().size() * 2 - markers.markers[MARKER_MAIN_EDGES].points.size() );
        misc_edge_marker.colors.resize( graph_slam->graph->edges().size() * 2 - markers.markers[MARKER_MAIN_EDGES].colors.size() );

        size_t i = 0;
        for( const auto& edge : graph_slam->graph->edges() ) {
            /*
            g2o::EdgeSE3*          edge_se3 = dynamic_cast<g2o::EdgeSE3*>( edge );
            if( edge_se3 ) {
                g2o::VertexSE3* v1  = dynamic_cast<g2o::VertexSE3*>( edge_se3->vertices()[0] );
                g2o::VertexSE3* v2  = dynamic_cast<g2o::VertexSE3*>( edge_se3->vertices()[1] );
                Eigen::Vector3d pt1 = v1->estimate().translation();
                Eigen::Vector3d pt2 = v2->estimate().translation();

                misc_edge_marker.points[i * 2].x     = pt1.x();
                misc_edge_marker.points[i * 2].y     = pt1.y();
                misc_edge_marker.points[i * 2].z     = pt1.z();
                misc_edge_marker.points[i * 2 + 1].x = pt2.x();
                misc_edge_marker.points[i * 2 + 1].y = pt2.y();
                misc_edge_marker.points[i * 2 + 1].z = pt2.z();

                double p1                       = static_cast<double>( v1->id() ) / graph_slam->graph->vertices().size();
                double p2                       = static_cast<double>( v2->id() ) / graph_slam->graph->vertices().size();
                misc_edge_marker.colors[i * 2].r     = 1.0 - p1;
                misc_edge_marker.colors[i * 2].g     = p1;
                misc_edge_marker.colors[i * 2].a     = 1.0;
                misc_edge_marker.colors[i * 2 + 1].r = 1.0 - p2;
                misc_edge_marker.colors[i * 2 + 1].g = p2;
                misc_edge_marker.colors[i * 2 + 1].a = 1.0;

                if( std::abs( v1->id() - v2->id() ) > 2 ) {
                    misc_edge_marker.points[i * 2].z += 0.5;
                    misc_edge_marker.points[i * 2 + 1].z += 0.5;
                }

                i++;
                continue;
            }
            */

            g2o::EdgeSE3Plane* edge_plane = dynamic_cast<g2o::EdgeSE3Plane*>( edge );
            if( edge_plane ) {
                g2o::VertexSE3* v1  = dynamic_cast<g2o::VertexSE3*>( edge_plane->vertices()[0] );
                Eigen::Vector3d pt1 = v1->estimate().translation();
                Eigen::Vector3d pt2( pt1.x(), pt1.y(), 0.0 );

                misc_edge_marker.points[i * 2].x     = pt1.x();
                misc_edge_marker.points[i * 2].y     = pt1.y();
                misc_edge_marker.points[i * 2].z     = pt1.z();
                misc_edge_marker.points[i * 2 + 1].x = pt2.x();
                misc_edge_marker.points[i * 2 + 1].y = pt2.y();
                misc_edge_marker.points[i * 2 + 1].z = pt2.z();

                misc_edge_marker.colors[i * 2].b     = 1.0;
                misc_edge_marker.colors[i * 2].a     = 1.0;
                misc_edge_marker.colors[i * 2 + 1].b = 1.0;
                misc_edge_marker.colors[i * 2 + 1].a = 1.0;

                i++;
                continue;
            }

            g2o::EdgeSE3PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE3PriorXY*>( edge );
            if( edge_priori_xy ) {
                g2o::VertexSE3* v1  = dynamic_cast<g2o::VertexSE3*>( edge_priori_xy->vertices()[0] );
                Eigen::Vector3d pt1 = v1->estimate().translation();
                Eigen::Vector3d pt2 = Eigen::Vector3d::Zero();
                pt2.head<2>()       = edge_priori_xy->measurement();

                misc_edge_marker.points[i * 2].x     = pt1.x();
                misc_edge_marker.points[i * 2].y     = pt1.y();
                misc_edge_marker.points[i * 2].z     = pt1.z() + 0.5;
                misc_edge_marker.points[i * 2 + 1].x = pt2.x();
                misc_edge_marker.points[i * 2 + 1].y = pt2.y();
                misc_edge_marker.points[i * 2 + 1].z = pt2.z() + 0.5;

                misc_edge_marker.colors[i * 2].r     = 1.0;
                misc_edge_marker.colors[i * 2].a     = 1.0;
                misc_edge_marker.colors[i * 2 + 1].r = 1.0;
                misc_edge_marker.colors[i * 2 + 1].a = 1.0;

                i++;
                continue;
            }

            g2o::EdgeSE3PriorXYZ* edge_priori_xyz = dynamic_cast<g2o::EdgeSE3PriorXYZ*>( edge );
            if( edge_priori_xyz ) {
                g2o::VertexSE3* v1  = dynamic_cast<g2o::VertexSE3*>( edge_priori_xyz->vertices()[0] );
                Eigen::Vector3d pt1 = v1->estimate().translation();
                Eigen::Vector3d pt2 = edge_priori_xyz->measurement();

                misc_edge_marker.points[i * 2].x     = pt1.x();
                misc_edge_marker.points[i * 2].y     = pt1.y();
                misc_edge_marker.points[i * 2].z     = pt1.z() + 0.5;
                misc_edge_marker.points[i * 2 + 1].x = pt2.x();
                misc_edge_marker.points[i * 2 + 1].y = pt2.y();
                misc_edge_marker.points[i * 2 + 1].z = pt2.z();

                misc_edge_marker.colors[i * 2].r     = 1.0;
                misc_edge_marker.colors[i * 2].a     = 1.0;
                misc_edge_marker.colors[i * 2 + 1].r = 1.0;
                misc_edge_marker.colors[i * 2 + 1].a = 1.0;

                i++;
                continue;
            }
        }

        if( i == 0 ) {
            markers.markers.resize( __NUM_MARKERS__ - 1 );
        }
    }

    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[MARKER_SPHERE];
    sphere_marker.header.frame_id             = map_frame_id;
    sphere_marker.header.stamp                = stamp;
    sphere_marker.ns                          = "loop_close_radius";
    sphere_marker.id                          = MARKER_SPHERE;
    sphere_marker.type                        = visualization_msgs::Marker::SPHERE;

    if( !keyframes.empty() ) {
        Eigen::Vector3d pos           = last_keyframe->node->estimate().translation();
        sphere_marker.pose.position.x = pos.x();
        sphere_marker.pose.position.y = pos.y();
        sphere_marker.pose.position.z = pos.z();
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector_distance_thresh * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;

    markers_pub.publish( markers );
}

}  // namespace hdl_graph_slam
#include <pcl/point_types.h>
#include <sc/Scancontext.h>

const int historyKeyframeSearchNum = 25;  // 2n+1 number of history key frames will be fused into a submap for loop closure


typedef pcl::PointXYZI PointType;
/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float  roll;
    float  pitch;
    float  yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Must be in global namespace
POINT_CLOUD_REGISTER_POINT_STRUCT( PointXYZIRPYT, ( float, x, x )( float, y, y )( float, z, z )( float, intensity, intensity )(
                                                      float, roll, roll )( float, pitch, pitch )( float, yaw, yaw )( double, time, time ) )

typedef PointXYZIRPYT PointTypePose;


namespace hdl_graph_slam {

/**
 * @brief Scan Context
 *
 */
class ScanContextLoopDetector {
public:
    ScanContextLoopDetector( rclcpp::Node::SharedPtr _node )
    {
        // Paremeters
        double filter_size = _node->get_parameter( "down_size_filter_history_leaf_size" ).as_double();

        // Scan Context
        downSizeFilterHistoryKeyFrames.setLeafSize( filter_size, filter_size, filter_size );
    }


    // adopted from https://github.com/irapkaist/SC-LeGO-LOAM/tree/master
    bool detectLoopClosure()
    {
        /*
         * 2. Scan context-based global localization
         */
        std::lock_guard<std::mutex> lock( mtx );

        SClatestSurfKeyFrameCloud->clear();
        SCnearHistorySurfKeyFrameCloud->clear();
        SCnearHistorySurfKeyFrameCloudDS->clear();

        latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
        SCclosestHistoryFrameID = -1;                               // init with -1
        auto detectResult       = scManager.detectLoopClosureID();  // first: nn index, second: yaw diff
        SCclosestHistoryFrameID = detectResult.first;
        yawDiffRad              = detectResult.second;  // not use for v1 (because pcl icp withi initial somthing wrong...)

        // if all close, reject
        if( SCclosestHistoryFrameID == -1 ) {
            return false;
        }

        // save latest key frames: query ptcloud (corner points + surface points)
        // NOTE: using "closestHistoryFrameID" to make same root of submap points to get a direct relative between the query point cloud
        // (latestSurfKeyFrameCloud) and the map (nearHistorySurfKeyFrameCloud). by giseop i.e., set the query point cloud within mapside's
        // local coordinate
        *SClatestSurfKeyFrameCloud += *transformPointCloud( cornerCloudKeyFrames[latestFrameIDLoopCloure],
                                                            &cloudKeyPoses6D->points[SCclosestHistoryFrameID] );
        *SClatestSurfKeyFrameCloud += *transformPointCloud( surfCloudKeyFrames[latestFrameIDLoopCloure],
                                                            &cloudKeyPoses6D->points[SCclosestHistoryFrameID] );

        pcl::PointCloud<PointType>::Ptr SChahaCloud( new pcl::PointCloud<PointType>() );
        int                             cloudSize = SClatestSurfKeyFrameCloud->points.size();
        for( int i = 0; i < cloudSize; ++i ) {
            if( (int)SClatestSurfKeyFrameCloud->points[i].intensity >= 0 ) {
                SChahaCloud->push_back( SClatestSurfKeyFrameCloud->points[i] );
            }
        }
        SClatestSurfKeyFrameCloud->clear();
        *SClatestSurfKeyFrameCloud = *SChahaCloud;

        // save history near key frames: map ptcloud (icp to query ptcloud)
        for( int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j ) {
            if( SCclosestHistoryFrameID + j < 0 || SCclosestHistoryFrameID + j > latestFrameIDLoopCloure ) continue;
            *SCnearHistorySurfKeyFrameCloud += *transformPointCloud( cornerCloudKeyFrames[SCclosestHistoryFrameID + j],
                                                                     &cloudKeyPoses6D->points[SCclosestHistoryFrameID + j] );
            *SCnearHistorySurfKeyFrameCloud += *transformPointCloud( surfCloudKeyFrames[SCclosestHistoryFrameID + j],
                                                                     &cloudKeyPoses6D->points[SCclosestHistoryFrameID + j] );
        }
        downSizeFilterHistoryKeyFrames.setInputCloud( SCnearHistorySurfKeyFrameCloud );
        downSizeFilterHistoryKeyFrames.filter( *SCnearHistorySurfKeyFrameCloudDS );

        // // optional: publish history near key frames
        // if (pubHistoryKeyFrames.getNumSubscribers() != 0){
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        //     cloudMsgTemp.header.frame_id = "/camera_init";
        //     pubHistoryKeyFrames.publish(cloudMsgTemp);
        // }

        return true;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud( pcl::PointCloud<PointType>::Ptr cloudIn )
    {
        // !!! DO NOT use pcl for point cloud transformation, results are not accurate
        // Reason: unkown
        pcl::PointCloud<PointType>::Ptr cloudOut( new pcl::PointCloud<PointType>() );

        PointType *pointFrom;
        PointType  pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize( cloudSize );

        for( int i = 0; i < cloudSize; ++i ) {
            pointFrom = &cloudIn->points[i];
            float x1  = ctYaw * pointFrom->x - stYaw * pointFrom->y;
            float y1  = stYaw * pointFrom->x + ctYaw * pointFrom->y;
            float z1  = pointFrom->z;

            float x2 = x1;
            float y2 = ctRoll * y1 - stRoll * z1;
            float z2 = stRoll * y1 + ctRoll * z1;

            pointTo.x         = ctPitch * x2 + stPitch * z2 + tInX;
            pointTo.y         = y2 + tInY;
            pointTo.z         = -stPitch * x2 + ctPitch * z2 + tInZ;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud( pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn )
    {
        pcl::PointCloud<PointType>::Ptr cloudOut( new pcl::PointCloud<PointType>() );

        PointType *pointFrom;
        PointType  pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize( cloudSize );

        for( int i = 0; i < cloudSize; ++i ) {
            pointFrom = &cloudIn->points[i];
            float x1  = cos( transformIn->yaw ) * pointFrom->x - sin( transformIn->yaw ) * pointFrom->y;
            float y1  = sin( transformIn->yaw ) * pointFrom->x + cos( transformIn->yaw ) * pointFrom->y;
            float z1  = pointFrom->z;

            float x2 = x1;
            float y2 = cos( transformIn->roll ) * y1 - sin( transformIn->roll ) * z1;
            float z2 = sin( transformIn->roll ) * y1 + cos( transformIn->roll ) * z1;

            pointTo.x         = cos( transformIn->pitch ) * x2 + sin( transformIn->pitch ) * z2 + transformIn->x;
            pointTo.y         = y2 + transformIn->y;
            pointTo.z         = -sin( transformIn->pitch ) * x2 + cos( transformIn->pitch ) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }


private:
    std::mutex mtx;

    SCManager scManager;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;

    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;


    pcl::PointCloud<PointType>::Ptr SClatestSurfKeyFrameCloud;

    pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloudDS;

    pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;  // for histor key frames of loop closure


    int   SCclosestHistoryFrameID;
    int   latestFrameIDLoopCloure;
    float yawDiffRad;

    float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
    float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;
};

}  // namespace hdl_graph_slam

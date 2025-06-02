/**
 * Created by Qingchen Bi on 2023/10/24
 */
#ifndef EXPLORATION_PLANNING_H
#define EXPLORATION_PLANNING_H

#include "path_planning.h"
#include "utils.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <two_opt.h>

namespace lrae_planner_ns {
struct ExactWindow {
    int oriindx, oriindy, winwithd, winheight;
    ExactWindow(int ox, int oy, int w, int h)
        : oriindx(ox), oriindy(oy), winwithd(w), winheight(h) {}
};

struct CellPoint {
    int indexX, indexY;
    int cellstate;
    geometry_msgs::Point position;
};

struct Centroid {
    geometry_msgs::Point centroidposition;
    utils_ns::Index2 cenindex;
    int unknownnum;
    bool isUnknown;
    int windowsoder;
    std::vector<utils_ns::ViewPoint> viewpoint_fir;
    std::vector<utils_ns::ViewPoint> viewpoint_sec;
    std::vector<utils_ns::ViewPoint> viewpoint_fin;
    std::multimap<float, utils_ns::ViewPoint*> evaluated_viewpoints;
    // [THÊM] Các trường mới cho centroid
    float positionX;
    float positionY;
};

struct CandiCentroidPair {
    Centroid centroidone;
    int oneindexincentroids;
    Centroid centroidtwo;
    int twoindexincentroids;
};

struct LastExState {
    bool findCenis;
    bool findNearCenPathis;
};

class ExplorationPlanning {
public:
    ExplorationPlanning(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
    ExplorationPlanning();
    ~ExplorationPlanning();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_;
    std::string ns_;
    bool has_map_;
    bool has_tra_map_;
    bool has_robot_position_;
    bool findCenIs_;
    bool go_home_;
    bool limit_max_square_;
    bool use_viewpoint_plan_;
    bool use_go_end_nearest_;
    int go_home_count_;
    int update_cen_count_;
    int update_cen_thre_;
    int pushcen_;
    double angle_pen_;
    double minrange_;
    double run_time_;
    double end_neacen_disthre_;
    double end_cur_disrate_;
    double unknown_num_thre_;
    double ori_x_;
    double ori_y_;
    double resolution_;
    int width_;
    int height_;
    geometry_msgs::Point ego_position_;
    geometry_msgs::Pose robot_pose_;
    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::OccupancyGrid traversibility_map_;
    std::string global_map_topic_;
    std::string traversibility_map_topic_;
    std::vector<Centroid> centroids_;
    std::vector<Centroid> last_centroids_;
    std::vector<Centroid> no_path_cens_;
    std::vector<Centroid> all_outwin_centroids_;
    Centroid last_near_cen_;
    CandiCentroidPair first_two_cen_;
    ExactWindow ExactWindow_;
    LastExState last_ex_state_;
    utils_ns::Index2 min_left_index_;
    utils_ns::Index2 max_right_index_;
    std::vector<CellPoint> in_win_Unpoints_;
    PathPlanning path_planner_;
    GraphSearch* cal_cost_planner_;
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    ros::Timer execution_timer_;
    ros::Subscriber global_map_sub_;
    ros::Subscriber traversibility_map_sub_;
    ros::Publisher exploration_path_pub_;
    ros::Publisher exploration_route_pub_;
    std::shared_ptr<visual::Marker> robot_position_marker_;
    std::shared_ptr<visual::Marker> centroids_marker_;
    std::shared_ptr<visual::Marker> windows_marker_;
    std::shared_ptr<visual::Marker> viewpoints_marker_;
    std::shared_ptr<visual::Marker> choose_centroids_marker_;
    std::shared_ptr<visual::Marker> choose_viewpoints_marker_;

    // [THÊM] Các biến mới để lưu trữ trọng số và trạng thái cập nhật bản đồ
    float weight_path_cost_;
    float weight_traver_degree_;
    float weight_disgridnum_;
    bool map_updated_;

    // [THÊM] Các biến cho metric
    float map_density_;
    float avg_centroid_distance_;
    float unknown_boundary_ratio_;
    float obstacle_threshold_;
    float boundary_threshold_;

    bool initialize();
    void getRobotPosition();
    std::vector<int> getRouteOrder();
    void traversibilityMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool getCentroid(const ExactWindow& ew);
    bool getCentroidHelper(const int ori_x, const int& ori_y, const int& w, const int& h, const int& winoder);
    bool assessUnknownRegion(Centroid* choosecentroid);
    void purgeViewpoint(std::vector<utils_ns::ViewPoint>& purgedViewpoints, Centroid& tmpCentroid);
    void execute(const ros::TimerEvent&);

    // [THÊM] Các hàm tính toán metric và trọng số
    void calculateMapDensity();
    void calculateAvgCentroidDistance();
    void calculateUnknownBoundaryRatio();
    float calculateWeightPathCost(float map_density, float avg_centroid_distance);
    float calculateWeightTraverDegree(float map_density);
    float calculateWeightDisgridnum(float unknown_boundary_ratio);

    inline bool isInBorder(const int& x, const int& y) {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    // [KHÔI PHỤC] Các hàm index2coord và coord2index
    geometry_msgs::Point index2coord(const int& ix, const int& iy) {
        geometry_msgs::Point coord;
        coord.x = resolution_ * double(ix) + ori_x_ + 0.5 * resolution_;
        coord.y = resolution_ * double(iy) + ori_y_ + 0.5 * resolution_;
        return coord;
    }

    utils_ns::Index2 coord2index(const geometry_msgs::Point& coord) {
        utils_ns::Index2 index;
        index.x = int((coord.x - ori_x_) / resolution_);
        index.y = int((coord.y - ori_y_) / resolution_);
        return index;
    }
};
} // namespace lrae_planner_ns

#endif
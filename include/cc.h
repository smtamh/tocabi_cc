#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random>
#include <fstream>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <chrono>
// #include <boost/shared_ptr.hpp>
#include "QP/QP_cartesian_velocity_wb.h"
#include "suhan_benchmark.h"


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    void ObjPoseCallback(const geometry_msgs::PoseConstPtr &msg);
    void JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg);
    void JointTargetCallback(const sensor_msgs::JointStatePtr &msg);
    void LHandPoseTargetCallback(const geometry_msgs::PoseStampedPtr &msg);
    void HeadPoseTargetCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RHandPoseTargetCallback(const geometry_msgs::PoseStampedPtr &msg);
    void TerminateCallback(const std_msgs::BoolPtr &msg);
    void HandMsgCallback(const std_msgs::Int32Ptr &msg);
    Eigen::Matrix3d Quat2rotmatrix(double q0, double q1, double q2, double q3);
    float PositionMapping( float haptic_pos, int i);
    bool saveImage(const sensor_msgs::ImageConstPtr &image_msg);
    void camera_img_callback(const sensor_msgs::ImageConstPtr &msg);
    // sensor_msgs::ImageConstPtr

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber joint_trajectory_sub;
    ros::Subscriber joint_target_sub;
    ros::Subscriber lhand_pose_target_sub;
    ros::Subscriber head_pose_target_sub;
    ros::Subscriber rhand_pose_target_sub;
    ros::Subscriber obj_pose_sub;
    ros::Publisher new_obj_pose_pub;
    ros::Publisher rrt_end_pub;

    Eigen::Vector3d obj_pos_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_qdot_;
    Eigen::Vector3d rhand_target_pos_;
    Eigen::Matrix3d rhand_target_rotm_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    bool init_time = false;
    int traj_index = 0;
    int num_waypoint = 0;

    bool joint_target_received = false;
    bool pose_target_received = false;
    double t_0_;
    std::vector<std::string> joint_names_;
    Eigen::VectorQd joint_target_;
    Eigen::VectorQd q_0_;
    Eigen::VectorQd qdot_0_;
    ros::Publisher terminate_pub;
    ros::Subscriber terminate_sub;
    std_msgs::Bool terminate_msg;
    ros::Publisher hand_open_pub;
    ros::Subscriber hand_open_sub;
    std_msgs::Int32 hand_open_msg;
    geometry_msgs::Pose new_obj_pose_msg_;

    void publishRobotPoses();
    ros::Publisher robot_pose_pub;
    ros::Publisher desired_robot_pose_pub_;
    ros::Publisher robot_joint_pub_;
    ros::Publisher desired_joint_pub_;
    geometry_msgs::PoseArray robot_pose_msg;
    geometry_msgs::PoseArray desired_robot_pose_msg_;

    void resetRobotPose(double duration);
    bool target_reached_ = false;
    Eigen::VectorQd q_init_;
    double time_init_ = 0.0;
    
    std::string folderPath, filePath_hand, filePath_joint, filePath_info;   // for hand pose and joint
    std::string folderPath_image, fileName_image, filePath_image;           // for images
    std::ofstream fout1, fout2, fout3;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    ros::Publisher camera_flag_pub;
    std_msgs::Bool camera_flag_msg;

    image_transport::Subscriber camera_image_sub;

    int camera_tick_ = 0;
    bool data_collect_start_ = false;
    bool make_dir = true;
    bool terminate = false;

    float distance_hand2obj;
    int prev_mode = 9;
    int num_data = 0;
    int num_test = 0;

    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv;

    std::unique_ptr<QP::CartesianVelocityWB> qp_cartesian_velocity_;
    
private:
    Eigen::VectorQd ControlVal_;
    map<std::string, int> JOINT_INDEX;
    const double hz_ = 2000.0;
    SuhanBenchmark cc_timer_;
};

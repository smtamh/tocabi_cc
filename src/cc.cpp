#include "cc.h"

std::mt19937_64 generator(0);
ros::Time init_;
bool is_reached = false;
int num_success = 0;
int num_trials = 0;

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    obj_pose_sub = nh_cc_.subscribe("/obj_pose", 1, &CustomController::ObjPoseCallback, this);
    joint_trajectory_sub = nh_cc_.subscribe("/tocabi/srmt/trajectory", 1, &CustomController::JointTrajectoryCallback, this);
    joint_target_sub = nh_cc_.subscribe("/tocabi/act/joint_target", 1, &CustomController::JointTargetCallback, this);
    lhand_pose_target_sub = nh_cc_.subscribe("/tocabi/act/lhand_pose_target", 1, &CustomController::LHandPoseTargetCallback, this);
    head_pose_target_sub = nh_cc_.subscribe("/tocabi/act/head_pose_target", 1, &CustomController::HeadPoseTargetCallback, this);
    rhand_pose_target_sub = nh_cc_.subscribe("/tocabi/act/rhand_pose_target", 1, &CustomController::RHandPoseTargetCallback, this);
    ControlVal_.setZero();
    image_transport::ImageTransport it(nh_cc_);
    robot_pose_pub = nh_cc_.advertise<geometry_msgs::PoseArray>("/tocabi/robot_poses", 1);
    robot_pose_msg.poses.resize(4);
    desired_robot_pose_pub_ = nh_cc_.advertise<geometry_msgs::PoseArray>("/tocabi/desired_robot_poses", 1);
    desired_robot_pose_msg_.poses.resize(3);
    robot_joint_pub_ = nh_cc_.advertise<sensor_msgs::JointState>("/tocabi/robot_joints", 1);
    desired_joint_pub_ = nh_cc_.advertise<sensor_msgs::JointState>("/tocabi/desired_joints", 1);
    camera_image_sub = it.subscribe("/mujoco_ros_interface/camera/image", 1, &CustomController::camera_img_callback, this);
    new_obj_pose_pub = nh_cc_.advertise<geometry_msgs::Pose>("/new_obj_pose", 1);
    terminate_pub = nh_cc_.advertise<std_msgs::Bool>("/tocabi/act/terminate", 1);
    terminate_sub = nh_cc_.subscribe("/tocabi/act/terminate", 1, &CustomController::TerminateCallback, this);
    hand_open_pub = nh_cc_.advertise<std_msgs::Int32>("/mujoco_ros_interface/hand_open", 1);
    hand_open_sub = nh_cc_.subscribe("/mujoco_ros_interface/hand_open", 1, &CustomController::HandMsgCallback, this);
    hand_open_msg.data = 0;

    nh_cc_.getParam("tocabi_cc/IL/num_data", num_data);
    nh_cc_.getParam("tocabi_cc/IL/num_test", num_test);

    for(int i = 0; i < MODEL_DOF; i++){
        JOINT_INDEX.insert({JOINT_NAME[i], i});
    }

    kp.setZero();
    kv.setZero();
    kp.diagonal() << 2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                    2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                    6000.0, 10000.0, 10000.0,
                    400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                    100.0, 100.0,
                    400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    kv.diagonal() << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                    15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                    200.0, 100.0, 100.0,
                    10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                    2.0, 2.0,
                    10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;

    qp_cartesian_velocity_ = std::make_unique<QP::CartesianVelocityWB>();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

double getRandomPosition(double minValue, double maxValue) 
{
    std::uniform_real_distribution<double> distribution(minValue, maxValue);
    return distribution(generator);
}

bool CustomController::saveImage(const sensor_msgs::ImageConstPtr &image_msg) {
    cv::Mat image;
    auto t_ = image_msg->header.stamp - init_;
    try
    {
      image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    }
    catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), "bgr8");
      return false;
    }

    if (!image.empty()) {
        // std::ostringstream oss;
        // oss << std::setw(9) << std::setfill('0') << t_.nsec;
        std::stringstream fileNameSS;
        // fileNameSS << "image_" << camera_tick_ << "_" << t_.sec << "_" << oss.str() << ".jpg";
        fileNameSS << "image_" << camera_tick_ << ".jpg";
        fileName_image = fileNameSS.str();

        std::stringstream filePathSS;
        filePathSS << folderPath_image << "/" << fileName_image;
        filePath_image = filePathSS.str();

        cv::imwrite(filePath_image, image);
        // ROS_INFO("Saved image %s", fileName.c_str());
    }
    else
    {
        ROS_ERROR("Couldn't save image, no data!");
        return false;
    }

    return true;
}

void CustomController::camera_img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    if(data_collect_start_){
        try
        {
            // ROS_INFO("Camera Callback. '%d'", camera_tick_);
            // save the image
            if (!saveImage(msg)) return;
            // ROS_INFO("Image Saved.");

            double t_ = (ros::Time::now() - init_).toSec();

            Eigen::Quaterniond hand_quat(rd_.link_[Right_Hand].rotm);
            Eigen::Quaterniond desired_hand_quat(rd_.link_[Right_Hand].rot_desired);
            Eigen::Quaterniond head_quat(rd_.link_[Head].rotm);
            // write data to the file
            fout1 << t_ << "\t"
                << rd_.link_[Right_Hand].xpos(0) << "\t" << rd_.link_[Right_Hand].xpos(1) << "\t" << rd_.link_[Right_Hand].xpos(2) << "\t" 
                << hand_quat.coeffs()[0] << "\t"<< hand_quat.coeffs()[1] << "\t" << hand_quat.coeffs()[2] << "\t" << hand_quat.coeffs()[3] << "\t" 
                << rd_.link_[Head].xpos(0) << "\t" << rd_.link_[Head].xpos(1) << "\t" << rd_.link_[Head].xpos(2) << "\t" 
                << head_quat.coeffs()[0] << "\t"<< head_quat.coeffs()[1] << "\t" << head_quat.coeffs()[2] << "\t" << head_quat.coeffs()[3] << "\t" 
                << rd_.link_[Right_Hand].x_desired(0) << "\t" << rd_.link_[Right_Hand].x_desired(1) << "\t" << rd_.link_[Right_Hand].x_desired(2) << "\t"
                << desired_hand_quat.coeffs()[0] << "\t"<< desired_hand_quat.coeffs()[1] << "\t" << desired_hand_quat.coeffs()[2] << "\t" << desired_hand_quat.coeffs()[3] << "\t"
                << hand_open_msg.data << endl;
            
            fout2 << t_ << "\t";
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << rd_.q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << desired_q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << rd_.q_dot_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << desired_qdot_[i] << "\t";
            }
            fout2 << endl;

            camera_tick_++;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}


void CustomController::computeSlow()
{
    //MODE 6,7,8,9 is reserved for cc
    //MODE 6: Homing & new obj pose
    //MODE 7: joint trajectory tracking for RRT & data collection
    //MODE 8: joint target tracking for ACT
    //MODE 9: right hand task space control with QP by jh
    queue_cc_.callAvailable(ros::WallDuration());
    static int pub_cnt = 0;
    if(pub_cnt == 40){
        publishRobotPoses();
        pub_cnt = 0;
    }
    pub_cnt++;
    
    if (rd_.tc_.mode == 6)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            q_init_ = rd_.q_;
            time_init_ = rd_.control_time_ + 0.5;   // wait 0.5s for hand closed
            if (prev_mode > 7) is_reached = (obj_pos_(2) > 1.0);
        }
        // move to ready pose
        double duration = 2.0;
        resetRobotPose(duration);

        if (rd_.control_time_ > time_init_ + duration)
        {
            // check and record whether the task has succeeded
            if (prev_mode == 7) is_reached = (obj_pos_(2) > 1.0);
            fout3 << is_reached << endl;
            std::cout<< (is_reached ? "Success" : "Fail") << std::endl;
            if (is_reached) num_success++;
            num_trials++;

            // end data collection
            data_collect_start_ = false;
            if(fout1.is_open()==true)
            {
                fout1.close();
            }
            if(fout2.is_open()==true)
            {
                fout2.close();
            }
            if(fout3.is_open()==true)
            {
                fout3.close();
            }

            // open hand
            hand_open_msg.data = 0;
            hand_open_pub.publish(hand_open_msg);

            if ((prev_mode == 7 && num_success < num_data) || (prev_mode > 7 && num_trials < num_test)){
            // relocate object to new position
            const double minX = -0.05;  // -0.1;
            const double maxX = 0.05;   // 0.1;
            const double minY = -0.05;  // -0.1;
            const double maxY = 0.05;   // 0.1;

            new_obj_pose_msg_.position.x = getRandomPosition(minX, maxX);
            new_obj_pose_msg_.position.y = getRandomPosition(minY, maxY);
            new_obj_pose_msg_.position.z = 0.0;
            double yaw = getRandomPosition(0, 0.75);
            Eigen::Quaterniond quaternion(DyrosMath::rotateWithZ(yaw));
            new_obj_pose_msg_.orientation.x = quaternion.coeffs()[0];
            new_obj_pose_msg_.orientation.y = quaternion.coeffs()[1];
            new_obj_pose_msg_.orientation.z = quaternion.coeffs()[2];
            new_obj_pose_msg_.orientation.w = quaternion.coeffs()[3];
            new_obj_pose_pub.publish(new_obj_pose_msg_);
            }

            rd_.tc_.mode = prev_mode;
            rd_.tc_init = true;
        }
    }

    else if (rd_.tc_.mode == 7)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 7 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 7;

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        
            // initialize file for data collection
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            // folderPathSS << "/home/hokyun20/2024winter_ws/src/camera_pubsub/" << ros::Time::now().sec;
            folderPathSS << "/home/smtamh/catkin_ws/src/data/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout1.open(filePath_hand);
            fout1 << "ros_time" << "\t" 
                << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                << "hand_pose_qx" << "\t" << "hand_pose_qy" << "\t" << "hand_pose_qz" << "hand_pose_qw" << "\t" 
                << "head_pose_x" << "\t" << "head_pose_y" << "\t" << "heand_pose_z" << "\t" 
                << "head_pose_qx" << "\t" << "head_pose_qy" << "\t" << "head_pose_qz" << "head_pose_qw" << "\t" 
                << "desired_hand_pose_x" << "\t" << "desired_hand_pose_y" << "\t" << "desired_hand_pose_z" << "\t" 
                << "desired_hand_pose_qx" << "\t" << "desired_hand_pose_qy" << "\t" << "desired_hand_pose_qz" << "desired_hand_pose_qw" << "\t" 
                << "hand_open_state" << endl; 
            if(!fout1.is_open()){
                ROS_ERROR("Couldn't open text file1");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "ros_time" << "\t" 
                << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                << "current_joint_vel" << "\t" << "target_joint_vel" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "obj_pose_x" << "\t" << "obj_pose_y" << "\t" << "obj_pose_z" << "\t" << "success" << endl;
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
            
        if(data_collect_start_){
            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            // get target q, qvel from trajectory waypoints & cubic spline
            double t_ = (ros::Time::now() - init_).toSec();
            if (t_ > points[traj_index+1].time_from_start.toSec()){
                traj_index++;
            }
            if (traj_index < num_waypoint-1){
                double t_0 = points[traj_index].time_from_start.toSec();
                auto   p_0 = points[traj_index].positions;
                auto   v_0 = points[traj_index].velocities;
                double t_f = points[traj_index+1].time_from_start.toSec();
                auto   p_f = points[traj_index+1].positions;
                auto   v_f = points[traj_index+1].velocities;
                for(int i = 0; i < joint_names_.size(); i++){                    
                    desired_q_[JOINT_INDEX[joint_names_[i]]] = DyrosMath::cubic(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                    desired_qdot_[JOINT_INDEX[joint_names_[i]]] = DyrosMath::cubicDot(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                }
            }
            else{
                // end of trajectory following
                for(int i = 0; i < joint_names_.size(); i++){                    
                    desired_q_[JOINT_INDEX[joint_names_[i]]] = points[traj_index].positions[i];
                    desired_qdot_[JOINT_INDEX[joint_names_[i]]] = points[traj_index].velocities[i];
                }
                rd_.tc_.mode = 6;
                rd_.tc_init = true;

                // close hand to grab the object
                hand_open_msg.data = 1;
                hand_open_pub.publish(hand_open_msg);
            }
        }
        // torque PD control
        rd_.torque_desired =  kp * (desired_q_ - rd_.q_) + kv * (desired_qdot_ - rd_.q_dot_);
    }

    else if (rd_.tc_.mode == 8)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 8;

            camera_tick_ = 0;
            joint_target_received = false;

            desired_q_ = rd_.q_;
            joint_target_ = rd_.q_;
            desired_qdot_.setZero();
        
            // initialize file for data collection
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            // folderPathSS << "/home/hokyun20/2024winter_ws/src/camera_pubsub/" << ros::Time::now().sec;
            folderPathSS << "/home/smtamh/catkin_ws/src/result/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout1.open(filePath_hand);
            fout1 << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                  << "hand_pose_qx" << "\t" << "hand_pose_qy" << "\t" << "hand_pose_qz" << "\t" << "hand_pose_qw" << "\t"
                  << "head_pose_x" << "\t" << "head_pose_y" << "\t" << "heand_pose_z" << "\t" 
                  << "head_pose_qx" << "\t" << "head_pose_qy" << "\t" << "head_pose_qz" << "head_pose_qw" << "\t" 
                  << "desired_hand_pose_x" << "\t" << "desired_hand_pose_y" << "\t" << "desired_hand_pose_z" << "\t" 
                  << "desired_hand_pose_qx" << "\t" << "desired_hand_pose_qy" << "\t" << "desired_hand_pose_qz" << "desired_hand_pose_qw" << "\t" 
                  << "hand_open_state" << endl; 
            if(!fout1.is_open()){
                ROS_ERROR("Couldn't open text file1");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << "\t" 
                    << "recieved_target" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "obj_pose_x" << "\t" << "obj_pose_y" << "\t" << "obj_pose_z" << "\t" << "success" << endl;
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
        
        if(joint_target_received){
            if(!data_collect_start_){
                data_collect_start_ = true;
                fout3 << obj_pos_(0) << "\t" << obj_pos_(1) << "\t" << obj_pos_(2) << "\t";
                init_ = ros::Time::now();
            }
            // compute current distance between hand and obj for terminal condition check
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            double t_ = rd_.control_time_;
            double dt = 1/30;
            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = DyrosMath::cubic(t_, t_0_, t_0_+dt, q_0_[i], joint_target_[i], qdot_0_[i], 0);
                desired_qdot_[i] = DyrosMath::cubicDot(t_, t_0_, t_0_+dt, q_0_[i], joint_target_[i], qdot_0_[i], 0);
            }
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }
    
    else if (rd_.tc_.mode == 9)
    {
        cc_timer_.reset();
        if (rd_.tc_init){
            std::cout << "mode 9 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 9;
            
            t_0_ = rd_.control_time_;
            rd_.q_desired = rd_.q_;
            rd_.q_dot_desired.setZero();
            q_0_ = rd_.q_;
            joint_target_ = rd_.q_;
            desired_qdot_.setZero();
            camera_tick_ = 0;
            pose_target_received = false;
            
            rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].xpos;
            rd_.link_[Left_Hand].rot_desired = rd_.link_[Left_Hand].rotm;
            rd_.link_[Head].x_desired = rd_.link_[Head].xpos;
            rd_.link_[Head].rot_desired = rd_.link_[Head].rotm;
            rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].xpos;
            rd_.link_[Right_Hand].rot_desired = rd_.link_[Right_Hand].rotm;

            // get desired pose from GUI taskcommand msg
            Vector3d lhand_pos_desired_l;
            lhand_pos_desired_l << rd_.tc_.l_x, rd_.tc_.l_y, rd_.tc_.l_z;
            rd_.link_[Left_Hand].x_desired += lhand_pos_desired_l;
            rd_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.l_roll * DEG2RAD) * DyrosMath::rotateWithY(rd_.tc_.l_pitch * DEG2RAD) * DyrosMath::rotateWithZ(rd_.tc_.l_yaw * DEG2RAD) * DyrosMath::Euler2rot(0, M_PI_2, M_PI_2).transpose();
            Vector3d rhand_pos_desired_l;
            rhand_pos_desired_l << rd_.tc_.r_x, rd_.tc_.r_y, rd_.tc_.r_z;
            rd_.link_[Right_Hand].x_desired += rhand_pos_desired_l;
            rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * DEG2RAD) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * DEG2RAD) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * DEG2RAD) * DyrosMath::Euler2rot(0, M_PI_2, M_PI).transpose();

            // initialize file for data collection
            init_ = ros::Time::now();
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            // folderPathSS << "/home/hokyun20/2024winter_ws/src/camera_pubsub/" << ros::Time::now().sec;
            folderPathSS << "/home/smtamh/catkin_ws/src/result/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout1.open(filePath_hand);
            fout1 << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                  << "hand_pose_qx" << "\t" << "hand_pose_qy" << "\t" << "hand_pose_qz" << "\t" << "hand_pose_qw" << "\t"
                  << "head_pose_x" << "\t" << "head_pose_y" << "\t" << "heand_pose_z" << "\t" 
                  << "head_pose_qx" << "\t" << "head_pose_qy" << "\t" << "head_pose_qz" << "head_pose_qw" << "\t" 
                  << "desired_hand_pose_x" << "\t" << "desired_hand_pose_y" << "\t" << "desired_hand_pose_z" << "\t" 
                  << "desired_hand_pose_qx" << "\t" << "desired_hand_pose_qy" << "\t" << "desired_hand_pose_qz" << "desired_hand_pose_qw" << "\t" 
                  << "hand_open_state" << endl; 
            if(!fout1.is_open()){
                ROS_ERROR("Couldn't open text file1");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << "\t" 
                    << "recieved_target" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "obj_pose_x" << "\t" << "obj_pose_y" << "\t" << "obj_pose_z" << "\t" << "success" << endl;
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
            
            new_obj_pose_msg_.position.x = 0.0;
            new_obj_pose_msg_.position.y = 0.0;
            new_obj_pose_msg_.position.z = 0.0;
        }

        // get desired pose from /tocabi/act/pose_target topic callback
        if (pose_target_received){
            if(!data_collect_start_){
                data_collect_start_ = true;
                fout3 << obj_pos_(0) << "\t" << obj_pos_(1) << "\t" << obj_pos_(2) << "\t";
                init_ = ros::Time::now();
            }
        }
        // new_obj_pose_msg_.position.y += 0.05/hz_;
        // new_obj_pose_pub.publish(new_obj_pose_msg_);

        // ==================== QP ====================
        Eigen::VectorXd gain_diag = Eigen::VectorXd::Zero(6);
        gain_diag << 5, 5, 5, 5, 5, 5;
        // set QP problem
        Eigen::VectorXd lhand_error = Eigen::VectorXd::Zero(6);
        lhand_error.head(3) = rd_.link_[Left_Hand].x_desired - rd_.link_[Left_Hand].xpos;
        lhand_error.tail(3) = DyrosMath::getPhi(rd_.link_[Left_Hand].rot_desired, rd_.link_[Left_Hand].rotm);
        Eigen::VectorXd head_error = Eigen::VectorXd::Zero(6);
        head_error.head(3) = rd_.link_[Head].x_desired - rd_.link_[Head].xpos;
        head_error.tail(3) = DyrosMath::getPhi(rd_.link_[Head].rot_desired, rd_.link_[Head].rotm);
        Eigen::VectorXd rhand_error = Eigen::VectorXd::Zero(6);
        rhand_error.head(3) = rd_.link_[Right_Hand].x_desired - rd_.link_[Right_Hand].xpos;
        rhand_error.tail(3) = DyrosMath::getPhi(rd_.link_[Right_Hand].rot_desired, rd_.link_[Right_Hand].rotm);

        Eigen::Matrix<double, 18, 21> Jacobian;
        Jacobian.setZero();
        Jacobian.block(0, 0, 6, 21) = rd_.link_[Left_Hand].Jac().block(0, 18, 6, 21);
        Jacobian.block(6, 0, 6, 21) = rd_.link_[Head].Jac().block(0, 18, 6, 21);
        Jacobian.block(12, 0, 6, 21) = rd_.link_[Right_Hand].Jac().block(0, 18, 6, 21);
        qp_cartesian_velocity_->setCurrentState(rd_.q_.tail(21), rd_.q_dot_desired.tail(21), Jacobian);
        qp_cartesian_velocity_->setDesiredEEVel(gain_diag.asDiagonal()*lhand_error, gain_diag.asDiagonal()*head_error, gain_diag.asDiagonal()*rhand_error);

        // solve QP
        Eigen::Matrix<double, 21, 1> opt_qdot;
        if(!qp_cartesian_velocity_->getOptJointVel(opt_qdot))
        {
            ROS_INFO("QP did not solved!!!");
        }

        rd_.q_dot_desired.tail(21) = opt_qdot;
        rd_.q_desired.tail(21) += opt_qdot / hz_;

        // torque PD control
        WBC::SetContact(rd_, 1, 1);
        rd_.torque_grav = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
        // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);
        rd_.torque_desired = kp * (rd_.q_desired - rd_.q_) + kv * (rd_.q_dot_desired - rd_.q_dot_) + rd_.torque_grav;

        double elapsed_time = cc_timer_.elapsedAndReset();
        // cout << elapsed_time * 1000 << " ms" << endl;
    } 
}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, 0.0,
            0.3, 0.3, 1.5, -1.27, -1, 0.0, -1, 0.0,
            0.0, 0.3,
            0.0, 0.3, -1.57, 1.2, 1.57, -1.5, -0.4, 0.2;
    for (int i = 0; i < MODEL_DOF; i++)
    {
        desired_q_(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ + duration, q_init_(i), q_target(i), 0.0, 0.0);
        desired_qdot_(i) = DyrosMath::cubicDot(rd_.control_time_, time_init_, time_init_ + duration, q_init_(i), q_target(i), 0.0, 0.0);
    }

    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
    rd_.torque_desired =  kp * (desired_q_ - rd_.q_) + kv * (desired_qdot_ - rd_.q_dot_);
}


void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {
    }
    else if (rd_.tc_.mode == 7)
    {
    }
}

void CustomController::ObjPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
    float obj_x = msg->position.x;
    float obj_y = msg->position.y;
    float obj_z = msg->position.z;
 
    obj_pos_ << obj_x, obj_y, obj_z;
    // std::cout << "Obj Pose subscribed!" << std::endl;
}

void CustomController::JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg)
{
    joint_names_ = msg->joint_names;
    points = msg->points;
    traj_index = 0;
    num_waypoint = points.size();

    data_collect_start_ = true;
    camera_tick_ = 0;
    init_ = ros::Time::now();
    fout3 << obj_pos_(0) << "\t" << obj_pos_(1) << "\t" << obj_pos_(2) << "\t";
}

void CustomController::JointTargetCallback(const sensor_msgs::JointStatePtr &msg)
{
    t_0_ = rd_.control_time_;
    q_0_ = rd_.q_desired;
    qdot_0_ = rd_.q_dot_;
    joint_names_ = msg->name;
    for(int i = 0; i < joint_names_.size(); i++){
        int j = JOINT_INDEX[joint_names_[i]];
        joint_target_[j] = msg->position[i];
    }
    joint_target_received = true;
}

void CustomController::LHandPoseTargetCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    rd_.link_[Left_Hand].x_desired << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    rd_.link_[Left_Hand].rot_desired = CustomController::Quat2rotmatrix(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void CustomController::HeadPoseTargetCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    rd_.link_[Head].x_desired << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    rd_.link_[Head].rot_desired = CustomController::Quat2rotmatrix(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

void CustomController::RHandPoseTargetCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    rd_.link_[Right_Hand].x_desired << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    rd_.link_[Right_Hand].rot_desired = CustomController::Quat2rotmatrix(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    pose_target_received = true;
}

void CustomController::publishRobotPoses()
{
    // =============== Current Robot pose ===============
    robot_pose_msg.header.stamp = ros::Time::now();
    robot_pose_msg.header.frame_id = "world";
    // pelvis
    robot_pose_msg.poses[0].position.x = rd_.link_[Pelvis].xpos(0);
    robot_pose_msg.poses[0].position.y = rd_.link_[Pelvis].xpos(1);
    robot_pose_msg.poses[0].position.z = rd_.link_[Pelvis].xpos(2);
    Eigen::Quaterniond quat_pelvis(rd_.link_[Pelvis].rotm);
    robot_pose_msg.poses[0].orientation.x = quat_pelvis.x();
    robot_pose_msg.poses[0].orientation.y = quat_pelvis.y();
    robot_pose_msg.poses[0].orientation.z = quat_pelvis.z();
    robot_pose_msg.poses[0].orientation.w = quat_pelvis.w();
    // left hand
    robot_pose_msg.poses[1].position.x = rd_.link_[Left_Hand].xpos(0);
    robot_pose_msg.poses[1].position.y = rd_.link_[Left_Hand].xpos(1);
    robot_pose_msg.poses[1].position.z = rd_.link_[Left_Hand].xpos(2);
    Eigen::Quaterniond quat_lhand(rd_.link_[Left_Hand].rotm);
    robot_pose_msg.poses[1].orientation.x = quat_lhand.x();
    robot_pose_msg.poses[1].orientation.y = quat_lhand.y();
    robot_pose_msg.poses[1].orientation.z = quat_lhand.z();
    robot_pose_msg.poses[1].orientation.w = quat_lhand.w();
    // head
    robot_pose_msg.poses[2].position.x = rd_.link_[Head].xpos(0);
    robot_pose_msg.poses[2].position.y = rd_.link_[Head].xpos(1);
    robot_pose_msg.poses[2].position.z = rd_.link_[Head].xpos(2);
    Eigen::Quaterniond q_head(rd_.link_[Head].rotm);
    robot_pose_msg.poses[2].orientation.x = q_head.x();
    robot_pose_msg.poses[2].orientation.y = q_head.y();
    robot_pose_msg.poses[2].orientation.z = q_head.z();
    robot_pose_msg.poses[2].orientation.w = q_head.w();
    // right hand
    robot_pose_msg.poses[3].position.x = rd_.link_[Right_Hand].xpos(0);
    robot_pose_msg.poses[3].position.y = rd_.link_[Right_Hand].xpos(1);
    robot_pose_msg.poses[3].position.z = rd_.link_[Right_Hand].xpos(2);
    Eigen::Quaterniond quat_rhand(rd_.link_[Right_Hand].rotm);
    robot_pose_msg.poses[3].orientation.x = quat_rhand.x();
    robot_pose_msg.poses[3].orientation.y = quat_rhand.y();
    robot_pose_msg.poses[3].orientation.z = quat_rhand.z();
    robot_pose_msg.poses[3].orientation.w = quat_rhand.w();

    // =============== Desired Robot pose ===============
    desired_robot_pose_msg_.header.stamp = ros::Time::now();
    desired_robot_pose_msg_.header.frame_id = "world";
    // left hand
    desired_robot_pose_msg_.poses[0].position.x = rd_.link_[Left_Hand].x_desired(0);
    desired_robot_pose_msg_.poses[0].position.y = rd_.link_[Left_Hand].x_desired(1);
    desired_robot_pose_msg_.poses[0].position.z = rd_.link_[Left_Hand].x_desired(2);
    Eigen::Quaterniond desired_quat_lhand(rd_.link_[Left_Hand].rot_desired);
    desired_robot_pose_msg_.poses[0].orientation.x = desired_quat_lhand.x();
    desired_robot_pose_msg_.poses[0].orientation.y = desired_quat_lhand.y();
    desired_robot_pose_msg_.poses[0].orientation.z = desired_quat_lhand.z();
    desired_robot_pose_msg_.poses[0].orientation.w = desired_quat_lhand.w();
    // head
    desired_robot_pose_msg_.poses[1].position.x = rd_.link_[Head].x_desired(0);
    desired_robot_pose_msg_.poses[1].position.y = rd_.link_[Head].x_desired(1);
    desired_robot_pose_msg_.poses[1].position.z = rd_.link_[Head].x_desired(2);
    Eigen::Quaterniond desired_q_head(rd_.link_[Head].rot_desired);
    desired_robot_pose_msg_.poses[1].orientation.x = desired_q_head.x();
    desired_robot_pose_msg_.poses[1].orientation.y = desired_q_head.y();
    desired_robot_pose_msg_.poses[1].orientation.z = desired_q_head.z();
    desired_robot_pose_msg_.poses[1].orientation.w = desired_q_head.w();
    // right hand
    desired_robot_pose_msg_.poses[2].position.x = rd_.link_[Right_Hand].x_desired(0);
    desired_robot_pose_msg_.poses[2].position.y = rd_.link_[Right_Hand].x_desired(1);
    desired_robot_pose_msg_.poses[2].position.z = rd_.link_[Right_Hand].x_desired(2);
    Eigen::Quaterniond desired_quat_rhand(rd_.link_[Right_Hand].rot_desired);
    desired_robot_pose_msg_.poses[2].orientation.x = desired_quat_rhand.x();
    desired_robot_pose_msg_.poses[2].orientation.y = desired_quat_rhand.y();
    desired_robot_pose_msg_.poses[2].orientation.z = desired_quat_rhand.z();
    desired_robot_pose_msg_.poses[2].orientation.w = desired_quat_rhand.w();

    // ===============  joint ===============
    sensor_msgs::JointState robot_joint_msg, desired_joint_msg;

    robot_joint_msg.header.stamp = ros::Time::now();
    desired_joint_msg.header.stamp = ros::Time::now();

    robot_joint_msg.name.resize(MODEL_DOF);
    robot_joint_msg.position.resize(MODEL_DOF);
    robot_joint_msg.velocity.resize(MODEL_DOF);
    robot_joint_msg.effort.resize(MODEL_DOF);

    desired_joint_msg.name.resize(MODEL_DOF);
    desired_joint_msg.position.resize(MODEL_DOF);
    desired_joint_msg.velocity.resize(MODEL_DOF);
    desired_joint_msg.effort.resize(MODEL_DOF);

    for(size_t i=0; i<MODEL_DOF; i++)
    {
        robot_joint_msg.name[i] = JOINT_NAME[i];
        robot_joint_msg.position[i] = rd_.q_(i);
        robot_joint_msg.velocity[i] = rd_.q_dot_(i);
        robot_joint_msg.effort[i] = rd_.torque_elmo_(i);

        desired_joint_msg.name[i] = JOINT_NAME[i];
        desired_joint_msg.position[i] = rd_.q_desired(i);
        desired_joint_msg.velocity[i] = rd_.q_dot_desired(i);
        desired_joint_msg.effort[i] = rd_.torque_desired(i);
    }

    robot_pose_pub.publish(robot_pose_msg);
    desired_robot_pose_pub_.publish(desired_robot_pose_msg_);
    robot_joint_pub_.publish(robot_joint_msg);
    desired_joint_pub_.publish(desired_joint_msg);
}

void CustomController::TerminateCallback(const std_msgs::BoolPtr &msg)
{
    cout << msg->data << endl;
    if (msg->data){
        cout << "Terminate Callback called!" << endl;
        data_collect_start_ = false;
        rd_.tc_.mode = 6;
        rd_.tc_init = true;
    }
}

void CustomController::HandMsgCallback(const std_msgs::Int32Ptr &msg)
{
    hand_open_msg.data = msg->data;
}

Eigen::Matrix3d CustomController::Quat2rotmatrix(double q0, double q1, double q2, double q3)
{
    double r00 = 2 * (q0 * q0 + q1 * q1) - 1 ;
    double r01 = 2 * (q1 * q2 - q0 * q3) ;
    double r02 = 2 * (q1 * q3 + q0 * q2) ;

    double r10 = 2 * (q1 * q2 + q0 * q3) ;
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1 ;
    double r12 = 2 * (q2 * q3 - q0 * q1) ;

    double r20 = 2 * (q1 * q3 - q0 * q2) ;
    double r21 = 2 * (q2 * q3 + q0 * q1) ;
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1 ;

    Eigen::Matrix3d rot_matrix;
    rot_matrix << r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22;
    return rot_matrix;
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}
#include "cc.h"

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();

    // for assigning target
    target_pose_sub_ = nh_cc_.subscribe("/tocabi/target_robot_poses", 1, &CustomController::TargetPosesCallback, this);
    target_rhand_pose_sub_ = nh_cc_.subscribe("/tocabi/target_rhand_pose", 1, &CustomController::TargetRHandPoseCallback, this);

    // for act (lyh)
    joint_target_sub_ = nh_cc_.subscribe("/tocabi/act/joint_target", 1, &CustomController::TargetJointCallback, this);
    lhand_pose_target_sub_ = nh_cc_.subscribe("/tocabi/act/lhand_pose_target", 1, &CustomController::TargetLHandPoseCallback, this);
    head_pose_target_sub_ = nh_cc_.subscribe("/tocabi/act/head_pose_target", 1, &CustomController::TargetHeadPoseCallback, this);
    rhand_pose_target_sub_ = nh_cc_.subscribe("/tocabi/act/rhand_pose_target", 1, &CustomController::TargetRHandPoseCallback, this);

    // for data logging
    robot_pose_pub = nh_cc_.advertise<geometry_msgs::PoseArray>("/tocabi/robot_poses", 1);
    robot_pose_msg.poses.resize(4);
    desired_robot_pose_pub_ = nh_cc_.advertise<geometry_msgs::PoseArray>("/tocabi/desired_robot_poses", 1);
    desired_robot_pose_msg_.poses.resize(3);
    robot_joint_pub_ = nh_cc_.advertise<sensor_msgs::JointState>("/tocabi/robot_joints", 1);
    desired_joint_pub_ = nh_cc_.advertise<sensor_msgs::JointState>("/tocabi/desired_joints", 1);

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
    target_robot_poses_local_.resize(4); // left hand, upper body, head, right hand
    target_robot_poses_world_.resize(4); // left hand, upper body, head, right hand
    for (auto &pose : target_robot_poses_local_) pose.setIdentity();
    for (auto &pose : target_robot_poses_world_) pose.setIdentity();

    for(int i = 0; i < MODEL_DOF; i++)
    {
        JOINT_INDEX.insert({JOINT_NAME[i], i});
        std::cout << "joint " << i << ": " << JOINT_NAME[i] <<std::endl;

    }

}

void CustomController::computeSlow()
{
    // MODE 6,7,8 are reserved for cc
    // MODE 6: joint command
    // MODE 7: CLIK for right hand
    // MODE 8: QP for head, right and left hand
    // MODE 9: HQP OSF for upper body, head, right and left hand
    queue_cc_.callAvailable(ros::WallDuration());

    if (rd_.tc_init) //한번만 실행됩니다(gui)
    {
        std::cout << "mode" << rd_.tc_.mode << " init!" << std::endl;
        rd_.tc_init = false;

        time_init_ = rd_.control_time_;

        rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init;
        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init;
        rd_.link_[Upper_Body].x_desired = rd_.link_[Upper_Body].x_init;
        rd_.link_[Head].x_desired = rd_.link_[Head].x_init;

        rd_.link_[Left_Hand].rot_desired = rd_.link_[Left_Hand].rot_init;
        rd_.link_[Right_Hand].rot_desired = rd_.link_[Right_Hand].rot_init;
        rd_.link_[Head].rot_desired = rd_.link_[Head].rot_init;

        rd_.q_desired = rd_.q_;
        rd_.q_dot_desired = rd_.q_dot_;
        q_init_ = rd_.q_;
        qdot_init_ = rd_.q_dot_;
        q_desired_ = rd_.q_;
        q_dot_desired_ = rd_.q_dot_;

        // ================== tc_. -> target_robot_poses_local_ ==================
        // left hand
        target_robot_poses_local_[0].translation() << rd_.tc_.l_x, 
                                            rd_.tc_.l_y, 
                                            rd_.tc_.l_z;
        target_robot_poses_local_[0].linear() = DyrosMath::rotateWithX(rd_.tc_.l_roll * DEG2RAD) * 
                                        DyrosMath::rotateWithY(rd_.tc_.l_pitch * DEG2RAD) * 
                                        DyrosMath::rotateWithZ(rd_.tc_.l_yaw * DEG2RAD);

        // upper body (only for orientation)
        target_robot_poses_local_[1].translation().setZero();
        target_robot_poses_local_[1].linear() = DyrosMath::rotateWithX(rd_.tc_.roll * DEG2RAD) * 
                                        DyrosMath::rotateWithY(rd_.tc_.pitch * DEG2RAD) * 
                                        DyrosMath::rotateWithZ(rd_.tc_.yaw * DEG2RAD);
        // head: no input

        // right hand
        target_robot_poses_local_[3].translation() << rd_.tc_.r_x, 
                                            rd_.tc_.r_y, 
                                            rd_.tc_.r_z;
        target_robot_poses_local_[3].linear() = DyrosMath::rotateWithX(rd_.tc_.r_roll * DEG2RAD) * 
                                        DyrosMath::rotateWithY(rd_.tc_.r_pitch * DEG2RAD) * 
                                        DyrosMath::rotateWithZ(rd_.tc_.r_yaw * DEG2RAD);
        
        // ================================================================

        // ====================== Initialize target_robot_poses_world_ ======================
        // left hand
        target_robot_poses_world_[0].translation() = rd_.link_[Left_Hand].xpos;
        target_robot_poses_world_[0].linear() = rd_.link_[Left_Hand].rotm;

        // upperbody
        target_robot_poses_world_[1].translation() = rd_.link_[Upper_Body].xpos;
        target_robot_poses_world_[1].linear() = rd_.link_[Upper_Body].rotm;

        // head
        target_robot_poses_world_[2].translation() = rd_.link_[Head].xpos;
        target_robot_poses_world_[2].linear() = rd_.link_[Head].rotm;

        // right hand
        target_robot_poses_world_[3].translation() = rd_.link_[Right_Hand].xpos;
        target_robot_poses_world_[3].linear() = rd_.link_[Right_Hand].rotm;
        // ==================================================================================
    }

    // ==================== Desired EE pose ==================== 
    if(is_world_base_)
    {
        // Left Hand
        rd_.link_[Left_Hand].x_desired = target_robot_poses_world_[0].translation();
        rd_.link_[Left_Hand].rot_desired = target_robot_poses_world_[0].rotation();
        
        // Upper body
        rd_.link_[Upper_Body].x_desired = target_robot_poses_world_[1].translation();
        rd_.link_[Upper_Body].rot_desired = target_robot_poses_world_[1].rotation();
        
        // Head
        rd_.link_[Head].x_desired = target_robot_poses_world_[2].translation();
        rd_.link_[Head].rot_desired = target_robot_poses_world_[2].rotation();
        
        // Right Hand
        rd_.link_[Right_Hand].x_desired = target_robot_poses_world_[3].translation();
        rd_.link_[Right_Hand].rot_desired = target_robot_poses_world_[3].rotation();
    }
    else
    {
        // Left Hand
        rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init + target_robot_poses_local_[0].translation();
        rd_.link_[Left_Hand].rot_desired = target_robot_poses_local_[0].rotation();
        
        // Upper body (only for orientation)
        rd_.link_[Upper_Body].rot_desired = target_robot_poses_local_[1].rotation();
        
        // Head (only for orientation)
        rd_.link_[Head].rot_desired = target_robot_poses_local_[2].rotation();
        
        // Right Hand
        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init + target_robot_poses_local_[3].translation();
        rd_.link_[Right_Hand].rot_desired = target_robot_poses_local_[3].rotation() * DyrosMath::Euler2rot(0, M_PI/2, M_PI).transpose();
    }
    // Set trajectory
    rd_.link_[Left_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].x_desired);
    rd_.link_[Left_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].rot_desired, false);
    rd_.link_[Upper_Body].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].x_desired);
    rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].rot_desired, false);
    rd_.link_[Head].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Head].x_desired);
    rd_.link_[Head].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Head].rot_desired, false);
    rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].x_desired);
    rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].rot_desired, false);
    // ========================================================== 
    
    if (rd_.tc_.mode == 6)
    {
        if(is_q_target_)
        {
            time_init_ = rd_.control_time_;
            is_q_target_ = false;
            q_init_ = rd_.q_desired;
            qdot_init_ = rd_.q_dot_desired;
        }

        double duration = 0.1;
        
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.q_desired[i] =  DyrosMath::cubic(rd_.control_time_, time_init_, time_init_+duration, q_init_[i], q_desired_[i], qdot_init_[i], q_dot_desired_[i]);
            rd_.q_dot_desired[i] =  DyrosMath::cubicDot(rd_.control_time_, time_init_, time_init_+duration, q_init_[i], q_desired_[i], qdot_init_[i], q_dot_desired_[i]);
        }
        WBC::SetContact(rd_, 1, 1); 
        rd_.torque_grav = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
        rd_.torque_desired = kp * (rd_.q_desired - rd_.q_) + kv * (rd_.q_dot_desired - rd_.q_dot_) + rd_.torque_grav;
    }
    else if (rd_.tc_.mode == 7)
    {
        // ==================== CLIK ====================
        // Right Hand
        Eigen::MatrixXd J_rhand = rd_.link_[Right_Hand].Jac();
        Eigen::MatrixXd J_rhand_rarm = J_rhand.block(0, 6+33-8, 6, 8);
        Eigen::MatrixXd J_rhand_rarm_pinv = J_rhand_rarm.transpose()*DyrosMath::pinv_COD(J_rhand_rarm*J_rhand_rarm.transpose() + 1e-4*Eigen::MatrixXd::Identity(6,6));

        Eigen::VectorXd x_error = Eigen::VectorXd::Zero(6);
        x_error.head(3) = rd_.link_[Right_Hand].x_traj - rd_.link_[Right_Hand].xpos;
        x_error.tail(3) = DyrosMath::getPhi(rd_.link_[Right_Hand].r_traj, rd_.link_[Right_Hand].rotm);

        Eigen::VectorXd gain_diag = Eigen::VectorXd::Zero(6);
        gain_diag << 10, 10, 10, 5, 5, 5;

        Eigen::VectorXd qdot_desired_rarm = J_rhand_rarm_pinv * gain_diag.asDiagonal() * x_error;

        rd_.q_dot_desired.setZero();
        rd_.q_dot_desired.tail(8) = qdot_desired_rarm;

        rd_.q_desired += rd_.q_dot_desired / hz_;
        // ===============================================

        rd_.torque_grav = WBC::GravityCompensationTorque(rd_);
        rd_.torque_desired = kp * (rd_.q_desired - rd_.q_) + kv * (rd_.q_dot_desired - rd_.q_dot_) + rd_.torque_grav;
    }
    else if (rd_.tc_.mode == 8)
    {
        // ==================== QP ====================
        Eigen::VectorXd gain_diag = Eigen::VectorXd::Zero(6);
        gain_diag << 5, 5, 5, 5, 5, 5;
        Eigen::VectorXd head_gain_diag = Eigen::VectorXd::Zero(6);
        head_gain_diag << 1, 1, 1, 1, 1, 1;
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
        qp_cartesian_velocity_->setDesiredEEVel(gain_diag.asDiagonal()*lhand_error, head_gain_diag.asDiagonal()*head_error, gain_diag.asDiagonal()*rhand_error);

        // solve QP
        Eigen::Matrix<double, 21, 1> opt_qdot;
        if(!qp_cartesian_velocity_->getOptJointVel(opt_qdot))
        {
            ROS_INFO("QP did not solved!!!");
        }

        rd_.q_dot_desired.setZero();
        rd_.q_dot_desired.tail(21) = opt_qdot;
        
        rd_.q_desired += rd_.q_dot_desired / hz_;
        if(rd_.control_time_ - time_init_ < 2.0)
        {
            rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(rd_.control_time_, time_init_, time_init_ + 2.0, q_init_, rd_.q_desired, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero());
        }
        // ============================================

        // torque PD control
        WBC::SetContact(rd_, 1, 1); 
        rd_.torque_grav = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
        // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);
        rd_.torque_desired = kp * (rd_.q_desired - rd_.q_) + kv * (rd_.q_dot_desired - rd_.q_dot_) + rd_.torque_grav;
    }  
    else if (rd_.tc_.mode == 9)
    {
        // Assume two foot are contact
        WBC::SetContact(rd_, 1, 1);

        // Set Gains
        if (rd_.tc_.customTaskGain)
        {
            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Left_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        }
        // ==================== Desired EE pose ==================== 
        // 1. Pelvis
        rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
        rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithY(rd_.tc_.pelv_pitch * DEG2RAD) * DyrosMath::rotateWithZ(rd_.link_[Pelvis].yaw_init);
        rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
        rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
        // ========================================================== 

        // ==================== HQP ====================
        rd_.torque_grav = WBC::GravityCompensationTorque(rd_);
        VectorQd torque_Task = rd_.torque_grav;

        // 1. Pelvis
        TaskSpace ts1(6);
        Eigen::MatrixXd Jtask1 = rd_.link_[Pelvis].JacCOM();
        Eigen::VectorXd fstar1 = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);

        ts1.Update(Jtask1,fstar1);
        WBC::CalcJKT(rd_, ts1);
        WBC::CalcTaskNull(rd_, ts1);
        static CQuadraticProgram task_qp1;
        if(WBC::TaskControlHQP(rd_, ts1, task_qp1, torque_Task, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), is_qp_first_))
        {
            torque_Task = ts1.torque_h_ + rd_.torque_grav;

            // 2. Right Hand
            TaskSpace ts2(6);
            Eigen::MatrixXd Jtask2 = rd_.link_[Right_Hand].Jac();
            Eigen::VectorXd fstar2 = WBC::GetFstar6d(rd_.link_[Right_Hand], true);

            ts2.Update(Jtask2, fstar2);
            WBC::CalcJKT(rd_, ts2);
            WBC::CalcTaskNull(rd_, ts2);
            static CQuadraticProgram task_qp2;
            if(WBC::TaskControlHQP(rd_, ts2, task_qp2, torque_Task, ts1.Null_task, is_qp_first_))
            {
                torque_Task = ts1.torque_h_ + 
                            ts1.Null_task * ts2.torque_h_ + 
                            rd_.torque_grav;

                // 3. Left Hand
                TaskSpace ts3(6);
                Eigen::MatrixXd Jtask3 = rd_.link_[Left_Hand].Jac();
                Eigen::VectorXd fstar3 = WBC::GetFstar6d(rd_.link_[Left_Hand], true);

                ts3.Update(Jtask3, fstar3);
                WBC::CalcJKT(rd_, ts3);
                WBC::CalcTaskNull(rd_, ts3);
                static CQuadraticProgram task_qp3;
                if(WBC::TaskControlHQP(rd_, ts3, task_qp3, torque_Task, ts1.Null_task * ts2.Null_task, is_qp_first_))
                {
                    torque_Task = ts1.torque_h_ + 
                                ts1.Null_task * ts2.torque_h_ + 
                                ts1.Null_task * ts2.Null_task * ts3.torque_h_ + 
                                rd_.torque_grav;

                    // 4. Upper body
                    TaskSpace ts4(3);
                    Eigen::MatrixXd Jtask4 = rd_.link_[Upper_Body].Jac().bottomRows(3);
                    Eigen::VectorXd fstar4 = WBC::GetFstarRot(rd_.link_[Upper_Body]);

                    ts4.Update(Jtask4, fstar4);
                    WBC::CalcJKT(rd_, ts4);
                    WBC::CalcTaskNull(rd_, ts4);
                    static CQuadraticProgram task_qp4;
                    if(WBC::TaskControlHQP(rd_, ts4, task_qp4, torque_Task, ts1.Null_task * ts2.Null_task * ts3.Null_task, is_qp_first_))
                    {
                        torque_Task = ts1.torque_h_ + 
                                      ts1.Null_task * ts2.torque_h_ + 
                                      ts1.Null_task * ts2.Null_task * ts3.torque_h_ + 
                                      ts1.Null_task * ts2.Null_task * ts3.Null_task * ts4.torque_h_ + 
                                      rd_.torque_grav;

                        // 5. Head
                        TaskSpace ts5(3);
                        Eigen::MatrixXd Jtask5 = rd_.link_[Head].Jac().bottomRows(3);
                        Eigen::VectorXd fstar5 = WBC::GetFstarRot(rd_.link_[Head]);

                        ts5.Update(Jtask5, fstar5);
                        WBC::CalcJKT(rd_, ts5);
                        static CQuadraticProgram task_qp5;
                        if(WBC::TaskControlHQP(rd_, ts5, task_qp5, torque_Task, ts1.Null_task * ts2.Null_task * ts3.Null_task * ts4.Null_task, is_qp_first_))
                        {
                            torque_Task = ts1.torque_h_ + 
                                          ts1.Null_task * ts2.torque_h_ + 
                                          ts1.Null_task * ts2.Null_task * ts3.torque_h_ + 
                                          ts1.Null_task * ts2.Null_task * ts3.Null_task * ts4.torque_h_ + 
                                          ts1.Null_task * ts2.Null_task * ts3.Null_task * ts4.Null_task * ts5.torque_h_ + 
                                          rd_.torque_grav;
                        }
                    }
                }


            }

        }
        if(is_qp_first_) is_qp_first_ = false;
        // =============================================
        
        // holding torque command (except upper body)
        VectorQd torque_pos_hold;
        torque_pos_hold = kp * (q_init_ - rd_.q_) + kv * ( - rd_.q_dot_);
        torque_pos_hold += WBC::ContactForceRedistributionTorque(rd_, rd_.torque_grav);
        torque_pos_hold.tail(21).setZero(); // set upper body torque as blanck

        // upper body torque command
        VectorQd torque_upper_body;
        torque_upper_body.setZero();
        torque_upper_body.tail(21) = WBC::ContactForceRedistributionTorque(rd_, torque_Task).tail(21);

        for (int i=12;i<MODEL_DOF;i++) // for upper body
        {
            rd_.torque_desired[i] = torque_pos_hold[i] + torque_upper_body[i];
        }
    }

    static int pub_cnt = 0;
    if(pub_cnt == 20)
    {
        // =============== Robot pose ===============
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

        robot_pose_pub.publish(robot_pose_msg);
        // ==========================================

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

        desired_robot_pose_pub_.publish(desired_robot_pose_msg_);

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
        robot_joint_pub_.publish(robot_joint_msg);
        desired_joint_pub_.publish(desired_joint_msg);

        pub_cnt = 0;
    }
    pub_cnt++;

}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;
    q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, 0.0,
            0.0, -0.3, 1.57, -1.2, -1.57, 1.5, 0.4, -0.2,
            0.0, 0.3,
            0.0, 0.3, -1.57, 1.2, 1.57, -1.5, -0.4, 0.2;

    for (int i = 0; i <MODEL_DOF; i++)
    {
        q_cubic(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ +duration, q_init_(i), q_target(i), 0.0, 0.0);
    }
    rd_.torque_grav = WBC::GravityCompensationTorque(rd_);
    for (int i = 0; i < MODEL_DOF; i++)
    {
        rd_.torque_desired[i] = rd_.pos_kp_v[i] * (q_cubic[i] - rd_.q_[i]) - rd_.pos_kv_v[i] * rd_.q_dot_[i] + rd_.torque_grav(i);
    }

    WBC::SetContact(rd_, 1, 1);
    rd_.torque_desired =  rd_.torque_desired + WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
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

void CustomController::TargetPosesCallback(const geometry_msgs::PoseArrayPtr &msg)
{
    std::vector<Eigen::Affine3d> temp_target_poses;
    temp_target_poses.resize(4);

    // left hand
    temp_target_poses[0].translation() << msg->poses[0].position.x,
                                          msg->poses[0].position.y,
                                          msg->poses[0].position.z;
    Eigen::Quaterniond target_quat_lhand(msg->poses[0].orientation.w,
                                         msg->poses[0].orientation.x,
                                         msg->poses[0].orientation.y,
                                         msg->poses[0].orientation.z);
    temp_target_poses[0].linear() = target_quat_lhand.toRotationMatrix();

    // upper body
    temp_target_poses[1].translation() << msg->poses[1].position.x,
                                          msg->poses[1].position.y,
                                          msg->poses[1].position.z;
    Eigen::Quaterniond target_quat_upper(msg->poses[1].orientation.w,
                                        msg->poses[1].orientation.x,
                                        msg->poses[1].orientation.y,
                                        msg->poses[1].orientation.z);
    temp_target_poses[1].linear() = target_quat_upper.toRotationMatrix();

    // head
    temp_target_poses[2].translation() << msg->poses[2].position.x,
                                          msg->poses[2].position.y,
                                          msg->poses[2].position.z;
    Eigen::Quaterniond target_quat_head(msg->poses[2].orientation.w,
                                        msg->poses[2].orientation.x,
                                        msg->poses[2].orientation.y,
                                        msg->poses[2].orientation.z);
    temp_target_poses[2].linear() = target_quat_head.toRotationMatrix();

    // right hand
    temp_target_poses[3].translation() << msg->poses[3].position.x,
                                        msg->poses[3].position.y,
                                        msg->poses[3].position.z;
    Eigen::Quaterniond target_quat_rhand(msg->poses[3].orientation.w,
                                        msg->poses[3].orientation.x,
                                        msg->poses[3].orientation.y,
                                        msg->poses[3].orientation.z);
    temp_target_poses[3].linear() = target_quat_rhand.toRotationMatrix();

    if(is_world_base_) target_robot_poses_world_ = temp_target_poses;
    else  target_robot_poses_local_ = temp_target_poses;
}

void CustomController::TargetLHandPoseCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    // left hand
    Eigen::Affine3d temp_target_lhand_pose;
    temp_target_lhand_pose.translation() << msg->pose.position.x,
                                            msg->pose.position.y,
                                            msg->pose.position.z;
    Eigen::Quaterniond target_quat_lhand(msg->pose.orientation.w,
                                         msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z);
    temp_target_lhand_pose.linear() = target_quat_lhand.toRotationMatrix();

    if(is_world_base_) target_robot_poses_world_[0] = temp_target_lhand_pose;
    else  target_robot_poses_local_[0] = temp_target_lhand_pose;
}

void CustomController::TargetHeadPoseCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    // head
    Eigen::Affine3d temp_target_head_pose;
    temp_target_head_pose.translation() << msg->pose.position.x,
                                            msg->pose.position.y,
                                            msg->pose.position.z;
    Eigen::Quaterniond target_quat_head(msg->pose.orientation.w,
                                         msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z);
    temp_target_head_pose.linear() = target_quat_head.toRotationMatrix();

    if(is_world_base_) target_robot_poses_world_[2] = temp_target_head_pose;
    else  target_robot_poses_local_[2] = temp_target_head_pose;
}

void CustomController::TargetRHandPoseCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    // right hand
    Eigen::Affine3d temp_target_rhand_pose;
    temp_target_rhand_pose.translation() << msg->pose.position.x,
                                            msg->pose.position.y,
                                            msg->pose.position.z;
    Eigen::Quaterniond target_quat_rhand(msg->pose.orientation.w,
                                         msg->pose.orientation.x,
                                         msg->pose.orientation.y,
                                         msg->pose.orientation.z);
    temp_target_rhand_pose.linear() = target_quat_rhand.toRotationMatrix();

    if(is_world_base_) target_robot_poses_world_[3] = temp_target_rhand_pose;
    else  target_robot_poses_local_[3] = temp_target_rhand_pose;
}

void CustomController::TargetJointCallback(const sensor_msgs::JointStatePtr &msg)
{
    if(rd_.tc_.mode == 6)
    {
        if (msg->name.size() != msg->position.size())
        {
            ROS_ERROR_STREAM("[TJC] name(" << msg->name.size()
                            << ") ≠ position(" << msg->position.size() << ") - drop");
            return;                 // 필수 조건 위배 : 즉시 반환
        }

        const bool has_velocity = (msg->velocity.size() == msg->name.size());

        if (!has_velocity)          // velocity 배열이 있는데 길이가 다르면 경고
        {
            ROS_WARN_STREAM("[TJC] velocity length("
                            << msg->velocity.size()
                            << ") ≠ name/position (" << msg->name.size()
                            << ")  →  모든 속도 = 0 처리");
        }

        /* ───── 2. q_desired_/q_dot_desired_ 크기 보장 ──────────────── */
        if (q_desired_.size()     != MODEL_DOF) q_desired_.setZero(MODEL_DOF);
        if (q_dot_desired_.size() != MODEL_DOF) q_dot_desired_.setZero(MODEL_DOF);

        /* ───── 3. 각 관절 갱신 ─────────────────────────────────────── */
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string& jname = msg->name[i];
            auto it = JOINT_INDEX.find(jname);
            if (it == JOINT_INDEX.end())
            {
                ROS_WARN_STREAM("[TJC] unknown joint \"" << jname << "\" (skip)");
                continue;
            }
            const int idx = it->second;
            if (idx < 0 || idx >= MODEL_DOF)
            {
                ROS_ERROR_STREAM("[TJC] index out of range for \"" << jname
                                << "\" idx=" << idx);
                continue;
            }

            /* position 은 반드시 존재 */
            q_desired_[idx] = msg->position[i];

            /* velocity 는 선택 사항 */
            double vel = 0.0;
            if (has_velocity) vel = msg->velocity[i];
            q_dot_desired_[idx] = vel;

            ROS_DEBUG_STREAM("[TJC] " << jname
                            << "  idx=" << idx
                            << "  pos=" << std::fixed << std::setprecision(4)
                            << msg->position[i]
                            << "  vel=" << vel);
        }

        /* ───── 4. 완료 로그 & 플래그 ──────────────────────────────── */
        is_q_target_ = true;
        ROS_INFO_STREAM("[TJC] processed "
                        << msg->name.size() << " joints  "
                        << "(velocities " << (msg->velocity.size()==0 ? "absent" : "used") << ")");
    }
    else
    {
        ROS_ERROR("Mode is not 6");
    }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}
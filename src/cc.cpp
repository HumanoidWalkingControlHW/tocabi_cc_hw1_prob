#include "cc.h"
using namespace TOCABI;

//TODO 1
//Set the path for data.txt file to your directory.
//ex. ofstream data1("/home/econom2-20/data/hw/data1.txt");
//The absolute path to the directory can be check by typing "pwd" command to the terminal.
ofstream data1("/ ... /data1.txt");
ofstream data2("/ ... /data2.txt");
ofstream data3("/ ... /data3.txt");
ofstream data4("/ ... /data4.txt");
ofstream data5("/ ... /data5.txt");
ofstream data6("/ ... /data6.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd) 
{
    nh_cc_.setCallbackQueue(&queue_cc_);

    ControlVal_.setZero();

    bool urdfmode = false;
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    // if (urdfmode)
    // {
    //     urdf_path = desc_package_path + "/dyros_tocabi_ankleRollDamping.urdf";
    // }
    // else
    // {
    //     urdf_path = desc_package_path + "/dyros_tocabi.urdf";
    // }

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_d_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_c_, true, false);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

double CustomController::getMpcFrequency() const
{
    return mpc_freq;
}

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());

    if (rd_.tc_.mode == 6)
    {   
        if (is_mode_6_init == true)
        {
            if (initial_flag == 0)
            {
                setGains();
                walkingParameterSetting();

                q_init_ = rd_.q_;

                walking_enable_ = true;

                cout << "COMPUTESLOW MODE 6 IS NOW INITIALIZED" << endl;
                cout << "TIME: "<< rd_.control_time_ << endl; 

                WBC::SetContact(rd_, 1, 1);
                Gravity_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);
                atb_grav_update_ = false;

                initial_flag = 1;
                is_mode_6_init = false;
            }

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_fast_ = Gravity_;
                atb_grav_update_ = false;
            }
        }

        moveInitialPose();

        rd_.torque_desired = Kp_diag * (q_ref_ - rd_.q_) - Kd_diag * rd_.q_dot_;
    }
    else if (rd_.tc_.mode == 7)
    {
        if(walking_enable_ == true)
        {
            if (is_mode_7_init == true)
            {
                q_init_ = rd_.q_;
                q_leg_desired_ = rd_.q_.segment(0,12);

                contact_wrench_torque.setZero();
                lfoot_contact_wrench.setZero();
                rfoot_contact_wrench.setZero();

                cout << "COMPUTESLOW MODE 7 IS NOW INITIALIZED" << endl;
                initial_flag = 0;
                is_mode_7_init = false;
            }

            updateInitialState();
            getRobotState();
            floatToSupportFootstep();

            if(current_step_num_ < total_step_num_)
            {
                circling_motion();
                computeIkControl(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_leg_desired_);
                q_ref_.segment(0, 12) = q_leg_desired_;

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_fast_ = Gravity_;
                    atb_grav_update_ = false;
                }

                if (walking_tick < 1.0 * hz_)
                {
                    q_ref_.segment(0, 12) = DyrosMath::cubicVector<12>(walking_tick, 0, 1.0 * hz_, q_init_.segment(0,12), q_leg_desired_, Eigen::Vector12d::Zero(), Eigen::Vector12d::Zero());
                }

                updateNextStepTime();
                q_prev_ = rd_.q_;

            }
        }
        else
        {
            std::cout << "==================== WALKING FINISH ====================" << std::endl;
        }

        ///////////////////////////////FINAL TORQUE COMMAND/////////////////////////////
        rd_.torque_desired = Kp_diag * (q_ref_ - rd_.q_) - Kd_diag * rd_.q_dot_;
        ///////////////////////////////////////////////////////////////////////////////
    }
}

void CustomController::computeFast()
{
    /*
    if (rd_.tc_.mode == 6)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1);

            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

                atb_grav_update_ = true;
                Gravity_ = Gravity_local;
                atb_grav_update_ = false;
            }
        }
    }
    else if (rd_.tc_.mode == 7)
    {
        if (walking_enable_ == true)
        {
            if (current_step_num_ < total_step_num_)
            {
                gravityCalculate();
            }
        }
        else
        {
            WBC::SetContact(rd_, 1, 1);
            int support_foot;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                support_foot = 1;
            }
            else
            {
                support_foot = 0;
            }
             
            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, support_foot);

                atb_grav_update_ = true;
                Gravity_ = Gravity_local;
                atb_grav_update_ = false;
            }
        }
    }
    */
}

void CustomController::computeThread3()
{

}

void CustomController::computePlanner()
{

}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::setGains()
{
    Kp.setZero(); Kp_diag.setZero();
    Kd.setZero(); Kd_diag.setZero();

    ////////////////
    // JOINT GAIN //
    Kp(0) = 2000.0;
    Kd(0) = 20.0; // Left Hip yaw
    Kp(1) = 5000.0;
    Kd(1) = 55.0; // Left Hip roll
    Kp(2) = 4000.0;
    Kd(2) = 45.0; // Left Hip pitch
    Kp(3) = 3700.0;
    Kd(3) = 40.0; // Left Knee pitch
    Kp(4) = 4000.0; 
    Kd(4) = 65.0; // Left Ankle pitch
    Kp(5) = 4000.0; 
    Kd(5) = 65.0; // Left Ankle roll /5000 

    Kp(6) = 2000.0;
    Kd(6) = 20.0; // Right Hip yaw
    Kp(7) = 5000.0;
    Kd(7) = 55.0; // Right Hip roll 
    Kp(8) = 4000.0;
    Kd(8) = 45.0; // Right Hip pitch
    Kp(9) = 3700.0;
    Kd(9) = 40.0; // Right Knee pitch
    Kp(10) = 4000.0; 
    Kd(10) = 65.0; // Right Ankle pitch
    Kp(11) = 4000.0;
    Kd(11) = 65.0; // Right Ankle roll

    Kp(12) = 6000.0;
    Kd(12) = 200.0; // Waist yaw
    Kp(13) = 10000.0;
    Kd(13) = 100.0; // Waist pitch
    Kp(14) = 10000.0;
    Kd(14) = 100.0; // Waist roll

    Kp(15) = 400.0;
    Kd(15) = 10.0;
    Kp(16) = 800.0;
    Kd(16) = 10.0;
    Kp(17) = 400.0;
    Kd(17) = 10.0;
    Kp(18) = 400.0;
    Kd(18) = 10.0;
    Kp(19) = 250.0;
    Kd(19) = 2.5;
    Kp(20) = 250.0;
    Kd(20) = 2.0;
    Kp(21) = 50.0;
    Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0;
    Kd(22) = 2.0; // Left Wrist

    Kp(23) = 50.0;
    Kd(23) = 2.0; // Neck
    Kp(24) = 50.0;
    Kd(24) = 2.0; // Neck

    Kp(25) = 400.0;
    Kd(25) = 10.0;
    Kp(26) = 800.0;
    Kd(26) = 10.0;
    Kp(27) = 400.0;
    Kd(27) = 10.0;
    Kp(28) = 400.0;
    Kd(28) = 10.0;
    Kp(29) = 250.0;
    Kd(29) = 2.5;
    Kp(30) = 250.0;
    Kd(30) = 2.0;
    Kp(31) = 50.0;
    Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0;
    Kd(32) = 2.0; // Right Wrist

    Kp_diag = Kp.asDiagonal();
    Kd_diag = Kd.asDiagonal();

    /////////////////////
    // JOINT POS LIMIT //
    joint_limit_l_.setZero(MODEL_DOF);
    joint_limit_h_.setZero(MODEL_DOF);

    for (int i = 0; i < 12; i++)
    {
        joint_limit_l_(i) = -180 * DEG2RAD;
        joint_limit_h_(i) = 180 * DEG2RAD;
    }

    joint_limit_l_(3) = 20 * DEG2RAD;    // Left knee pitch
    joint_limit_h_(3) = 110 * DEG2RAD;   // Left knee pitch
    joint_limit_l_(9) = 20 * DEG2RAD;    // Right knee pitch
    joint_limit_h_(9) = 110 * DEG2RAD;   // Right knee pitch
    joint_limit_l_(12) =-30 * DEG2RAD;
    joint_limit_h_(12) = 30 * DEG2RAD;
    joint_limit_l_(13) =-15 * DEG2RAD;
    joint_limit_h_(13) = 15 * DEG2RAD;
    joint_limit_l_(14) =-15 * DEG2RAD;
    joint_limit_h_(14) = 15 * DEG2RAD;
    joint_limit_l_(15) =-30 * DEG2RAD;
    joint_limit_h_(15) = 20 * DEG2RAD;
    joint_limit_l_(16) =-50 * DEG2RAD;
    joint_limit_h_(16) = 50 * DEG2RAD;
    joint_limit_l_(17) = 45 * DEG2RAD;
    joint_limit_h_(17) = 65 * DEG2RAD;
    joint_limit_l_(18) =-90 * DEG2RAD;
    joint_limit_h_(18) =-65 * DEG2RAD;
    joint_limit_l_(19) =-150 * DEG2RAD;
    joint_limit_h_(19) =-10 * DEG2RAD;
    joint_limit_l_(20) =-180 * DEG2RAD;
    joint_limit_h_(20) = 180 * DEG2RAD;
    joint_limit_l_(21) =-70 * DEG2RAD;
    joint_limit_h_(21) = 70 * DEG2RAD;
    joint_limit_l_(22) =-60 * DEG2RAD;
    joint_limit_h_(22) = 60 * DEG2RAD;
    //HEAD
    joint_limit_l_(23) =-80 * DEG2RAD;
    joint_limit_h_(23) = 80 * DEG2RAD;
    joint_limit_l_(24) =-40 * DEG2RAD;
    joint_limit_h_(24) = 30 * DEG2RAD;
    //RIGHT ARM
    joint_limit_l_(25) =-20 * DEG2RAD;
    joint_limit_h_(25) = 30 * DEG2RAD;
    joint_limit_l_(26) =-50 * DEG2RAD;
    joint_limit_h_(26) = 50 * DEG2RAD;
    joint_limit_l_(27) =-65 * DEG2RAD;
    joint_limit_h_(27) =-45 * DEG2RAD;
    joint_limit_l_(28) = 65 * DEG2RAD;
    joint_limit_h_(28) = 90 * DEG2RAD;
    joint_limit_l_(29) = 10 * DEG2RAD;
    joint_limit_h_(29) = 150 * DEG2RAD;
    joint_limit_l_(30) =-180 * DEG2RAD;
    joint_limit_h_(30) = 180 * DEG2RAD;
    joint_limit_l_(31) =-70 * DEG2RAD;
    joint_limit_h_(31) = 70 * DEG2RAD;
    joint_limit_l_(32) =-60 * DEG2RAD;
    joint_limit_h_(32) = 60 * DEG2RAD;

    /////////////////////
    // JOINT VEL LIMIT //
    joint_vel_limit_l_.setZero(MODEL_DOF);
    joint_vel_limit_h_.setZero(MODEL_DOF);
    for (int i = 0; i < 12; i++)
    {
        joint_vel_limit_l_(i) = -2 * M_PI;
        joint_vel_limit_h_(i) =  2 * M_PI;
    }
    for (int i = 12; i < 33; i++)
    {
        joint_vel_limit_l_(i) = 0.0; // *2
        joint_vel_limit_h_(i) = 0.0; // *2
    }

    joint_vel_limit_l_(12) =-M_PI * 3.0;
    joint_vel_limit_h_(12) = M_PI * 3.0;
    joint_vel_limit_l_(13) =-M_PI / 6.0;
    joint_vel_limit_h_(13) = M_PI / 6.0;
    joint_vel_limit_l_(14) =-M_PI / 6.0;
    joint_vel_limit_h_(14) = M_PI / 6.0;
    joint_vel_limit_l_(15) =-M_PI * 1.5;
    joint_vel_limit_h_(15) = M_PI * 1.5;
    joint_vel_limit_l_(16) =-M_PI * 1.5;
    joint_vel_limit_h_(16) = M_PI * 1.5;
    joint_vel_limit_l_(17) =-M_PI * 1.5;
    joint_vel_limit_h_(17) = M_PI * 1.5;
    joint_vel_limit_l_(25) =-M_PI * 1.5;
    joint_vel_limit_h_(25) = M_PI * 1.5;
    joint_vel_limit_l_(26) =-M_PI * 1.5;
    joint_vel_limit_h_(26) = M_PI * 1.5;
    joint_vel_limit_l_(27) =-M_PI * 1.5;
    joint_vel_limit_h_(27) = M_PI * 1.5;
}

void CustomController::moveInitialPose()
{
    Eigen::VectorQd q_init_desired_; q_init_desired_.setZero();
    q_init_desired_ = q_init_;
    q_init_desired_(15) = + 15.0 * DEG2RAD; 
    q_init_desired_(16) = + 10.0 * DEG2RAD; 
    q_init_desired_(17) = + 80.0 * DEG2RAD; 
    q_init_desired_(18) = - 70.0 * DEG2RAD; 
    q_init_desired_(19) = - 45.0 * DEG2RAD; 
    q_init_desired_(21) =   0.0 * DEG2RAD; 

    q_init_desired_(25) = - 15.0 * DEG2RAD; 
    q_init_desired_(26) = - 10.0 * DEG2RAD;            
    q_init_desired_(27) = - 80.0 * DEG2RAD;  
    q_init_desired_(28) = + 70.0 * DEG2RAD; 
    q_init_desired_(29) = + 45.0 * DEG2RAD;       
    q_init_desired_(31) = - 0.0 * DEG2RAD; 

    q_ref_ = DyrosMath::cubicVector<MODEL_DOF>(initial_tick_, 0, 2.0 * hz_, q_init_, q_init_desired_, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero()); 

    initial_tick_++;
}

void CustomController::walkingParameterSetting()
{
    // BIPED WALKING
    ros::param::get("/tocabi_controller/target_x_", target_x_);
    ros::param::get("/tocabi_controller/target_y_", target_y_);
    ros::param::get("/tocabi_controller/target_z_", target_z_);
    ros::param::get("/tocabi_controller/step_length_x_", step_length_x_);
    ros::param::get("/tocabi_controller/step_length_y_", step_length_y_);
    ros::param::get("/tocabi_controller/com_height_", com_height_);
    ros::param::get("/tocabi_controller/is_right_foot_swing_", is_right_foot_swing_);

    ros::param::get("/tocabi_controller/t_rest_init_", t_rest_init_); 
    ros::param::get("/tocabi_controller/t_rest_last_", t_rest_last_);
    ros::param::get("/tocabi_controller/t_double1_", t_double1_);
    ros::param::get("/tocabi_controller/t_double2_", t_double2_);
    ros::param::get("/tocabi_controller/t_total_", t_total_);
    ros::param::get("/tocabi_controller/t_temp_", t_temp_);

    ros::param::get("/tocabi_controller/foot_height_", foot_height_);

    // ZMP CTRL
    ros::param::get("/tocabi_controller/kp_cp", kp_cp);

    // ZMP OFFSET
    ros::param::get("/tocabi_controller/zmp_offset", zmp_offset_);

    t_rest_init_ = t_rest_init_ * hz_;
    t_rest_last_ = t_rest_last_ * hz_;
    t_double1_ = t_double1_ * hz_;
    t_double2_ = t_double2_ * hz_;
    t_total_ = t_total_ * hz_;
    t_temp_ = t_temp_ * hz_;

    t_dsp1_ = t_rest_init_ + t_double1_;
    t_dsp2_ = t_rest_last_ + t_double2_;
    t_ssp_ = t_total_ - (t_dsp1_ + t_dsp2_);

    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;
    t_start_real_ = t_start_ + t_rest_init_;

    current_step_num_ = 0;
}

void CustomController::updateInitialState()
{
    if (walking_tick == 0)
    {
        calculateFootStepTotal();

        pelv_rpy_current_.setZero();
        pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Pelvis].xpos);

        pelv_float_init_.translation()(0) = 0;
        pelv_float_init_.translation()(1) = 0;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        lfoot_float_init_.translation()(0) = 0;
        lfoot_float_init_.translation()(1) = 0.1225;

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        rfoot_float_init_.translation()(0) = 0;
        rfoot_float_init_.translation()(1) = -0.1225;

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        com_float_init_(0) = 0;
        com_float_init_(1) = 0;

        if (aa == 0)
        {
            lfoot_float_init_.translation()(1) = 0.1025;
            rfoot_float_init_.translation()(1) = -0.1025;
            aa = 1;
        }

        Eigen::Isometry3d ref_frame;

        if (foot_step_(0, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(0, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame), rfoot_float_init_);
        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame), com_float_init_);

        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();
        swingfoot_float_init_.setZero();

        if (foot_step_(0, 6) == 1) //left suppport foot
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }
        else
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }

        total_step_num_ = foot_step_.col(1).size();
    }
    else if (current_step_num_ != 0 && is_support_foot_change == true) // step change
    {
        pelv_rpy_current_.setZero();
        pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));

        rfoot_rpy_current_.setZero();
        lfoot_rpy_current_.setZero();
        rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
        lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

        rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
        lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
        rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
        lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Pelvis].xpos);

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        Eigen::Isometry3d ref_frame_yaw_only;
        ref_frame_yaw_only.translation() = ref_frame.translation();
        Eigen::Vector3d ref_frame_rpy;
        ref_frame_rpy = DyrosMath::rot2Euler(ref_frame.linear());
        ref_frame_yaw_only.linear() = DyrosMath::rotateWithZ(ref_frame_rpy(2));

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame_yaw_only) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        is_support_foot_change = false;
    }
}

void CustomController::getRobotState()
{
    pelv_rpy_current_.setZero();
    pelv_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

    R_angle = pelv_rpy_current_(0);
    P_angle = pelv_rpy_current_(1);
    pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));

    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

    pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Pelvis].rotm;

    pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Pelvis].xpos);

    lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Left_Foot].rotm;
    // lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * rd_.link_[Right_Foot].rotm;
    // rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].v);

    cp_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_), rd_.link_[COM_id].xpos + rd_.link_[COM_id].v / wn);

    if (walking_tick == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * com_float_current_dot;

    if (walking_tick == 0)
    {
        com_float_current_LPF = com_float_current_;
    }

    com_float_current_LPF = 1 / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_;

    double support_foot_flag = foot_step_(current_step_num_, 6);
    if (support_foot_flag == 0)
    {
        supportfoot_float_current_ = rfoot_float_current_;
    }
    else if (support_foot_flag == 1)
    {
        supportfoot_float_current_ = lfoot_float_current_;
    }

    ///////////dg edit
    Eigen::Isometry3d supportfoot_float_current_yaw_only;
    supportfoot_float_current_yaw_only.translation() = supportfoot_float_current_.translation();
    Eigen::Vector3d support_foot_current_rpy;
    support_foot_current_rpy = DyrosMath::rot2Euler(supportfoot_float_current_.linear());
    supportfoot_float_current_yaw_only.linear() = DyrosMath::rotateWithZ(support_foot_current_rpy(2));

    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * pelv_float_current_;
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * lfoot_float_current_;
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * rfoot_float_current_;

    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    // cp_measured_(0) = com_support_current_(0) + com_float_current_dot_LPF(0) / wn;
    // cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;
    cp_measured_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), cp_float_current_).head(2);
 
    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    if (walking_tick == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;
 
    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP X
    zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y
    wn = sqrt(GRAVITY / com_height_);

    if (walking_tick == 0)
    {
        zmp_measured_LPF_.setZero();
    } 
    zmp_measured_LPF_ = (2 * M_PI * 2.0 * del_t) / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 2.0 * del_t) * zmp_measured_LPF_;
}

void CustomController::calculateFootStepTotal()
{
    double initial_rot = 0.0;
    double final_rot = 0.0;
    double initial_drot = 0.0;
    double final_drot = 0.0;

    initial_rot = atan2(target_y_, target_x_);

    if (initial_rot > 0.0)
        initial_drot = 20 * DEG2RAD;
    else
        initial_drot = -20 * DEG2RAD;

    unsigned int initial_total_step_number = initial_rot / initial_drot;
    double initial_residual_angle = initial_rot - initial_total_step_number * initial_drot;

    final_rot = target_theta_ - initial_rot;
    if (final_rot > 0.0)
        final_drot = 20 * DEG2RAD;
    else
        final_drot = -20 * DEG2RAD;

    unsigned int final_total_step_number = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_number * final_drot;
    double length_to_target = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    double dlength = step_length_x_;
    unsigned int middle_total_step_number = length_to_target / dlength;
    double middle_residual_length = length_to_target - middle_total_step_number * dlength;

    double step_width_init;
    double step_width;

    step_width_init = 0.01;
    step_width = 0.02;

    if (length_to_target == 0.0)
    {
        middle_total_step_number = 20; //total foot step number
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = 2 + initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;
    
    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
    {
        if (initial_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(initial_residual_angle) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
            number_of_foot_step = number_of_foot_step + del_size;
    }

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();
    
    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    int temp0;
    temp0 = -is_right;

    double initial_dir = 0.0;

    if (aa == 0)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            if (i == 0)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width_init * (i + 1));
            }
            else if (i == 1)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width_init * (i + 1));
            }

            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }
    else if (aa == 1)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
    {
        for (int i = 0; i < initial_total_step_number; i++)
        {
            temp *= -1;
            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((i + 1) * initial_drot);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((i + 1) * initial_drot);
            foot_step_(index, 5) = (i + 1) * initial_drot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step_(index, 0) = temp * (0.1025 + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (0.1025 + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001) // 직진, 제자리 보행
    {

        for (int i = 0; i < middle_total_step_number; i++)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;

                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == -is_right)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025 + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_number; i++)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin((i + 1) * final_drot + initial_rot);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos((i + 1) * final_drot + initial_rot);
            foot_step_(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (0.1025 + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (0.1025 + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }
    
    //
    bool Forward = false, Side = false;

    if(Forward)
    {
        number_of_foot_step = 11;
        foot_step_.resize(number_of_foot_step, 7);
        foot_step_.setZero();
        foot_step_support_frame_.resize(number_of_foot_step, 7);
        foot_step_support_frame_.setZero();

        // Forward
        foot_step_(0, 0) = 0.0; foot_step_(0, 1) = -0.1125; foot_step_(0, 6) = 1.0; 
        foot_step_(1, 0) = 0.0; foot_step_(1, 1) =  0.1225; foot_step_(1, 6) = 0.0;
        foot_step_(2, 0) = 0.05; foot_step_(2, 1) = -0.1225; foot_step_(2, 6) = 1.0; 
        foot_step_(3, 0) = 0.15; foot_step_(3, 1) =  0.1225; foot_step_(3, 6) = 0.0;
        foot_step_(4, 0) = 0.30; foot_step_(4, 1) = -0.1225; foot_step_(4, 6) = 1.0; 
        foot_step_(5, 0) = 0.15; foot_step_(5, 1) =  0.1225; foot_step_(5, 6) = 0.0;
        foot_step_(6, 0) = 0.30; foot_step_(6, 1) = -0.1225; foot_step_(6, 6) = 1.0; 
        foot_step_(7, 0) = 0.15; foot_step_(7, 1) =  0.1225; foot_step_(7, 6) = 0.0;
        foot_step_(8, 0) = 0.05; foot_step_(8, 1) = -0.1225; foot_step_(8, 6) = 1.0;
        foot_step_(9, 0) = 0.0; foot_step_(9, 1) =  0.1225; foot_step_(9, 6) = 0.0; 
        foot_step_(10, 0) = 0.0; foot_step_(10, 1) = -0.1225; foot_step_(10, 6) = 1.0; 
    }
    if(Side)
    {
        number_of_foot_step = 11;
        foot_step_.resize(number_of_foot_step, 7);
        foot_step_.setZero();
        foot_step_support_frame_.resize(number_of_foot_step, 7);
        foot_step_support_frame_.setZero();

        foot_step_(0, 0) = 0.0; foot_step_(0, 1) = -0.1125; foot_step_(0, 6) = 1.0; 
        foot_step_(1, 0) = 0.0; foot_step_(1, 1) =  0.1225; foot_step_(1, 6) = 0.0;
        foot_step_(2, 0) = 0.0; foot_step_(2, 1) = -0.1225; foot_step_(2, 6) = 1.0;
        foot_step_(3, 0) = 0.0; foot_step_(3, 1) =  0.1225 + 0.05*2; foot_step_(3, 6) = 0.0;
        foot_step_(4, 0) = 0.0; foot_step_(4, 1) = -0.1225 - 0.05*2; foot_step_(4, 6) = 1.0;
        foot_step_(5, 0) = 0.0; foot_step_(5, 1) =  0.1225 + 0.05*2*0; foot_step_(5, 6) = 0.0;
        foot_step_(6, 0) = 0.0; foot_step_(6, 1) = -0.1225 - 0.05*2*0; foot_step_(6, 6) = 1.0; 
        foot_step_(7, 0) = 0.0; foot_step_(7, 1) =  0.1225 + 0.05*2; foot_step_(7, 6) = 0.0; 
        foot_step_(8, 0) = 0.0; foot_step_(8, 1) = -0.1225 - 0.05*2; foot_step_(8, 6) = 1.0;
        foot_step_(9, 0) = 0.0; foot_step_(9, 1) =  0.1225; foot_step_(9, 6) = 0.0;
        foot_step_(10, 0) = 0.0; foot_step_(10, 1) = -0.1225; foot_step_(10, 6) = 1.0;
    }
       
    
}

void CustomController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;

    if (current_step_num_ == 0)
    {
        if (foot_step_(0, 6) == 0)
        {
            reference.translation() = rfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
        else
        {
            reference.translation() = lfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_ - 1, 5));
        for (int i = 0; i < 3; i++)
        {
            reference.translation()(i) = foot_step_(current_step_num_ - 1, i);
        }
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    for (int i = 0; i < total_step_num_; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp_global_position(j) = foot_step_(i, j);
        }

        temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

        for (int j = 0; j < 3; j++)
        {
            foot_step_support_frame_(i, j) = temp_local_position(j);
        }

        foot_step_support_frame_(i, 3) = foot_step_(i, 3);
        foot_step_support_frame_(i, 4) = foot_step_(i, 4);
        if (current_step_num_ == 0)
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - supportfoot_float_init_(5);
        }
        else
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - foot_step_(current_step_num_ - 1, 5);
        }
    }

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if (current_step_num_ == 0)
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if (current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);
}

void CustomController::gravityCalculate()
{
    double contact_gain = 0.0;
    double eta = 0.9;
    VectorQd grav_;

    if (walking_tick < t_dsp1_)
    {
        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
        Gravity_SSP_.setZero();
        contact_gain = 1.0;
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }
    else if (walking_tick >= t_dsp1_ && walking_tick < t_start_ + t_dsp1_) // 0.03 s
    {
        contact_gain = DyrosMath::cubic(walking_tick, t_start_ + 0.5*t_dsp1_, t_start_ + t_dsp1_, 1.0, 0.0, 0.0, 0.0);

        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
        Gravity_SSP_.setZero();
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }
    else if (walking_tick >= t_start_ + t_dsp1_ && walking_tick < t_start_ + t_total_ - t_dsp2_) // SSP
    {
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            WBC::SetContact(rd_, 1, 0);
            Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 0, 1);
            Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
        }
        Gravity_DSP_.setZero();
    }
    else if (walking_tick >= t_start_ + t_total_ - t_dsp2_ && walking_tick < t_start_ + t_total_ - 0.5*t_dsp2_)
    {
        contact_gain = DyrosMath::cubic(walking_tick, t_start_ + t_total_ - t_dsp2_, t_start_ + t_total_ - 0.5*t_dsp2_, 0.0, 1.0, 0.0, 0.0);
        Gravity_SSP_.setZero();
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }
    else if (walking_tick >= t_start_ + t_total_ - 0.5*t_dsp2_ && walking_tick < t_start_ + t_total_)
    {
        contact_gain = 1.0;

        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);

        Gravity_SSP_.setZero();
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
        }
    }

    if (atb_grav_update_ == false)
    {
        atb_grav_update_ = true;
        Gravity_ = Gravity_DSP_ + Gravity_SSP_;
        atb_grav_update_ = false;
    }
}

void CustomController::getZmpTrajectory()
{
    
}

void CustomController::addZmpOffset()
{
    double lfoot_zmp_offset_, rfoot_zmp_offset_;

    lfoot_zmp_offset_ = -zmp_offset_; // simul 1.1 s
    rfoot_zmp_offset_ =  zmp_offset_;

    foot_step_support_frame_offset_ = foot_step_support_frame_;

    supportfoot_support_init_offset_ = supportfoot_support_init_;

    if (foot_step_(0, 6) == 0) //right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    }
    else
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_; 
    }       

    for (int i = 0; i < total_step_num_; i++)
    {
        if (foot_step_(i, 6) == 0) // left support foot 
        {
            foot_step_support_frame_offset_(i, 1) += lfoot_zmp_offset_;
        }
        else // right support foot
        {
            foot_step_support_frame_offset_(i, 1) += rfoot_zmp_offset_;
        }
    }
}

void CustomController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    
}

void CustomController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py)
{
    
}

void CustomController::getComTrajectory_offline()
{
    
}

void CustomController::comGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    
}

void CustomController::onestepCom(unsigned int current_step_number, Eigen::VectorXd &temp_cx, Eigen::VectorXd &temp_cy)
{
    
}

void CustomController::getComTrajectory()
{
    
}

void CustomController::preview_Parameter(double dt, int NL, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx, Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C)
{
    
}

void CustomController::previewcontroller(double dt, int NL, int tick, 
                                         Eigen::Vector3d &x_k, Eigen::Vector3d &y_k, double &UX, double &UY,
                                         const Eigen::MatrixXd &Gi, const Eigen::VectorXd &Gd, const Eigen::MatrixXd &Gx, 
                                         const Eigen::MatrixXd &A,  const Eigen::VectorXd &B,  const Eigen::MatrixXd &C)
{
    
}

void CustomController::getComTrajectory_mpc()
{
    
}

void CustomController::getCPTrajectory()
{
    
}

void CustomController::getFootTrajectory() 
{
    Eigen::Vector6d target_swing_foot; target_swing_foot.setZero();
    target_swing_foot = foot_step_support_frame_.row(current_step_num_).transpose().segment(0,6);

    Eigen::Isometry3d &support_foot_traj           = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_trajectory_support_ : rfoot_trajectory_support_;
    Eigen::Vector3d &support_foot_traj_euler       = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_trajectory_euler_support_ : rfoot_trajectory_euler_support_;
    const Eigen::Isometry3d &support_foot_init     = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_support_init_ : rfoot_support_init_;
    const Eigen::Vector3d &support_foot_euler_init = (is_lfoot_support == true && is_rfoot_support == false) ? lfoot_support_euler_init_ : rfoot_support_euler_init_;

    Eigen::Isometry3d &swing_foot_traj             = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_trajectory_support_ : lfoot_trajectory_support_;
    Eigen::Vector3d &swing_foot_traj_euler         = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_trajectory_euler_support_ : lfoot_trajectory_euler_support_;
    const Eigen::Isometry3d &swing_foot_init       = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_support_init_ : lfoot_support_init_;
    const Eigen::Vector3d &swing_foot_euler_init   = (is_lfoot_support == true && is_rfoot_support == false) ? rfoot_support_euler_init_ : lfoot_support_euler_init_;

    if (is_dsp1 == true)
    {
        support_foot_traj.translation().setZero();
        support_foot_traj_euler.setZero();

        swing_foot_traj.translation() = swing_foot_init.translation();
        swing_foot_traj.translation()(2) = 0.0;
        swing_foot_traj_euler = swing_foot_euler_init;
    }
    else if (is_ssp == true)
    {
        support_foot_traj.translation().setZero();
        support_foot_traj_euler.setZero();

        if (walking_tick < t_start_ + t_dsp1_ + t_ssp_ / 2.0)
        {
            swing_foot_traj.translation()(2) = DyrosMath::cubic(walking_tick, 
                                                                t_start_ + t_dsp1_,
                                                                t_start_ + t_dsp1_ + t_ssp_ / 2.0, 
                                                                0.0, foot_height_, 
                                                                0.0, 0.0);
        }
        else
        {
            swing_foot_traj.translation()(2) = DyrosMath::cubic(walking_tick, 
                                                                t_start_ + t_dsp1_ + t_ssp_ / 2.0,
                                                                t_start_ + t_dsp1_ + t_ssp_, 
                                                                foot_height_, target_swing_foot(2), 
                                                                0.0, 0.0);
        }

        swing_foot_traj.translation().segment(0,2) = DyrosMath::cubicVector<2>(walking_tick, 
                                                                               t_start_ + t_dsp1_,
                                                                               t_start_ + t_dsp1_ + t_ssp_, 
                                                                               swing_foot_init.translation().segment(0,2), target_swing_foot.segment(0,2),
                                                                               Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

        swing_foot_traj_euler.setZero();
        swing_foot_traj_euler(2) = DyrosMath::cubic(walking_tick, 
                                                    t_start_ + t_dsp1_, 
                                                    t_start_ + t_dsp1_ + t_ssp_, 
                                                    swing_foot_euler_init(2), target_swing_foot(5), 
                                                    0.0, 0.0);
    }
    else if (is_dsp2 == true)
    {
        support_foot_traj_euler.setZero();
        
        swing_foot_traj.translation() = target_swing_foot.segment(0,3);
        swing_foot_traj_euler = target_swing_foot.segment(3,3);
    }

    swing_foot_traj.linear() = DyrosMath::Euler2rot(swing_foot_traj_euler(0), swing_foot_traj_euler(1), swing_foot_traj_euler(2));
    support_foot_traj.linear() = DyrosMath::Euler2rot(support_foot_traj_euler(0), support_foot_traj_euler(1), support_foot_traj_euler(2));
}

void CustomController::getFootTrajectory_Kajita()
{
    Eigen::Vector6d target_swing_foot;
    for (int i = 0; i < 6; i++)
    { target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i); }

    //before swing
    if (walking_tick < t_start_ + t_dsp1_)
    {
        if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
        {
            lfoot_trajectory_support_.translation().setZero();
            lfoot_trajectory_euler_support_.setZero();
            
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.translation().setZero();

            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;            
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
    }
    //mid swing
    else if (walking_tick >= t_start_ + t_dsp1_ && walking_tick < t_start_ + t_total_ - t_dsp2_)
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_.setZero();
            
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            if (walking_tick < t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0)
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, rfoot_support_init_.translation()(2), rfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
            }
            else
            {
                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_ - t_dsp2_, rfoot_support_init_.translation()(2) + foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, rfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

            rfoot_trajectory_euler_support_(0) = 0;
            rfoot_trajectory_euler_support_(1) = 0;
            rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, rfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_euler_support_.setZero();

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            if (walking_tick < t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0)
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, lfoot_support_init_.translation()(2), lfoot_support_init_.translation()(2) + foot_height_, 0.0, 0.0);
            }
            else
            {
                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_ + (t_total_ - t_dsp1_ - t_dsp2_) / 2.0, t_start_ + t_total_ - t_dsp2_, lfoot_support_init_.translation()(2) + foot_height_, target_swing_foot(2), 0.0, 0.0);
            }

            for (int i = 0; i < 2; i++)
            {
                lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, lfoot_support_init_.translation()(i), target_swing_foot(i), 0.0, 0.0);
            }

            lfoot_trajectory_euler_support_(0) = 0;
            lfoot_trajectory_euler_support_(1) = 0;
            lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick, t_start_ + t_dsp1_, t_start_ + t_total_ - t_dsp2_, lfoot_support_euler_init_(2), target_swing_foot(5), 0.0, 0.0);
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        }
    }
    //after swing
    else
    {
        if (foot_step_(current_step_num_, 6) == 1)
        {
            lfoot_trajectory_euler_support_.setZero();
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

            for (int i = 0; i < 3; i++)
            {
                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }

            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (foot_step_(current_step_num_, 6) == 0)
        {
            rfoot_trajectory_euler_support_.setZero();
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

            for (int i = 0; i < 3; i++)
            {
                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
            }

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
        }
    }
}

void CustomController::getPelvTrajectory()
{
    double z_rot = foot_step_support_frame_(current_step_num_, 5);

    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0 * (com_desired_(0) - com_support_current_(0)); 
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.7 * (com_desired_(1) - com_support_current_(1));  
    pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.0 * (com_desired_(2) - com_support_current_(2));  

    Eigen::Vector3d pelv_traj_euler; pelv_traj_euler.setZero();

    if (is_dsp1 == true)
    {
        pelv_traj_euler(2) = pelv_support_euler_init_(2);
    }
    else if (is_ssp == true)
    {
        pelv_traj_euler(2) = DyrosMath::cubic(walking_tick, 
                                              t_start_ + t_dsp1_,
                                              t_start_ + t_dsp1_ + t_ssp_, 
                                              pelv_support_euler_init_(2), 
                                              z_rot / 2.0, 
                                              0.0, 0.0);
    }
    else
    {
        pelv_traj_euler(2) = z_rot / 2.0;
    }

    R_angle_input_dot = 2.0 * (0.0 - R_angle) ;
    P_angle_input_dot = 2.0 * (0.0 - P_angle) ;

    R_angle_input = DyrosMath::minmax_cut(R_angle_input + R_angle_input_dot * del_t, -5.0 * DEG2RAD, 5.0 * DEG2RAD);
    P_angle_input = DyrosMath::minmax_cut(P_angle_input + P_angle_input_dot * del_t, -5.0 * DEG2RAD, 5.0 * DEG2RAD);

    pelv_traj_euler(0) = R_angle_input;
    pelv_traj_euler(1) = P_angle_input;    
    
    pelv_trajectory_support_.linear() = DyrosMath::Euler2rot(pelv_traj_euler(0), pelv_traj_euler(1), pelv_traj_euler(2));
}

void CustomController::computeIkControl(const Eigen::Isometry3d &float_trunk_transform, const Eigen::Isometry3d &float_lleg_transform, const Eigen::Isometry3d &float_rleg_transform, Eigen::Vector12d &q_des)
{
    /*
    Explanations for the Eigen variable:

    float_trunk_transform : Function argument for the Homogeneous Transform from base to pelvis.
    float_lleg_transform  : Function argument for the Homogeneous Transform from base to left ankle.
    float_rleg_transform  : Function argument Homogeneous Transform from base to right ankle.

    
    Examples for the access to the rotation and translation in Homogeneous Transform

    Rotation              : float_trunk_transform.rotation().    Returns 3x3 Eigen::Matrix variable.
    Translation           : float_trunk_transform.translation(). Returns 3x1 Eigen::Vector variable.

    
    Other useful functions for Eigen variable.
    
    Transpose             : float_trunk_transform.rotation().transpose() Returns the transpose matrix of rotation matrix. 
    */

    Eigen::Vector3d R_r, R_D, L_r, L_D;

    //TODO 2
    //Calculate the joint angles to implement the pelvis and foot trajectory
    //Fill the Vector variables for D (Distance from pelvis to hip)
    L_D(0) = ;
    L_D(1) = ;
    L_D(2) = ;

    R_D(0) = ;
    R_D(1) = ;
    R_D(2) = ;

    //Calculate the Vector variables for r (Vector from ankle to hip)
    L_r = ;
    R_r = ;

    
    double R_C = 0, L_C = 0, L_upper = 0.351, L_lower = 0.351, R_alpha = 0, L_alpha = 0;
    
    //Calculate the size of Vector r (Distance from ankle to hip)
    L_C = ;
    R_C = ;
     
    double knee_acos_var_L = 0;
    double knee_acos_var_R = 0;

    //Calculate the cos(PI - q7)
    knee_acos_var_L = ;
    knee_acos_var_R = ;

    //Do not Touch
    //MinMax Cut for the calculated value for the safety of real robot.
    knee_acos_var_L = DyrosMath::minmax_cut(knee_acos_var_L, -0.99, + 0.99);
    knee_acos_var_R = DyrosMath::minmax_cut(knee_acos_var_R, -0.99, + 0.99);

    //Calculate the q7(left knee), q13(right knee)
    q_des(3) = ;
    q_des(9) = ;
    
    //Calculate the ankle alpha
    L_alpha = ;
    R_alpha = ;
    
    //Calculate the q8(left ankle pitch), q14(right ankle pitch)
    q_des(4)  = ;
    q_des(10) = ;

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat,      L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat,          L_Hip_rot_mat;

    //Calculate the rotation matrix for knee and ankle pitch, ankle roll
    //rotateWithX(a) : Returns rotation matrix with axis X and degree a.
    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY();
    L_Ankle_X_rot_mat      = DyrosMath::rotateWithX();

    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY();
    R_Ankle_X_rot_mat      = DyrosMath::rotateWithX();

    L_Hip_rot_mat.setZero();
    R_Hip_rot_mat.setZero();

    //Calculate the rotation matrix for hip
    L_Hip_rot_mat = ;
    R_Hip_rot_mat = ;


    q_des(0) = ;         // Hip yaw
    q_des(1) = ;         // Hip roll    
    q_des(2) = ;         // Hip pitch
    q_des(3) = q_des(3); // Knee pitch
    q_des(4) = q_des(4); // Ankle pitch
    q_des(5) = ;         // Ankle roll

    q_des(6)  = ;
    q_des(7)  = ;
    q_des(8)  = ;
    q_des(9)  = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = ;
}

void CustomController::circling_motion()
{
    /*
    Explanations for the Eigen variable:

    pelv_trajectory_float_  : Global variable to save the Homogeneous Transform from base to pelvis.
    rfoot_trajectory_float_ : Global variable to save the Homogeneous Transform from base to left ankle.
    lfoot_trajectory_float_ : Global variable to save the Homogeneous Transform from base to right ankle.

    
    Examples for the access to the rotation and translation in Homogeneous Transform

    Rotation              : pelv_trajectory_float_.rotation().    Returns 3x3 Eigen::Matrix variable.
    Linear                : pelv_trajectory_float_.linear().      Returbs 3x3 Eigen::Matrix variable. Works Same as rotation.
    Translation           : pelv_trajectory_float_.translation(). Returns 3x1 Eigen::Vector variable.

    
    Other useful functions for Eigen variable.
    
    Transpose             : pelv_trajectory_float_.rotation().transpose().  Returns the transpose matrix of rotation matrix. 
    SetZero               : pelv_trajectory_float_.setZero().               Set all element of homogeneous transform to zero.
                            pelv_trajectory_float_.translation().setZero(). A part of the matrix can be set to zero.
                            pelv_trajectory_float_.rotation().setZero().
    SetIdentity           : pelv_trajectory_float_.linear().setIdentity().  Set square matrix to identity matrix.

    
    Variables in walking controller
    walking_tick_         : simulation tick.
    hz_                   : control hz of walking controller. 2000hz here.
    */

    //TODO 3
    //Calculate the pelvis and foot trajectory for the circling motion
    //Set homogeneous transform from base to pelvis

    //Set homogeneous transform from base to right foot

    //Set homogeneous transform from base to left foot
}

void CustomController::contactWrenchCalculator()
{ 
    ////// DEL ZMP CALCULATION //////
    double cp_eos_calc3 = 1/(1 - exp(wn*0.05));
    del_zmp(0) = cp_eos_calc3*cp_desired_(0) + (1 - cp_eos_calc3)*cp_measured_(0) - ZMP_X_REF_;
    del_zmp(0) = DyrosMath::minmax_cut(del_zmp(0), -0.09, 0.13);
    del_zmp(1) = cp_eos_calc3*cp_desired_(1) + (1 - cp_eos_calc3)*cp_measured_(1) - ZMP_Y_REF_;
    del_zmp(1) = DyrosMath::minmax_cut(del_zmp(1), ZMP_Y_REF_ - 0.07 + 0.02*pow(-1, current_step_num_), ZMP_Y_REF_ + 0.07 + 0.02*pow(-1, current_step_num_));

    ////// CONTACT WRENCH CALCULATION //////
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y, Tau_R_y, Tau_L_y = 0;
    double Tau_all_x, Tau_R_x, Tau_L_x = 0; 
     
    alpha = (ZMP_Y_REF_ + del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
    alpha = DyrosMath::minmax_cut(alpha, 0.0, 1.0);
    alpha_lpf_ = DyrosMath::lpf(alpha, alpha_lpf_, 2000.0, 50.0);
    alpha_lpf_ = DyrosMath::minmax_cut(alpha_lpf_, 0.0, 1.0);

    //////////// FORCE ////////////
    F_R = -(1 - alpha_lpf_) * rd_.link_[COM_id].mass * GRAVITY;
    F_L =     - alpha_lpf_  * rd_.link_[COM_id].mass * GRAVITY;

    //////////// TORQUE ////////////
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF_ + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF_ + del_zmp(1))) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF_ + del_zmp(0))) * F_L);
 
    Tau_R_x = (1 - alpha_lpf_) * Tau_all_x;
    Tau_R_y =-(1 - alpha_lpf_) * Tau_all_y;
    Tau_L_x = alpha_lpf_ * Tau_all_x;
    Tau_L_y =-alpha_lpf_ * Tau_all_y;

    lfoot_contact_wrench << 0.0, 0.0, F_L, Tau_L_x, Tau_L_y, 0.0;
    rfoot_contact_wrench << 0.0, 0.0, F_R, Tau_R_x, Tau_R_y, 0.0;
  
    contact_wrench_torque.setZero();
    contact_wrench_torque = rd_.link_[Left_Foot].Jac().rightCols(MODEL_DOF).transpose()  * lfoot_contact_wrench + rd_.link_[Right_Foot].Jac().rightCols(MODEL_DOF).transpose() * rfoot_contact_wrench;
}

void CustomController::contactWrench_Kajita()
{
    double alpha = 0;
    double F_R = 0, F_L = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
    double zmp_offset = 0.0;
    double alpha_new = 0;

    zmp_offset = zmp_offset_; // econom2

    if (walking_tick > t_temp_)
    {
        if (walking_tick < t_start_ + t_dsp1_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset * (walking_tick - (t_start_ + t_dsp1_) + t_dsp1_) / t_dsp1_;
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset * (walking_tick - (t_start_ + t_dsp1_) + t_dsp1_) / t_dsp1_;
            }
        }
        else if (walking_tick >= t_start_ + t_dsp1_ && walking_tick < t_start_ + t_total_ - t_dsp2_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset;
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset;
            }
        }
        else if (walking_tick >= t_start_ + t_total_ - t_dsp2_ && walking_tick < t_start_ + t_total_)
        {
            if (foot_step_(current_step_num_, 6) == 1)
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ + zmp_offset - zmp_offset * (walking_tick - (t_start_ + t_total_ - t_dsp2_)) / t_dsp2_;
            }
            else
            {
                ZMP_Y_REF_alpha_ = ZMP_Y_REF_ - zmp_offset + zmp_offset * (walking_tick - (t_start_ + t_total_ - t_dsp2_)) / t_dsp2_;
            }
        }
        else
        {
            ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
        }
    }
    else
    {
        ZMP_Y_REF_alpha_ = ZMP_Y_REF_;
    }

    double ZMP_X_DES_CALC = 0.0;
    double ZMP_Y_DES_CALC = 0.0;
    double lambda_desired = 0.0;

    double cp_eos_calc3 = 1/(1 - exp(wn*0.05));
    if(walking_tick < 0.75*t_temp_ || walking_tick > t_temp_ + total_step_num_*t_total_)
    {
        cp_eos_calc3 = 1/(1 - exp(wn*0.5));
    }

    del_zmp(0) = cp_eos_calc3*cp_desired_(0) + (1 - cp_eos_calc3)*cp_measured_(0) - ZMP_X_REF_;
    del_zmp(1) = cp_eos_calc3*cp_desired_(1) + (1 - cp_eos_calc3)*cp_measured_(1) - ZMP_Y_REF_;

    //del_zmp = kp_cp*(cp_measured_ - cp_desired_);

    ZMP_X_DES_CALC = ZMP_X_REF_ + del_zmp(0);
    ZMP_Y_DES_CALC = ZMP_Y_REF_alpha_ + del_zmp(1);

    ZMP_X_DES_CALC = DyrosMath::minmax_cut(ZMP_X_DES_CALC, ZMP_X_REF_ - 0.09, ZMP_X_REF_ + 0.13);
    ZMP_Y_DES_CALC = DyrosMath::minmax_cut(ZMP_Y_DES_CALC, ZMP_Y_REF_alpha_ - 0.07, ZMP_Y_REF_alpha_ + 0.07);

    double calc_z_max = 0.08;
    double alpha_zmp_calc = ZMP_Y_DES_CALC;

    alpha = (alpha_zmp_calc - (rfoot_support_current_.translation()(1) + calc_z_max)) / ((lfoot_support_current_.translation()(1) - calc_z_max) - (rfoot_support_current_.translation()(1) + calc_z_max));
    alpha = DyrosMath::minmax_cut(alpha, 0.0, 1.0);
    alpha_lpf_ = DyrosMath::lpf(alpha, alpha_lpf_, 2000.0, 50.0);
    alpha_lpf_ = DyrosMath::minmax_cut(alpha_lpf_, 0.0, 1.0);

    F_R = -(1 - alpha) * (rd_.link_[COM_id].mass * GRAVITY);
    F_L =     - alpha  * (rd_.link_[COM_id].mass * GRAVITY);

    if (walking_tick == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }

    //////////// Force

    F_F_input_dot = 0.0001 * ((l_ft_(2) - r_ft_(2)) - (F_L - F_R)) - 3.0 * F_F_input;
    F_F_input = F_F_input + F_F_input_dot * del_t;
    F_F_input = DyrosMath::minmax_cut(F_F_input, -0.02, 0.02);

    //////////// Torque
    Tau_all_x = -((rfoot_support_current_.translation()(1) - ZMP_Y_DES_CALC) * F_R + (lfoot_support_current_.translation()(1) - ZMP_Y_DES_CALC) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - ZMP_X_DES_CALC) * F_R + (lfoot_support_current_.translation()(0) - ZMP_X_DES_CALC) * F_L);
    Tau_all_x = DyrosMath::minmax_cut(Tau_all_x, -100.0, 100.0);
    Tau_all_y = DyrosMath::minmax_cut(Tau_all_y, -100.0, 100.0);
    

    Tau_R_x = (1 - alpha) * Tau_all_x;
    Tau_L_x = (alpha)*Tau_all_x;

    Tau_L_y = -alpha * Tau_all_y;
    Tau_R_y = -(1 - alpha) * Tau_all_y;

    // Roll 방향 (-0.02/-30 0.9초)
    //F_T_L_x_input_dot = -0.1 * (Tau_L_x - l_ft_LPF(3)) - 50.0 * F_T_L_x_input;
    F_T_L_x_input_dot = -0.04 * (Tau_L_x - l_ft_LPF(3)) - 40.0 * F_T_L_x_input;
    F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;

    //F_T_R_x_input_dot = -0.1 * (Tau_R_x - r_ft_LPF(3)) - 50.0 * F_T_R_x_input;
    F_T_R_x_input_dot = -0.04 * (Tau_R_x - r_ft_LPF(3)) - 40.0 * F_T_R_x_input;
    F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
    
    // Pitch 방향  (0.005/-30 0.9초)
    //F_T_L_y_input_dot = 0.1 * (Tau_L_y - l_ft_LPF(4)) - 50.0 * F_T_L_y_input;
    F_T_L_y_input_dot = 0.04 * (Tau_L_y - l_ft_LPF(4)) - 40.0 * F_T_L_y_input;
    F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;

    //F_T_R_y_input_dot = 0.1 * (Tau_R_y - r_ft_LPF(4)) - 50.0 * F_T_R_y_input;
    F_T_R_y_input_dot = 0.04 * (Tau_R_y - r_ft_LPF(4)) - 40.0 * F_T_R_y_input;
    F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;

    F_T_L_x_input = DyrosMath::minmax_cut(F_T_L_x_input, -0.2, 0.2);
    F_T_R_x_input = DyrosMath::minmax_cut(F_T_R_x_input, -0.2, 0.2);

    F_T_L_y_input = DyrosMath::minmax_cut(F_T_L_y_input, -0.2, 0.2);
    F_T_R_y_input = DyrosMath::minmax_cut(F_T_R_y_input, -0.2, 0.2);
}

void CustomController::supportToFloatPattern()
{
    pelv_trajectory_float_  = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;

    if(walking_tick >= t_start_ + t_dsp1_ && walking_tick <= t_start_ + t_total_ - t_dsp2_)
    {
        if(foot_step_(current_step_num_,6) == 1) // left foot support
        {
            lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
            rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
        }
        else
        {
            lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
            rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
        }
    }
    else
    {
        rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
        lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
    }
}

void CustomController::updateNextStepTime()
{       
    /*
    if (walking_tick >= t_last_)
    {   
        if (current_step_num_ != total_step_num_ - 1)
        {   
            t_start_ = t_last_ + 1;
            t_start_real_ = t_start_ + t_rest_init_;
            t_last_ = t_start_ + t_total_ - 1;
            current_step_num_++;            
        }  

        is_support_foot_change = true;
    }

    if ((current_step_num_ == total_step_num_ - 1 && walking_tick >= t_total_ + t_last_) != true)
    {
        walking_tick++;
    }*/
   walking_tick++;
}

void CustomController::walkingStateMachine()
{
    if (foot_step_(current_step_num_, 6) == 1) 
    {
        is_lfoot_support = true;
        is_rfoot_support = false;
    }
    else if (foot_step_(current_step_num_, 6) == 0) 
    {
        is_lfoot_support = false;
        is_rfoot_support = true;
    }

    if (walking_tick < t_start_ + t_rest_init_ + t_double1_)
    {
        is_dsp1 = true;
        is_ssp  = false;
        is_dsp2 = false;
    }
    else if (walking_tick >= t_start_ + t_rest_init_ + t_double1_ && walking_tick < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        is_dsp1 = false;
        is_ssp  = true;
        is_dsp2 = false;
    }
    else
    {
        is_dsp1 = false;
        is_ssp  = false;
        is_dsp2 = true;
    } 
}

void CustomController::pubDataSlowToThread3()
{
    if(atb_mpc_update_ == false)
    {
        atb_mpc_update_ = true;

        walking_tick_container = walking_tick;
        zmp_start_time_container = zmp_start_time_; 
        current_step_container = current_step_num_;
        ref_zmp_container = ref_zmp_;

        x_mpc_container = x_mpc_;
        y_mpc_container = y_mpc_;

        atb_mpc_update_ = false;
    }
}

void CustomController::subDataSlowToThread3()
{
    if(atb_mpc_update_ == false)
    {
        atb_mpc_update_ = true;

        walking_tick_thread3 = walking_tick_container;
        zmp_start_time_thread3 = zmp_start_time_container;
        current_step_thread3 = current_step_container;

        ref_zmp_thread3 = ref_zmp_container;

        x_mpc_thread3 = x_mpc_container;
        y_mpc_thread3 = y_mpc_container;

        atb_mpc_update_ = false;
    }
}

void CustomController::pubDataThread3ToSlow()
{
    if (atb_mpc_x_update_ == false)
    {
        atb_mpc_x_update_ = true;
        
        x_mpc_container2 = x_mpc_thread3;

        current_step_checker = current_step_thread3;

        atb_mpc_x_update_ = false;
    }

    is_mpc_x_update = true;

    if (atb_mpc_y_update_ == false)
    {
        atb_mpc_y_update_ = true;
        
        y_mpc_container2 = y_mpc_thread3;

        current_step_checker = current_step_thread3;

        atb_mpc_y_update_ = false;
    }

    is_mpc_y_update = true;
}

void CustomController::subDataThread3ToSlow()
{
    if (is_mpc_x_update == true)
    {
        if (atb_mpc_x_update_ == false)
        {
            atb_mpc_x_update_ = true;
            
            x_mpc_prev = x_mpc_;
            
            if (current_step_checker == current_step_num_)
            {
                x_mpc_ = x_mpc_container2;

                mpc_interpol_cnt_x = 1;
            }
            else
            {
                std::cout << "MPC output X is ignored: step number mismatch (MPC step = " 
                        << current_step_checker << ", real-time step = " 
                        << current_step_num_ << ")." << std::endl;
            }

            atb_mpc_x_update_ = false;
        }
        
        is_mpc_x_update = false;
    }

    if (is_mpc_y_update == true)
    {
        if (atb_mpc_y_update_ == false)
        {
            atb_mpc_y_update_ = true;

            y_mpc_prev = y_mpc_;

            if (current_step_checker == current_step_num_)
            {
                y_mpc_ = y_mpc_container2;

                mpc_interpol_cnt_y = 1;
            }
            else
            {
                std::cout << "MPC output Y is ignored: step number mismatch (MPC step = " 
                        << current_step_checker << ", real-time step = " 
                        << current_step_num_ << ")." << std::endl;
            }

            atb_mpc_y_update_ = false;
        }

        is_mpc_y_update = false;
    }
}

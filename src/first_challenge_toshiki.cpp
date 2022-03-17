#include "first_challenge_toshiki/first_challenge_toshiki.h"

FirstChallenge::FirstChallenge():private_nh_("~"), nh_("")
{
    private_nh_.param("hz", hz_, {80});
    private_nh_.param("phase", phase_, {0});
    private_nh_.param("turn_count", turn_count_, {0});
    private_nh_.param("distance_target_from_start_position", distance_target_from_start_position_, {0.3}); // 単位：m
    private_nh_.param("distance_target_to_wall", distance_target_to_wall_, {0.50}); // 単位：m
    private_nh_.param("distance", distance_, {0.0}); // 単位：m
    private_nh_.param("yaw_target", yaw_target_, {0.0}); // 単位：rad
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

// odmetryのコールバック関数
void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

// laserのコールバック関数
void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

// 直進
int FirstChallenge::straight()
{
    cmd_vel_.mode = 11; // 任意の動きを実行するモード
    cmd_vel_.cntl.linear.x = 0.1;
    cmd_vel_.cntl.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_); // Roombaを動かす指令
    return 1;
}

// 旋回
int FirstChallenge::turn()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.7;
    pub_cmd_vel_.publish(cmd_vel_);
    return 2;
}

// 停止
int FirstChallenge::stop()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
    return -1;
}

int FirstChallenge::move()
{
    int move_command = 0;
    switch(phase_)
    {
        case 0 : // １ｍ直進
            cal_distance();
            if(distance_ < distance_target_from_start_position_)
            {
                move_command = straight();
            }
            else
            {
                move_command = stop();
                phase_++;
            }
            break;

        case 1 : // １周回る
            if(can_turn())
            {
                move_command = turn();
            }
            else
            {
                move_command = stop();
                phase_++;
            }
            break;

        case 2: // 壁５０cm 手前で止まる
            if(can_find_wall())
            {
                move_command = stop();
                phase_++;
            }
            else
            {
                move_command = straight();
            }
            break;

        default: break;
    }
    return move_command;
}

// レーザ値の最小値を返す
float FirstChallenge::get_scan_min()
{
    float range_min   = 1e6;
    int   start_index = laser_.ranges.size()/2 - 10; // laser_.ranges.size()/12;
    int   end_index   = laser_.ranges.size()/2 + 10; // laser_.ranges.size()/12;

    // ROS_INFO("ranges.size:%ld",laser_.ranges.size());
    // ROS_INFO_STREAM(laser_.ranges[laser_.ranges.size()/2]);

    // レーザ値の最小値の更新
    for(int i=start_index; i<end_index; i++)
        if(laser_.ranges[i] < range_min)
            range_min = laser_.ranges[i];

    return range_min;
}

// オイラー角から計算したyaw角を返す
float FirstChallenge::get_yaw()
{
    tf::Quaternion quat(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return float(yaw);
}

// 目標旋回角に達するまでTrueを返す
bool FirstChallenge::can_turn()
{
    float current_yaw = get_yaw();

    if(yaw_target_+M_PI/2.0<current_yaw && turn_count_==0)
        turn_count_++;
    if((yaw_target_<=current_yaw && current_yaw<yaw_target_+M_PI/2.0) && 0<turn_count_)
        return false;

    return true;
}

// 壁との距離が目標値以内であればTrueを返す
bool FirstChallenge::can_find_wall()
{
    if(get_scan_min() <= distance_target_to_wall_)
        return true;
    else
        return false;
}

// 初期位置と現在位置の直線距離の算出
void FirstChallenge::cal_distance()
{
    float dx = odometry_.pose.pose.position.x;
    float dy = odometry_.pose.pose.position.y;
    distance_ = sqrt(powf(dx, 2.0)+powf(dy, 2.0));
}

// 実行した動作と各種状態の表示
void FirstChallenge::print_info(int move_command)
{
    switch(move_command)
    {
        case -1 : ROS_INFO("Move : Stop");     break;
        case  1 : ROS_INFO("Move : Straight"); break;
        case  2 : ROS_INFO("Move : Turn");     break;
        default: break;
    }

    ROS_INFO("phase      : %d", phase_);

    switch(phase_)
    {
        case 0:
            ROS_INFO("== odometry ==");
            ROS_INFO("x = %f", odometry_.pose.pose.position.x);
            ROS_INFO("y = %f", odometry_.pose.pose.position.y);
            ROS_INFO("z = %f\n", odometry_.pose.pose.position.z);
            ROS_INFO("== distance ==");
            ROS_INFO("distance                            = %f", distance_);
            ROS_INFO("distance_target_from_start_position = %f", distance_target_from_start_position_);
            ROS_INFO("distance_diff                       = %f", distance_target_from_start_position_ - distance_);
            break;

        case 1:
            ROS_INFO("turn_count : %d\n", turn_count_);
            ROS_INFO("== yawing angle ==");
            ROS_INFO("yaw        = %f", get_yaw());
            ROS_INFO("yaw_target = %f", yaw_target_);
            ROS_INFO("yaw_diff   = %f", yaw_target_ - get_yaw());
            break;

        case 2:
            ROS_INFO("== laser ==");
            ROS_INFO("scan_min                = %f", get_scan_min());
            ROS_INFO("distance_target_to_wall = %f", distance_target_to_wall_);
            ROS_INFO("distance_diff           = %f", get_scan_min() - distance_target_to_wall_);
            break;

        default:
            break;

    }
    ROS_INFO("---------------------------------------------------\n\n\n");
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_); // 周波数の設定
    int move_command = 0;     // 実行動作を記録

    while(ros::ok())
    {
        move_command = move();
        print_info(move_command);

        ros::spinOnce();
        loop_rate.sleep(); // 周期が終わるまで待つ

        if(3 <= phase_) break;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "first_challenge_toshiki"); // ノードの初期化
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();

    return 0;
}

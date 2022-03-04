#ifndef FIRST_CHALLENGE_TOSiHIKI_H
#define FIRST_CHALLENGE_TOSHIKI_H

#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

class FirstChallenge
{
    public:
        FirstChallenge(); // デフォルトコンストラクタ
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);  // odmetryのコールバック関数
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr&); // laserのコールバック関数
        void print_info(int move_command);                            // 実行した動作と各種状態の表示

        int   straight();      // 直進
        int   turn();          // 旋回
        int   stop();          // 停止
        int   move();
        float get_scan_min();  // レーザ値の最小値を返す
        float get_yaw();       // オイラー角から計算したyaw角を返す
        bool  can_turn();      // 目標旋回角に達するまでTrueを返す
        bool  can_find_wall(); // 壁との距離が目標値以内であればTrueを返す
        void  cal_distance();  // 初期位置と現在位置の直線距離の算出


        int   hz_;                                  // ループ周波数
        int   phase_;                               // 動作の段階
        int   turn_count_;                          // 回転の回数
        float distance_target_from_start_position_; // 初期位置からの目標移動距離
        float distance_target_to_wall_;             // 壁までの目標距離
        float distance_;                            // 初期位置と現在位置の直線距離を算出
        float yaw_target_;                          // 目標yaw角


        nav_msgs::Odometry odometry_;                // 現在位置
        sensor_msgs::LaserScan laser_;               // レーザ値
        roomba_500driver_meiji::RoombaCtrl cmd_vel_; // 速度指令

        ros::NodeHandle nh_;         // ノードハンドル
        ros::NodeHandle private_nh_; // プライベートノードハンドル
        ros::Subscriber sub_odom_;   // サブスクライバ（odometry）
        ros::Subscriber sub_laser_;  // サブスクライバ（laser）
        ros::Publisher pub_cmd_vel_; // パブリッシャ（速度指令）
};
#endif

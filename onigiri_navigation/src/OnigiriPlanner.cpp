/**
 * \file OnigiriPlanner.cpp
 * \brief  
 * \date 2018.7.27   ver.0.1 OnigiriPlanner新規作成
 *  date 
 *
 * \note
 *
 */
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>

#include <std_msgs/String.h>  

struct Point{
    float pos_x;
    float pos_y;
    float ori_z;
    float front_time;
    float wait_time;
    bool  back_run;
};


/** ex) Onigiri World **/
Point start_ptn = { 0.100, -0.00, -0.02, 0, 0, false};
Point docking_ptn = { 0.00, 0.00, 0.00, 0, 0, false};
Point goal_ptn[] = {
//{0.850, 0.438,  0.250, 0.5, 2.0},	// 手前左の見える位置
//{0.898,-0.481, -0.222, 0.5, 2.0},	// 手前右の見える位置
//{1.418, 0.032, -0.005, 0.5, 2.0},	// 手前真ん中の位置
{2.297, 0.777, -0.011, 0.5, 2.0, false},	// 0:左側の正面
//{2.297, 0.777, -0.324, 2.5,15.0},	// 左側から敵を監視
{2.297, 0.777, -0.702, 0.5, 2.0, false},	// 1:左側の中央向き
{2.297, 0.777, -0.999, 0.5, 2.0, false},	// 2:左側の後ろ (敵がいたらスキップ）
{2.297, 0.777, -0.324, 0.0, 0.0, false},	// 3:左側の正面より右向き
//{2.297, 0.777,  0.324, 0.0, 0.0, false},	// 左側の正面より左向き
//{1.518, 0.032, -0.324, 0.0, 0.0, true},	// 手前真ん中の位置
{3.250,  0.012,  0.000, 0.0, 0.0, false},	// 4:敵陣センター
{3.250,  0.012, -0.999, 1.0, 2.0, false},	// 5:敵陣センター(敵がいなかった場合)
{3.250,  0.012,  0.30,  0.0, 0.0, false},	// 6:敵陣センター(敵がいた場合) 
{2.374, -0.889,  0.987, 0.5, 2.0, false},	// 7:右側の後ろ
{2.374, -0.889,  0.712, 0.5, 2.0, false},	// 8:右側の中央向き
{2.374, -0.889,  0.049, 0.5, 2.0, false},	// 9:右側の正面　(敵がいたらスキップ)
{2.374, -0.889,  0.712, 0.0, 0.0, false},	// 10:右側の中央向き
{2.28, -2.121, 0.694, 0.0, 60.0, true},	// 11:大殿ごもる
};



typedef enum {
	JOY_AXES_MANUAL_VERTICAL     = 1,   // front or back (PS4：左スティック)      [joy_msg.axes]
	JOY_AXES_MANUAL_HORIZONTAL   = 0,   // Right or Left (PS4：左スティック)      [joy_msg.axes]
	JOY_AXES_CURSOR_VERTICAL     = 7,   // Up or Down    (PS4：十字キー)          [joy_msg.axes]
	JOY_AXES_CURSOR_HORIZONTAL   = 6,   // Right or Left (PS4: 十字キー)          [joy_msg.axes]
} JOY_AXES;

typedef enum {
	JOY_BUTTON_BRUSH             = 0,   // Brush ON/OFF     (PS4:×ボタン)  [joy_msg.buttons]
	JOY_BUTTON_NORMAL_MODE       = 2,   // Set Normal Mode  (PS4:△ボタン)  [joy_msg.buttons]
	JOY_BUTTON_AUTOMODE          = 5,   // Set Auto Mode    (PS4:L1ボタン)  [joy_msg.buttons]
	JOY_BUTTON_MANUALMODE        = 4,   // Set Manual Mode  (PS4:R1ボタン)  [joy_msg.buttons]
	JOY_BUTTON_MANUALBOOST       = 3,   // Boost Move        (PS4;□ボタン) [joy_msg.buttons]
	JOY_BUTTON_MEANDER_MODE      = 1,   // Meander Mode      (PS4:〇ボタン) [joy_msg.buttons]
	JOY_BUTTON_START             = 13,  // START (PS4:中央タッチパッド)             [joy_msg.buttons]
	JOY_BUTTON_RESET             = 12,  // Reset (PS4:PSボタン)     [joy_msg.buttons]
} JOY_BUTTON;

typedef enum {
	RULO_PLAN_STATE_STANDBY = 0,      // Nothing to do
	RULO_PLAN_STATE_MOVE_KNOWNGOAL,   // Go to known goal
	RULO_PLAN_STATE_MOVE_NEARENEMY,  // Go to near coffee(adjustment)
	RULO_PLAN_STATE_MOVE_NEARTARGET,  // Go to near coffee(adjustment)
	RULO_PLAN_STATE_IN_CHARGE,        // Go to charge dock
	RULO_PLAN_STATE_MANUAL,           // Go to Manual Ope mode
	OTHER,
} RULO_PLAN_STATE;

typedef enum {
	RULO_PLAN_KNOWNGOAL_AROUND = 0,   // MOVE_KNOWGOAL 周回移動状態
	RULO_PLAN_KNOWNGOAL_MEANDER,      // MOVE_KNOWGOAL 蛇行移動状態
	RULO_PLAN_KNOWNGOAL_DOCKING,      // MOVE_KNOWGOAL ドッキングステーション移動状態
} RULO_PLAN_KNOWNGOAL_TARGET;


/**
 * @brief Client for 
 **/
class OnigiriPlanner{

private:

	/***************************************************************************
	* Here is the member variable declaration area
	***************************************************************************/

	// This is the node handle for public node
	ros::NodeHandle nh_;
	// this is the node handle for private node
	ros::NodeHandle private_nh_;

	// They are the publish handle
	ros::Publisher initialpose_pub;
	ros::WallTimer actmove_timer;

	// This is a actionlib client handle for moveBase
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  m_moveBaseClient;

	// MoveBaseGoal point
	move_base_msgs::MoveBaseGoal move_goal;
	int global_goal_id;
	int ioState;	//0:なし、1:遷移初回時(in)、2:次状態への遷移前(out)

	//camera detect
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub;
	cv::Mat hsv;
	cv::Mat mask;
	cv::Mat image;
	double m_diffPos;
	double m_enemy_det_time;
	bool m_back_run;


  	// PS4 Controller node (for debug)   
	ros::Subscriber joy_sub,targetid_sub;
	ros::Publisher twist_pub;
	double start_time, diff_time;
	geometry_msgs::Twist cmd_vel_joy, cmd_vel_joy_p, cmd_vel_hack;

	// target_id topic
	std_msgs::String m_knownID[16];
	int m_knownID_num;
	bool m_isNewID;
	double near_front_time, near_wait_time, near_back_time;

	int m_near_tgt_state;

	// This parameter mentions what rulo currently doing
	RULO_PLAN_STATE m_state;
	RULO_PLAN_STATE p_state;	// Manualモードとの遷移用
	RULO_PLAN_STATE m_action;
	RULO_PLAN_KNOWNGOAL_TARGET m_goal_target;

	/***************************************************************************
	* move_base_client 初期関数
	***************************************************************************/
	void movebase_start(void){
		geometry_msgs::PoseWithCovarianceStamped initial_pose;

		//wait for the action server to come up
		ROS_INFO("Waiting move_base action server");
		m_moveBaseClient.waitForServer(ros::Duration(1.0));

		//cancel a previous goal information
		m_moveBaseClient.cancelAllGoals();

		//send initialpose
		initial_pose.header.frame_id = "map";
		initial_pose.header.stamp = ros::Time::now();
		initial_pose.pose.pose.position.x    = start_ptn.pos_x;
		initial_pose.pose.pose.position.y    = start_ptn.pos_y;
		initial_pose.pose.pose.orientation.z = start_ptn.ori_z;
		initial_pose.pose.pose.orientation.w = sqrt(1-pow(start_ptn.ori_z,2));
		initial_pose.pose.covariance[0] = 0.25;
		initial_pose.pose.covariance[7] = 0.25;
		initial_pose.pose.covariance[35] = 0.06853891945200942;

		initialpose_pub.publish(initial_pose);

    	return;
    }


	/***************************************************************************
	* 状態遷移動作設定
	***************************************************************************/
	void ActState(){
		//the move_base state machine, handles the control logic for navigation
		switch(m_state){


		/*************************			
		*  STANDBY Mode
		*************************/	
		case RULO_PLAN_STATE_STANDBY:
			// 初回遷移時 
			if(ioState==1){
				ROS_INFO("state : STANDBY");

				if(p_state != RULO_PLAN_STATE_MANUAL){
					// 初期化処理(move_base初期位置設定など）
					OnigiriPlanner::movebase_start();
				
					//設定値初期化(リセット時)
					global_goal_id = 0;
					m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
				}
				// STANDBY遷移初回時、時刻情報を習得
				start_time = ros::Time::now().toSec();
				m_enemy_det_time = ros::Time::now().toSec();
			}

			// 常時処理
			//5秒立った or PSボタン押しで、MOVE_KNOWNGOALに遷移する（仮仕様)
			diff_time = ros::Time::now().toSec() - start_time;
			if(diff_time< 1.0){
				cmd_vel_hack.linear.x = 0;
				cmd_vel_hack.angular.z = 0.5;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY1, %f",diff_time);
			}else if(diff_time <3.0){
				cmd_vel_hack.linear.x = 0.5;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY2, %f",diff_time);
			}else if(diff_time <3.5){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY3, %f",diff_time);
			}else if(diff_time <5.35){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = -1.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY4, %f",diff_time);
			}else if(diff_time <7.8){
				cmd_vel_hack.linear.x = 1.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY5, %f",diff_time);
			}else if(diff_time <9.0){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 1.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY6, %f",diff_time);
			}else if(diff_time <10.0){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY7, %f",diff_time);
			}else if(diff_time <11.5){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 1.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY8, %f",diff_time);
			}else if(diff_time <12.6){
				cmd_vel_hack.linear.x = 1.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY9, %f",diff_time);
			}else if(diff_time <13.75){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = -1.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY10, %f",diff_time);
			}else if(diff_time <14.75){
				cmd_vel_hack.linear.x = 1.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY11, %f",diff_time);
			}else if(diff_time <16.1){
				cmd_vel_hack.linear.x = -1.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("sub-s : STANDBY12, %f",diff_time);
			}else{
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);

				//geometry_msgs::PoseWithCovarianceStamped initial_pose;
				//initial_pose.header.frame_id = "map";
				//initial_pose.header.stamp = ros::Time::now();
				//initial_pose.pose.pose.position.x    = start_ptn.pos_x;
				//initial_pose.pose.pose.position.y    = start_ptn.pos_y;
				//initial_pose.pose.pose.orientation.z = start_ptn.ori_z;
				//initial_pose.pose.pose.orientation.w = sqrt(1-pow(start_ptn.ori_z,2));
				//initial_pose.pose.covariance[0] = 0.25;
				//initial_pose.pose.covariance[7] = 0.25;
				//initial_pose.pose.covariance[35] = 0.06853891945200942;

				//initialpose_pub.publish(initial_pose);

				m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
				ioState = 2;
			}    


			//完了後にmovebaseモードに移行
			//initialpose_pub.publish(initial_pose);

			// 遷移前処理
			if(ioState==2){
				// /move_base/clear_costmaps サービスを発行
				//if(clear_costmap_client.call(clear_costmap_req, clear_costmap_res))	ROS_INFO("clear_costmap!");
				//else ROS_INFO("clear_costmap error!");
			}
	        break;


		/*************************			
		*  MOVE_KNOWNGOAL Mode
		*************************/
		case RULO_PLAN_STATE_MOVE_KNOWNGOAL:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : MOVE_KNOWNGOAL,  goal target[%d]",  m_goal_target);

				// dynamic reconfigure設定
				dynamic_reconfigure::ReconfigureRequest dwa_srv_req;
				dynamic_reconfigure::ReconfigureResponse  dwa_srv_resp;
				dynamic_reconfigure::DoubleParameter  dwa_double_param;
				dynamic_reconfigure::Config dwa_conf;

				// AROUND動作時、既存Goalリストを送信
				if(m_goal_target == RULO_PLAN_KNOWNGOAL_AROUND){

					move_goal.target_pose.header.frame_id = "map";
					move_goal.target_pose.header.stamp = ros::Time::now();
					move_goal.target_pose.pose.position.x = goal_ptn[global_goal_id].pos_x;
					move_goal.target_pose.pose.position.y = goal_ptn[global_goal_id].pos_y;
					move_goal.target_pose.pose.orientation.z = goal_ptn[global_goal_id].ori_z;
					move_goal.target_pose.pose.orientation.w = sqrt(1-pow(goal_ptn[global_goal_id].ori_z,2));
					ROS_INFO("Sending Goal: No.%d [%0.3f,%0.3f,%0.3f]", global_goal_id+1, goal_ptn[global_goal_id].pos_x, goal_ptn[global_goal_id].pos_y, goal_ptn[global_goal_id].ori_z);

					//バック走行の可否を判断
					m_back_run =goal_ptn[global_goal_id].back_run;
					if(global_goal_id == 7 && (ros::Time::now().toSec() - m_enemy_det_time) < 5){
						m_back_run = true;
					} 

                	if(m_back_run == true){
						dwa_double_param.value = 0.1;
					}else{
						dwa_double_param.value = 0.5;
					}
					dwa_double_param.name = "max_vel_x";
					dwa_conf.doubles.push_back(dwa_double_param);

					dwa_srv_req.config = dwa_conf;
					ros::service::call("/move_base/DWAPlannerROS/set_parameters", dwa_srv_req, dwa_srv_resp);
					ROS_INFO("max_vel_x : %f", dwa_double_param.value);
				}
				// Docking検出モードの場合、既存Docking位置座標を送信(未使用）
				else if(m_goal_target == RULO_PLAN_KNOWNGOAL_DOCKING){

					move_goal.target_pose.header.frame_id = "map";
					move_goal.target_pose.header.stamp = ros::Time::now();
					move_goal.target_pose.pose.position.x = docking_ptn.pos_x;
					move_goal.target_pose.pose.position.y = docking_ptn.pos_y;
					move_goal.target_pose.pose.orientation.z = docking_ptn.ori_z;
					move_goal.target_pose.pose.orientation.w = sqrt(1-pow(docking_ptn.ori_z,2));
					ROS_INFO("Sending Goal: Docking [%0.3f,%0.3f,%0.3f]", docking_ptn.pos_x, docking_ptn.pos_y, docking_ptn.ori_z);
				}
				m_moveBaseClient.sendGoal(move_goal, boost::bind(&OnigiriPlanner::doneMovingKnownGoalCb, this, _1, _2), 0, 0);
			
			}

			// 常時処理

			// 遷移前処理
			if(ioState==2){
				//別状態に遷移する場合のみ cancelGoal処理
				m_moveBaseClient.cancelGoal();
			}
			break;


		/*************************			
		*  MOVE_NEARENEMY Mode
		*************************/	

		case RULO_PLAN_STATE_MOVE_NEARENEMY:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : MOVE_NEARENEMY");
				start_time = ros::Time::now().toSec();
			}

			// 常時処理
			diff_time = ros::Time::now().toSec() - start_time;
			if(diff_time < 2.0){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.005* m_diffPos;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("Near Ememy Mode[1], %f, %f",cmd_vel_hack.linear.x, cmd_vel_hack.angular.z);
			}else if(diff_time <4.0){
				cmd_vel_hack.linear.x = 0.2;
				cmd_vel_hack.angular.z = 0.005* m_diffPos;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("Near Ememy Mode[2], %f, %f",cmd_vel_hack.linear.x, cmd_vel_hack.angular.z);
			}else if(diff_time >=4.0){
				cmd_vel_hack.linear.x = 0.3;
				cmd_vel_hack.angular.z = 0.005* m_diffPos;
				twist_pub.publish(cmd_vel_hack);
				ROS_INFO("Near Ememy Mode[3], %f, %f",cmd_vel_hack.linear.x, cmd_vel_hack.angular.z);
			}

			// 遷移前処理
			if(ioState==2){
				cmd_vel_hack.linear.x = -0.3;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ros::Duration(1).sleep();

				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				ros::Duration(3).sleep();
			}
			break;

		/*************************			
		*  MOVE_NEARTARGET Mode
		*************************/	

		case RULO_PLAN_STATE_MOVE_NEARTARGET:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : MOVE_NEAR_TARGET");
			    m_near_tgt_state = 0;
				ROS_INFO("NEAR TARGET : 0");

				near_front_time = goal_ptn[global_goal_id].front_time;
				near_wait_time  = goal_ptn[global_goal_id].wait_time;
				near_back_time  = goal_ptn[global_goal_id].front_time;

				// 遷移初回時、時刻情報を習得
				start_time = ros::Time::now().toSec();
			}

			diff_time = ros::Time::now().toSec() - start_time;

			// 常時処理
			// 前進処理時
			if(m_near_tgt_state == 0){
				cmd_vel_hack.linear.x = 1.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				if(m_isNewID){
					start_time = ros::Time::now().toSec();
					near_back_time = diff_time;
					ROS_INFO("NEAR TARGET : 2");
					m_near_tgt_state = 2;
				}
				if(diff_time > near_front_time){
					start_time = ros::Time::now().toSec();
					ROS_INFO("NEAR TARGET : 1");
					m_near_tgt_state = 1;
				}
			}else if(m_near_tgt_state == 1){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				if(diff_time > near_wait_time || m_isNewID){
					start_time = ros::Time::now().toSec();
					ROS_INFO("NEAR TARGET : 2");
					m_near_tgt_state = 2;
				}
			}else if(m_near_tgt_state == 2){
				cmd_vel_hack.linear.x = -1.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);
				if(diff_time > near_back_time){
					start_time = ros::Time::now().toSec();
					m_near_tgt_state = 0;
					ROS_INFO("NEAR TARGET : 3");
					ioState = 2;
				}
			}


			// 遷移前処理
			if(ioState==2){
				cmd_vel_hack.linear.x = 0.0;
				cmd_vel_hack.angular.z = 0.0;
				twist_pub.publish(cmd_vel_hack);

				// 次のWAYPOINT周回モードに移行
				m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
				//goal_id更新
				global_goal_id = (global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1) ? 0 : global_goal_id+1); 

				// SKIPゴール判定 (ID=2or5or8 で直近10秒以内で敵を検出していた場合は１つskip）
				ROS_INFO("Enemy detect: %f", (ros::Time::now().toSec() - m_enemy_det_time));
				if(global_goal_id==2 || global_goal_id==5 || global_goal_id==9){
					if((ros::Time::now().toSec() - m_enemy_det_time) < 10){
					 global_goal_id++;
					 ROS_INFO("Enemy Near : Caution!! Goal Skip");
					}	
				}else if(global_goal_id==6){
					global_goal_id++;
				}
				//次のゴール設定時にNewID検出フラグはリット
				m_isNewID = false;
			}
			break;

		/*************************			
		*  IN_CHARGE Mode
		*************************/	
		case RULO_PLAN_STATE_IN_CHARGE:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : IN_CHARGE");
			}
			
			// 常時処理

			// 遷移前処理
			if(ioState==2){

			}
			break;


		/*************************			
		*  MANUAL Mode
		*************************/	
		case RULO_PLAN_STATE_MANUAL:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : MANUAL");
			}
			
			// 常時処理
			// cmd_vel更新
			twist_pub.publish(cmd_vel_joy);
			if(cmd_vel_joy_p.linear.x != cmd_vel_joy.linear.x || cmd_vel_joy_p.angular.z != cmd_vel_joy.angular.z ){
				ROS_INFO("[Manual mode]linear.x: %0.2f, angular.z: %0.2f",cmd_vel_joy.linear.x, cmd_vel_joy.angular.z);
				cmd_vel_joy_p = cmd_vel_joy;
			}		
				
			// 遷移前処理error: no matching function for call to ‘actionlib::SimpleActionClient
			if(ioState==2){
			}
			break;
		}
		/*************************			
		*  Common
		*************************/	
		//状態更新(common)
		// ioState=2(終了処理)の場合、stateコピー
		if(ioState==2){
			p_state = m_state;
			m_state = m_action;
			ioState=1;
			ROS_INFO("m_action:%s,m_state:%s,p_state:%s",StrState(m_action).data.c_str(), StrState(m_state).data.c_str(), StrState(p_state).data.c_str());
		}
		else if(ioState==1){
			ioState=0;
		}
		return;
	}



	/***************************************************************************
	* move_base_client state監視関数
	***************************************************************************/
	void doneMovingKnownGoalCb(const actionlib::SimpleClientGoalState& goal_state, const move_base_msgs::MoveBaseResultConstPtr& result){

		//** [debug用] Manualモード時は何もしない **//
		if(m_state == RULO_PLAN_STATE_MANUAL){
			return;
		}
	
		/***************
		* GOAL到着時
		***************/
		if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("MOVE_KNONWGOAL Succeeded : (%s)", goal_state.toString().c_str());


			// 周回モードの場合、周回を継続
			if(m_goal_target == RULO_PLAN_KNOWNGOAL_AROUND){
		
				//wait_timeが0以上の場合は、_MOVE_NEARTARGETに移行
				if(goal_ptn[global_goal_id].wait_time > 0){
					m_action = RULO_PLAN_STATE_MOVE_NEARTARGET;
				}else{
					m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
					//goal_id更新
					global_goal_id = (global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1) ? 0 : global_goal_id+1); 

					// SKIPゴール判定 (ID=2or5or9 で直近10秒以内で敵を検出していた場合は１つskip）
					ROS_INFO("Enemy detect: %f", (ros::Time::now().toSec() - m_enemy_det_time));
					if(global_goal_id==2 || global_goal_id==5 || global_goal_id==9){
						if((ros::Time::now().toSec() - m_enemy_det_time) < 10){
						 global_goal_id++;
						 ROS_INFO("Enemy Near : Caution!! Goal Skip");
						}	
					}else if(global_goal_id==6){
						global_goal_id++;
					}

					//次のゴール設定時にNewID検出フラグはリセット
					m_isNewID = false;


				}
			}
			// Docking検出モードの場合、AUTO_DOCKING_STATEへ遷移
			//else if(m_goal_target == RULO_PLAN_KNOWNGOAL_DOCKING){
			//	m_action = RULO_PLAN_STATE_AUTO_DOCKING;
			//}
			ioState = 2;
		}
		/***************
		* GOAL上書き or キャンセル時
		***************/
		else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
			ROS_INFO("MOVE_KNONWGOAL Canceled : (%s)", goal_state.toString().c_str());
		}
		/***************
		* GOAL設定できず
		***************/
        else if(goal_state == actionlib::SimpleClientGoalState::ABORTED){
        	ROS_INFO("MOVE_KNONWGOAL not_setGoal : (%s)", goal_state.toString().c_str());
			if(m_back_run == true){
				cmd_vel_hack.linear.x = 0.3;
			}else{
				cmd_vel_hack.linear.x = -0.3;
			}
			cmd_vel_hack.angular.z = 0.0;
			twist_pub.publish(cmd_vel_hack);
			ros::Duration(1).sleep();

			cmd_vel_hack.linear.x = 0.0;
			cmd_vel_hack.angular.z = 0.0;
			twist_pub.publish(cmd_vel_hack);

			ioState = 2;
		}
		/***************
		* GOAL未到着時 (要因ごとの詳細はこれから実装）
		***************/
		else{
            ROS_INFO("MOVE_KNONWGOAL  Failed : (%s)F",goal_state.toString().c_str());
			
			// 次の動作まで3秒待つ(仮仕様)
			//ros::Duration(3).sleep();
			ioState = 2;
		}

		return;
	}

	/***************************************************************************
	* str_state関数
	* 状態情報をString型に変換する
	***************************************************************************/
	std_msgs::String StrState(RULO_PLAN_STATE state){
		std_msgs::String msg;
		if(state == RULO_PLAN_STATE_STANDBY) msg.data="STANDBY";
		else if(state == RULO_PLAN_STATE_MOVE_KNOWNGOAL)  msg.data="MOVE_KNOWNGOAL";
		else if(state == RULO_PLAN_STATE_MOVE_NEARENEMY) msg.data="MOVE_NEARENEMY";
		else if(state == RULO_PLAN_STATE_IN_CHARGE)    msg.data="IN_CHARGE";
		else if(state == RULO_PLAN_STATE_MANUAL)       msg.data="MANUAL";

		return msg;
	}

	/***************************************************************************
	* target_id取得関数
	***************************************************************************/
	void TargetIdCallback(const std_msgs::String &id_msg){ 
		int i;
		bool isNewID = true;
		for(i=0; i <m_knownID_num; i++){
			if(id_msg.data == m_knownID[i].data)	isNewID = false;
		}

		m_isNewID = isNewID;

		if(m_isNewID){
            ROS_INFO("GET New Target! [ID:%s]",id_msg.data.c_str());

			// MOVE_KNOWNGOALの場合、次のゴール指示
			if(m_state == RULO_PLAN_STATE_MOVE_KNOWNGOAL){
				global_goal_id = (global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1) ? 0 : global_goal_id+1);

				// SKIPゴール判定 (ID=2or5or9 で直近10秒以内で敵を検出していた場合は１つskip）
				ROS_INFO("Enemy detect: %f", (ros::Time::now().toSec() - m_enemy_det_time));
				if(global_goal_id==2 || global_goal_id==5 || global_goal_id==9){
					if((ros::Time::now().toSec() - m_enemy_det_time) < 10){
					 global_goal_id++;
					 ROS_INFO("Enemy Near : Caution!! Goal Skip");
					}	
				}else if(global_goal_id==6){
					global_goal_id++;
				}
				ioState = 2;
			}
			// MOVE_NEARENEMYの場合
			if(m_state == RULO_PLAN_STATE_MOVE_NEARENEMY){
				m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
				ioState = 2;
			}

			//m_knownID更新
			m_knownID[m_knownID_num].data = id_msg.data;
			m_knownID_num++;
		}
		return;
	}

	/***************************************************************************
	* camera画像処理関数
	***************************************************************************/
	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		const int IMG_CENTER = 200;
		const int range = 10;
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::cvtColor(cv_ptr->image, hsv, CV_BGR2HSV);	//色空間変換
		cv::inRange(hsv, cv::Scalar(60 - range, 100, 100), cv::Scalar(60 + range, 255, 255), mask); // 緑色検出でマスク画像の作成

		cv::Moments mu = cv::moments(mask, false);
		cv::Point2f mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

		double area = mu.m00;
		int x = mu.m10 / mu.m00;
		int y = mu.m01 / mu.m00;
		//ROS_INFO("AREA = %f", area);

		if ( area > 1000 ) {
			m_diffPos = -(x - IMG_CENTER);
			ROS_INFO("敵発見！！ [%f, %f]", area, m_diffPos);
			m_enemy_det_time = ros::Time::now().toSec();
			// 近くなって追いかけれる距離になれば KNOWNENEMYに移動
			if(m_state == RULO_PLAN_STATE_MOVE_KNOWNGOAL & area > 40000 ){
				m_action = RULO_PLAN_STATE_MOVE_NEARENEMY;
				ioState = 2;
			}
		}else{
			if(m_state == RULO_PLAN_STATE_MOVE_NEARENEMY & (ros::Time::now().toSec() - m_enemy_det_time) > 5){
				m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
				ioState = 2;
			}
		}
	    // Update GUI Window
	    cv::imshow("Image window", mask);
		cv::waitKey(3);
	}

	/***************************************************************************
	* PS4コントローラによる操作(debug用)
	***************************************************************************/
	void JoyConCallback(const sensor_msgs::Joy &joy_msg){ 

		/***************
		* ボタン判別
		***************/
		// リセットボタン  : 初期リセット(待機モードへ強制遷移)
		if(joy_msg.buttons[JOY_BUTTON_RESET] == 1){
			ROS_INFO("Reset OnigiriPlanner!");
		
			//state初期化(reset)
			m_state =  RULO_PLAN_STATE_STANDBY;
			m_action = RULO_PLAN_STATE_STANDBY;
			m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
			global_goal_id = 0;
			ioState = 1;
			//OnigiriPlanner::ActState();
		}
    	// スタートボタン  : 待機状態からの遷移
		if(joy_msg.buttons[JOY_BUTTON_START] == 1 && m_state==RULO_PLAN_STATE_STANDBY){
			ROS_INFO("start OnigiriPlanner!");
		
			//STANBYからの遷移
			m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
			ioState = 2;
		}
    	// 十字ボタン(上) : KNOWNGOAL時 一つ先のゴール位置に移動 (MOVE_KNOWNGOAL の 周回モード ⇛ Dockingへ遷移)
		if(joy_msg.axes[JOY_AXES_CURSOR_VERTICAL] == 1.0 && m_state==RULO_PLAN_STATE_MOVE_KNOWNGOAL){
			ROS_INFO("send next goal!");
			if(m_goal_target == RULO_PLAN_KNOWNGOAL_AROUND || m_goal_target == RULO_PLAN_KNOWNGOAL_MEANDER){
				if(global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1)){
					m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
				}else{
					 global_goal_id++;
				}
			}
			else{
				m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
				global_goal_id = 0;
			}
			//	global_goal_id = (global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1) ? 0 : global_goal_id+1); 
			ioState = 2;
		}
    	// 十字ボタン(下) : KNOWNGOAL時一つ前のゴール位置に移動 (MOVE_KNOWNGOAL の 周回モードのみ)
		if(joy_msg.axes[JOY_AXES_CURSOR_VERTICAL] == -1.0 && m_state==RULO_PLAN_STATE_MOVE_KNOWNGOAL){
			ROS_INFO("send previous goal!");
			if(m_goal_target == RULO_PLAN_KNOWNGOAL_AROUND|| m_goal_target == RULO_PLAN_KNOWNGOAL_MEANDER){
				if(global_goal_id == 0){
					m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
				}else{
					 global_goal_id--;
				}
			}			
			else{
				m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
				global_goal_id = ((sizeof(goal_ptn)/sizeof(Point))-1);
			}
			//global_goal_id = (global_goal_id == 0 ? ((sizeof(goal_ptn)/sizeof(Point))-1)  : global_goal_id-1); 
			ioState = 2;
		}

    	//十字ボタン(右) : 一つ先の状態に移動
		if(joy_msg.axes[JOY_AXES_CURSOR_HORIZONTAL] == -1.0){
			if(m_state != RULO_PLAN_STATE_MANUAL){
				m_action = (m_action ==  RULO_PLAN_STATE_IN_CHARGE) ? RULO_PLAN_STATE_STANDBY : RULO_PLAN_STATE(m_action+1);
				ROS_INFO("change forward state");
				ioState = 2;
			}else{
				p_state = (p_state ==  RULO_PLAN_STATE_IN_CHARGE) ? RULO_PLAN_STATE_STANDBY : RULO_PLAN_STATE(p_state+1);
				ROS_INFO("[Manual Mode] change p_state: %s", StrState(p_state).data.c_str());
			}
		}
    	// 十字ボタン(左) : 一つ前の状態に移動
		if(joy_msg.axes[JOY_AXES_CURSOR_HORIZONTAL] == 1.0){
			if(m_state != RULO_PLAN_STATE_MANUAL){
				m_action = (m_action == RULO_PLAN_STATE_STANDBY) ?  RULO_PLAN_STATE_IN_CHARGE : RULO_PLAN_STATE(m_action-1);
				ROS_INFO("change back state");
				ioState = 2;
			}else{
				p_state = (p_state == RULO_PLAN_STATE_STANDBY) ?  RULO_PLAN_STATE_IN_CHARGE : RULO_PLAN_STATE(p_state-1);
				ROS_INFO("[Manual Mode] change p_state: %s", StrState(p_state).data.c_str());
			}
		}
    	
		
		/***************
		* マニュアルモード時との遷移
		***************/
		
    	// Manualモードとの遷移コード
		//  L1ボタン[4] : Manualモード
		//  R1ボタン[5] : 自律移動モード
		if(joy_msg.buttons[JOY_BUTTON_MANUALMODE] == 1 && m_state != RULO_PLAN_STATE_MANUAL){
			// Manualモードへ移動時、前状態をスタック
			p_state = m_state;
			m_action = RULO_PLAN_STATE_MANUAL;
			ioState = 2;
			OnigiriPlanner::ActState();
		}else if(joy_msg.buttons[JOY_BUTTON_AUTOMODE] == 1 && m_state == RULO_PLAN_STATE_MANUAL){
			// Manualモードから前状態へ遷移する場合
    		m_action = p_state;
			ioState = 2;
			OnigiriPlanner::ActState();
		}

    	/***************
		* マニュアルモード時の操作
    	***************/
    	// JoyCon 操作
		int boost;
		if(m_state == RULO_PLAN_STATE_MANUAL){
			// □ボタン (マニュアル操作時速度加速)
			boost = (joy_msg.buttons[JOY_BUTTON_MANUALBOOST] == 1) ? 3 : 1;
			cmd_vel_joy.linear.x = joy_msg.axes[JOY_AXES_MANUAL_VERTICAL]*0.25*boost;
			cmd_vel_joy.angular.z = joy_msg.axes[JOY_AXES_MANUAL_HORIZONTAL]*1*boost;
    	}
		
   	
   	return;
    }
 



public:

	/**
	* @brief Constructor for the client.
	*/
	OnigiriPlanner():
		nh_(),
		it_(nh_),
		m_moveBaseClient("move_base",true)
	{

		actmove_timer   = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&OnigiriPlanner::ActState, this));
		initialpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
		joy_sub         = nh_.subscribe("joy", 10, &OnigiriPlanner::JoyConCallback, this);
		targetid_sub    = nh_.subscribe("target_id", 10, &OnigiriPlanner::TargetIdCallback, this);
		image_sub       = it_.subscribe("image_raw", 1, &OnigiriPlanner::imageCb, this);


    	//state&action initialize
		m_state =  RULO_PLAN_STATE_STANDBY;
		p_state =  RULO_PLAN_STATE_STANDBY;
		m_action = RULO_PLAN_STATE_STANDBY;
		ioState = 1;
		m_knownID_num = 0;
		m_isNewID = false;

		//debug(Manualモード用)
		twist_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;
		twist_pub.publish(twist);

		cv::namedWindow("Image window");

		ROS_INFO("OnigiriPlanner Constructor is finished!");
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "OnigiriPlanner");
	OnigiriPlanner client;

	ros::spin();
    
    //Ctrl-C処理
	ros::NodeHandle nh_;
	ros::Publisher twist_pub;
	twist_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	geometry_msgs::Twist twist;
	twist.linear.x = 0.0;
	twist.angular.z = 0.0;
	twist_pub.publish(twist);

	return 0;
}

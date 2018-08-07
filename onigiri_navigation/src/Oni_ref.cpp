/**
 * \file OnigiriPlanner.cpp
 * \brief  
 * \date 2017.11.14  ver.0.1 第二回(12/12)中間発表向けに状態遷移ベース作成(M.Hirose)
 *  date 2017.11.22  ver.0.2 Arm Client処理追加
 *
 * \note
 *
 */
#include <ros/ros.h>
#include <ros/wall_timer.h>
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_control/ArmControlAction.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>

#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <std_msgs/Char.h>
#include <std_msgs/String.h>  
#include <std_srvs/Empty.h> 

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <rulo_msgs/Battery.h>
#include <rulo_msgs/IRCharacter.h>
#include <rulo_msgs/BrushesPWM_cmd.h>

#include <rulo7_msgs/NearCoffeeAction.h>
#include <rulo7_msgs/AutoDockingAction.h>
#include <rulo7_msgs/DockingIRchar.h>
#include <rulo7_msgs/MakeMeanderPlan.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#define MODEVOICE_ON 

struct Point{
    float pos_x;
    float pos_y;
    float ori_z;
};

/***
 ex) hirose home
Point start_ptn   = {-0.361, 2.759, 0.776}; //0.629
Point docking_ptn = { 0.000, 2.800, 0.000}; //1.000
Point goal_ptn[] = {
   { -0.391, 3.524,  0.954}, //0.297
   { -0.506, 4.907,  0.060}, //0.998
   {  0.925, 2.894,  0.000}};//1.000
**/
/***
 ex) kensyu course  
Point start_ptn = { 0.16,  0.24, 0.004};
Point docking_ptn = { 0.45, 0.36, -0.94};
Point goal_ptn[] = {
   {  1.405, 0.238, -0.113},
   {  2.916, 0.063,  0.386},
   {  3.056, 1.386,  0.936},
   {  1.558, 1.508, -0.709},
   {  1.450, 0.419, -0.955},
   {  0.363, 0.355,  0.828},
   {  0.320, 1.539,  0.001},
   {  1.466, 1.392, -0.666}};
***/
/***
 ex) seminer room honban (18/2/26) **/
Point start_ptn = {-0.12, -0.07, -0.01};
Point docking_ptn = {-0.00, -0.07, 0.956};
Point goal_ptn[] = {
   {  2.077,-0.648, -0.736},
   {  2.058,-2.033, -0.010},
   {  3.685,-2.265,  0.678},
   {  3.742,-0.362, -0.997},
   {  2.077,-0.648, -0.736},
   {  1.882,-2.082,  0.999},
   { -0.107,-1.916,  0.674},
   {  0.080,-0.199, -0.046}};

/***
 ex) creative caffe (kadoma) 
Point start_ptn = { 0.0206, -0.2529, 0.00729};
Point docking_ptn = {-0.186, -0.166,  0.925};
Point goal_ptn[] = {
   {  1.893, 0.152,  0.074},
   {  3.037, 0.004, -0.291},
   {  3.442,-0.451, -0.708},
   {  3.271,-2.023, -0.998},
   {  1.529,-1.841,  0.996},
   {  0.449,-1.685,  0.939},
   {  0.052,-0.613,  0.547}};
***/

Point armgoal_ptn = { 0.150, 0.000, 0.100};



	/***************************************************************************
	* PS4/JOYコントローラによる操作(debug用)
	* JOY_AXES_MANUAL_VERTICAL   : マニュアルモードでの前後移動操作
	* JOY_AXES_MANUAL_HORIZONTAL : マニュアルモードでの左右回転操作
	* JOY_AXES_CURSOR_VERTICAL   :  GOAL遷移、FLAG切替え (自律移動モードのみ有効) 
		- KNOWNGOAL状態時のUP側   : 一つ先のゴール位置に移動 (MOVE_KNOWNGOAL の 周回モード ⇛ Dockingへ遷移)
		- KNOWNGOAL状態時のDOWN側 : 一つ前のゴール位置に移動 (MOVE_KNOWNGOAL の 周回モード ⇛ Dockingへ遷移)
		- NEARCOFFEE状態時 :  缶保持有無の変更(flagのTrue/False切り替え)
	* JOY_AXES_CURSOR_HORIZONTAL : State遷移
		- Manualモードでない場合のUP側   ： 一つ先の状態に移動 (例えば、MOVE_KNOWNGOALの場合、NEARCOFFEEに強制遷移)
		- Manualモードでない場合のDOWN側  : 一つ前の状態に移動 (例えば、MOVE_KNOWNGOALの場合、STANDBYに強制遷移)
		- Manualモードの場合のUP側        : Manual→自律移動モードへ遷移する際に遷移先の状態を一つ先に移動させておく
		- Manualモードの場合のDOWN側      : Manual→自律移動モードへ遷移する際に遷移先の状態を一つ前に移動させておく
		   ※使用例
			・KNOWNGOAL状態時で一旦マニュアルモードに遷移 (JOY_BUTTON_MANUALMODE押し)、
			  その後にARM_CONTROL状態から自律移動モードを再スタートさせたい場合はJOY_AXES_CURSOR_HORIZONTALをUP側に2回押し、
			  その後にJOY_BUTTON_AUTOMODE押しをすれば、ARM_CONTROL状態から自律移動が再スタートする仕様
	*
	* JOY_BUTTON_BRUSH       :  ブラシON/OFFの切り替え
	* JOY_BUTTON_NORMAL_MODE :  緊急停止回避モード(set_normal.shの代わり)
	* JOY_BUTTON_AUTOMODE    :  自律移動モードへの遷移 (マニュアルモードから自律移動モードへ遷移したい場合に使用)
	* JOY_BUTTON_MANUALMODE  :  マニュアルモードへの遷移 (自律移動モードからマニュアルモードへ遷移したい場合に使用)
	* JOY_BUTTON_MANUALBOOST :  マニュアルモード時の前後移動の速度アップ (Bダッシュするイメージ)
	* JOY_BUTTON_START       :  待機状態からMOVE_KNOWNGOAL状態への遷移
	* JOY_BUTTON_RESET       :  全状態から待機状態へのリセット遷移
	* JOY_BUTTON_MEANDER_MODE : MOVE_KNOWNGOAL状態で 周回モード↔ 蛇行モードの遷移 (ボタン押の度に遷移)
	***************************************************************************/

#if 1
// Kadoma
typedef enum {
	JOY_AXES_MANUAL_VERTICAL     = 1,   // front or back (PS4：左スティック)      [joy_msg.axes]
	JOY_AXES_MANUAL_HORIZONTAL   = 0,   // Right or Left (PS4：左スティック)      [joy_msg.axes]
	JOY_AXES_CURSOR_VERTICAL     = 7,   // Up or Down    (PS4：十字キー)          [joy_msg.axes]
	JOY_AXES_CURSOR_HORIZONTAL   = 6,   // Right or Left (PS4: 十字キー)          [joy_msg.axes]
} JOY_AXES;

typedef enum {
	JOY_BUTTON_BRUSH             = 1,   // Brush ON/OFF     (PS4:×ボタン)  [joy_msg.buttons]
	JOY_BUTTON_NORMAL_MODE       = 3,   // Set Normal Mode  (PS4:△ボタン)  [joy_msg.buttons]
	JOY_BUTTON_AUTOMODE          = 5,   // Set Auto Mode    (PS4:L1ボタン)  [joy_msg.buttons]
	JOY_BUTTON_MANUALMODE        = 4,   // Set Manual Mode  (PS4:R1ボタン)  [joy_msg.buttons]
	JOY_BUTTON_MANUALBOOST       = 0,   // Boost Move        (PS4;□ボタン) [joy_msg.buttons]
	JOY_BUTTON_MEANDER_MODE      = 2,   // Meander Mode      (PS4:〇ボタン) [joy_msg.buttons]
	JOY_BUTTON_START             = 13,  // START (PS4:中央タッチパッド)             [joy_msg.buttons]
	JOY_BUTTON_RESET             = 12,  // Reset (PS4:PSボタン)     [joy_msg.buttons]
} JOY_BUTTON;
#else
// Yokohama
typedef enum {
	JOY_AXES_MANUAL_VERTICAL     = 3,   // front or back (PS4：左スティック)      [joy_msg.axes]
	JOY_AXES_MANUAL_HORIZONTAL   = 2,   // Right or Left (PS4：左スティック)      [joy_msg.axes]
	JOY_AXES_CURSOR_VERTICAL     = 1,   // Up or Down    (PS4：十字キー)          [joy_msg.axes]
	JOY_AXES_CURSOR_HORIZONTAL   = 0,   // Right or Left (PS4: 十字キー)          [joy_msg.axes]
} JOY_AXES;

typedef enum {
	JOY_BUTTON_BRUSH             = 2,   // Brush ON/OFF     (PS4:×ボタン)  [joy_msg.buttons]
	JOY_BUTTON_NORMAL_MODE       = 1,   // Set Normal Mode  (PS4:△ボタン)  [joy_msg.buttons]
	JOY_BUTTON_AUTOMODE          = 4,   // Set Auto Mode    (PS4:L1ボタン)  [joy_msg.buttons]
	JOY_BUTTON_MANUALMODE        = 5,   // Set Manual Mode  (PS4:R1ボタン)  [joy_msg.buttons]
	JOY_BUTTON_MANUALBOOST       = 0,   // Boost Move        (PS4;□ボタン) [joy_msg.buttons]
	JOY_BUTTON_START             = 11,  // START (PS4:PSボタン)             [joy_msg.buttons]
	JOY_BUTTON_RESET             = 10,  // Reset (PS4:中央タッチパット)     [joy_msg.buttons]
} JOY_BUTTON;
#endif

typedef enum {
	RULO_PLAN_STATE_STANDBY = 0,      // Nothing to do
	RULO_PLAN_STATE_MOVE_KNOWNGOAL,   // Go to known goal
	RULO_PLAN_STATE_MOVE_NEARCOFFEE,  // Go to near coffee(adjustment)
	RULO_PLAN_STATE_ARM_CONTROL,      // In arm controling with /Rulo/cmd_vel == STOPPED
	RULO_PLAN_STATE_AUTO_DOCKING,     // Go to charge dock
	RULO_PLAN_STATE_IN_CHARGE,        // Go to charge dock
	RULO_PLAN_STATE_MANUAL,           // Go to Manual Ope mode
	OTHER,
} RULO_PLAN_STATE;

typedef enum {
	RULO_PLAN_KNOWNGOAL_AROUND = 0,   // MOVE_KNOWGOAL 周回移動状態
	RULO_PLAN_KNOWNGOAL_MEANDER,      // MOVE_KNOWGOAL 蛇行移動状態
//	RULO_PLAN_KNOWNGOAL_COFFEE,       // MOVE_KNOWGOAL コーヒー缶移動状態
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

	// They are the subscribe handle
	ros::Subscriber  tgtobj_sub;
	ros::Subscriber  docking_irchar_sub;
	ros::Subscriber  docking_battery_sub;
	ros::Subscriber  amclpose_sub;
	ros::Subscriber  globalmap_sub;
	
	// This is a actionlib client handle for moveBase
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  m_moveBaseClient;

	// This is a actionlib client handle for moveNearCoffee
	actionlib::SimpleActionClient<rulo7_msgs::NearCoffeeAction>  m_nearCoffeeClient;
	
	// This is a actionlib client handle for armControl
	actionlib::SimpleActionClient<arm_control::ArmControlAction>  m_armControlClient;	

	// This is a actionlib client handle for autodocking shiraya
	actionlib::SimpleActionClient<rulo7_msgs::AutoDockingAction>  m_autoDockingClient;	
	
	// MoveBaseGoal point
	move_base_msgs::MoveBaseGoal move_goal;
	int global_goal_id;
	int ioState;	//0:なし、1:遷移初回時(in)、2:次状態への遷移前(out)
    std_srvs::Empty::Request clear_costmap_req;
    std_srvs::Empty::Response clear_costmap_res;
    ros::ServiceClient clear_costmap_client;
	geometry_msgs::PoseWithCovarianceStamped m_pose;	//現在のmap上の自己位置情報
	//nav_msgs::OccupancyGrid global_map;

	//  MoveBaseGoal Meander handle
	ros::ServiceClient meander_client;
	rulo7_msgs::MakeMeanderPlan::Request meander_req;
	rulo7_msgs::MakeMeanderPlan::Response meander_res;
	int local_goal_id;
	
	
	// MoveNearCoffee point
	rulo7_msgs::NearCoffeeGoal coffee_goal;

	// ArmControl point
	arm_control::ArmControlGoal arm_goal;
	
	// AutoDocking point
	rulo7_msgs::AutoDockingGoal autodocking_goal;
	
    // PS4 Controller node (for debug)   
	ros::Subscriber joy_sub;
	ros::Publisher modevoice_pub;
	std_msgs::Char modevoice;
	ros::Publisher twist_pub;
	ros::Publisher mode_pub;
	ros::Publisher brush_pub;
	geometry_msgs::Twist cmd_vel_joy, cmd_vel_joy_p, cmd_vel_dock;
	bool brush_on;
	std_msgs::String rulomode;
	double start_time, diff_time;

	// coffee information
	geometry_msgs::PoseStamped m_coffee_pose;
	int getCoffee_status;		//コーヒ把持フラグ 0:未把持、1:把持しようとして失敗, 2: 把持

	// IR Character information
	rulo7_msgs::DockingIRchar m_dockir, p_dockir;
	float docking_dit;
	float docking_deg;
	int docking_state,prev_dockig_state;
	bool docking_cmdvel_zero_flag;

	/*****************************************************************
	 * This is a state variable managed as main task for this class
	*****************************************************************/
	// This parameter mentions what rulo currently doing
	RULO_PLAN_STATE m_state;
	RULO_PLAN_STATE p_state;	// Manualモードとの遷移用
	RULO_PLAN_KNOWNGOAL_TARGET m_goal_target;

	// This parameter mentions what rulo wants to do when m_state is in idle
	RULO_PLAN_STATE m_action;

	/***************************************************************************
	* Here is the member function definition area
	***************************************************************************/

	/***************************************************************************
	* move_base_client 初期関数
	***************************************************************************/
	void movebase_start(void){
		geometry_msgs::PoseWithCovarianceStamped initial_pose;

		//wait for the action server to come up
		ROS_INFO("Waiting move_base action server");
		m_moveBaseClient.waitForServer(ros::Duration(5.0));

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
	* move_nearcoffee 初期関数
	***************************************************************************/
	void nearcoffee_start(void){
		//wait for the action server to come up
		ROS_INFO("Waiting near_coffee action server");
		m_nearCoffeeClient.waitForServer(ros::Duration(5.0));

		//send initialpose

    	return;
	}
	

	/***************************************************************************
	* ArmController_client 初期関数
	***************************************************************************/
	void armcontrol_start(void){
		//wait for the action server to come up
		ROS_INFO("Waiting arm_control action server");
		m_armControlClient.waitForServer(ros::Duration(5.0));

		//cancel a previous goal information
		m_armControlClient.cancelAllGoals();

		//send initialpose

		return;
    }

	/***************************************************************************
	* AutoDocking_client 初期関数
	***************************************************************************/	
	void autodocking_start(void){
		//wait for the action server to come up
		ROS_INFO("Waiting autodocking action server");
		m_autoDockingClient.waitForServer(ros::Duration(5.0));

		//send initialpose

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
				modevoice.data = 0;					// STANBY
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);	//状態音声を音声読み上げ
				#endif
				
				if(p_state != RULO_PLAN_STATE_MANUAL){
					// 初期化処理(move_base初期位置設定など）
					OnigiriPlanner::movebase_start();
					OnigiriPlanner::nearcoffee_start();
					OnigiriPlanner::armcontrol_start();
					OnigiriPlanner::autodocking_start();
				
					//設定値初期化(リセット時)
					getCoffee_status = 0;
					global_goal_id = 0;
					local_goal_id = 0;
					m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
					coffee_goal.state = "fresh";
				}
				// STANDBY遷移初回時、時刻情報を習得
				start_time = ros::Time::now().toSec();
			}

			// 常時処理
			//45秒立った or PSボタン押しで、MOVE_KNOWNGOALに遷移する（仮仕様)
			//ros::Duration(10).sleep();
			diff_time = ros::Time::now().toSec() - start_time;
			//if(diff_time > 45){
			//	m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
			//	ioState = 2;
		 	//}

			// 遷移前処理
			if(ioState==2){
				// /move_base/clear_costmaps サービスを発行
				clear_costmap_client.call(clear_costmap_req, clear_costmap_res);
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

                // MoveBaseParameterの変更 (Meanderの場合、後方回転を強化)
                if(m_goal_target == RULO_PLAN_KNOWNGOAL_MEANDER){
					dwa_double_param.name = "min_vel_x";
					dwa_double_param.value = -0.5;
					dwa_conf.doubles.push_back(dwa_double_param);

					dwa_srv_req.config = dwa_conf;
					ros::service::call("/move_base/DWAPlannerROS/set_parameters", dwa_srv_req, dwa_srv_resp);
//					nh_.setParam("/move_base/DWAPlannerROS/max_vel_x",  0.5);
//					nh_.setParam("/move_base/DWAPlannerROS/min_vel_x", -0.5);
//					nh_.setParam("/move_base/DWAPlannerROS/max_rot_vel", 1.5);
//					nh_.setParam("/move_base/DWAPlannerROS/min_rot_vel",  0.3);
				}else{
					dwa_double_param.name = "min_vel_x";
					dwa_double_param.value = 0.3;
					dwa_conf.doubles.push_back(dwa_double_param);

					dwa_srv_req.config = dwa_conf;
					ros::service::call("/move_base/DWAPlannerROS/set_parameters", dwa_srv_req, dwa_srv_resp);

//					nh_.setParam("/move_base/DWAPlannerROS/max_vel_x",  1.5);
//					nh_.setParam("/move_base/DWAPlannerROS/min_vel_x",  0.2);
//					nh_.setParam("/move_base/DWAPlannerROS/max_rot_vel", 1.0);
//					nh_.setParam("/move_base/DWAPlannerROS/min_rot_vel", 0.1);
			    }
		
				// AROUND動作時、既存Goalリストを送信
				if(m_goal_target == RULO_PLAN_KNOWNGOAL_AROUND){
					if(m_state != p_state){					// 別Stateからの遷移時は
						modevoice.data = 1;					// 001_MOVE_KNOWNGOAL音声読み上げ
					}else{
						modevoice.data = 24;				// 024_move_around音声読み上げ
					}
					#ifdef MODEVOICE_ON
					modevoice_pub.publish(modevoice);
					#endif
					
					move_goal.target_pose.header.frame_id = "map";
					move_goal.target_pose.header.stamp = ros::Time::now();
					move_goal.target_pose.pose.position.x = goal_ptn[global_goal_id].pos_x;
					move_goal.target_pose.pose.position.y = goal_ptn[global_goal_id].pos_y;
					move_goal.target_pose.pose.orientation.z = goal_ptn[global_goal_id].ori_z;
					move_goal.target_pose.pose.orientation.w = sqrt(1-pow(goal_ptn[global_goal_id].ori_z,2));
					ROS_INFO("Sending Goal: No.%d [%0.3f,%0.3f,%0.3f]", global_goal_id+1, goal_ptn[global_goal_id].pos_x, goal_ptn[global_goal_id].pos_y, goal_ptn[global_goal_id].ori_z);
				}
				// コーヒ缶検出モードの場合、RealSenseカメラからの座標位置を送信
				//else if(m_goal_target == RULO_PLAN_KNOWNGOAL_COFFEE){
				//	move_goal.target_pose = m_coffee_pose;
				//	ROS_INFO("Sending Goal: Near Coffee ");
				//}
				// MEANDER(蛇行)動作時、現在地とGOAL情報をplannerに投げてlocalgoalを取得
				else if(m_goal_target == RULO_PLAN_KNOWNGOAL_MEANDER){
					//MEANDERモード初回の場合
					if(local_goal_id==0){
						modevoice.data = 26;				// 026_move_meander音声読み上げ
						#ifdef MODEVOICE_ON
						modevoice_pub.publish(modevoice);
						#endif

						// 現在位置情報
						meander_req.start.header = m_pose.header;
						meander_req.start.pose = m_pose.pose.pose;
						//m_pose.pose.pose.position.x
						//goal情報
						meander_req.goal.header.frame_id = "map";
						meander_req.goal.header.stamp = ros::Time::now();
						meander_req.goal.pose.position.x = goal_ptn[global_goal_id].pos_x;
						meander_req.goal.pose.position.y = goal_ptn[global_goal_id].pos_y;
						meander_req.goal.pose.orientation.z = goal_ptn[global_goal_id].ori_z;
						meander_req.goal.pose.orientation.w = sqrt(1-pow(goal_ptn[global_goal_id].ori_z,2));
						ROS_INFO("Sending G_Goal: No.%d [%0.3f,%0.3f,%0.3f]", global_goal_id+1, goal_ptn[global_goal_id].pos_x, goal_ptn[global_goal_id].pos_y, goal_ptn[global_goal_id].ori_z);
						
						meander_client.call(meander_req, meander_res);
						//サービスリクエスト
						//if(meander_client.call(meander_req, meander_res)){
						//	//
						//	ROS_INFO("Success to get_meander_path!");
						//}else{
						//	// 失敗した音声を追加
						//	ROS_INFO("Fail to get_meander_path!");
						//	ioState=2;
						//}
					}
					//m_moveBaseClientへのGoal情報を指示
					move_goal.target_pose = meander_res.path[local_goal_id];
					ROS_INFO("Sending Meander_Goal[%d] [%0.3f,%0.3f,%0.3f]", local_goal_id, move_goal.target_pose.pose.position.x, move_goal.target_pose.pose.position.y, move_goal.target_pose.pose.orientation.z);
				}
				// Docking検出モードの場合、既存Docking位置座標を送信
				else if(m_goal_target == RULO_PLAN_KNOWNGOAL_DOCKING){
					modevoice.data = 25;				// 025_move_docking音声読み上げ
					#ifdef MODEVOICE_ON
					modevoice_pub.publish(modevoice);
					#endif

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
				// cancelGoalの結果待ちする?
				// 別のモードに遷移する場合は local_goal_idは0にしておく
				if(m_action != RULO_PLAN_STATE_MOVE_KNOWNGOAL && m_action != RULO_PLAN_STATE_MANUAL)	local_goal_id = 0;
			}
			break;

		/*************************			
		*  MOVE_NEARCOFFEE Mode
		*************************/	
		case RULO_PLAN_STATE_MOVE_NEARCOFFEE:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : MOVE_NEARCOFFEE");
				if(getCoffee_status == 0){					// コーヒ缶把持前の場合、コーヒ缶に近づく
					modevoice.data = 2;					// MOVE_NEARCOFFEE音声読み上げ
					#ifdef MODEVOICE_ON
					modevoice_pub.publish(modevoice);
					#endif
					// Goal座標情報を送信
					// ★一旦、ARM設定座標を既知座標として設定（最終は削除予定)
					coffee_goal.state = "fresh";
					m_nearCoffeeClient.sendGoal(coffee_goal, boost::bind(&OnigiriPlanner::doneMovingNearCoffeeCb, this, _1, _2), 0, 0);
				}else{									// コーヒー缶把持後の場合、コーヒー缶から離れる
					modevoice.data = 34;				// leave_coffee音声読み上げ
					#ifdef MODEVOICE_ON
					modevoice_pub.publish(modevoice);
					#endif
					// Goal座標情報を送信
					// ★一旦、ARM設定座標を既知座標として設定（最終は削除予定)
					coffee_goal.state = "leave";
					//m_nearCoffeeClient.sendGoal(coffee_goal, boost::bind(&OnigiriPlanner::doneMovingNearCoffeeCb, this, _1, _2), 0, 0);

					// STANDBY遷移初回時、時刻情報を習得
					start_time = ros::Time::now().toSec();
				}
				
			}
			
			// 常時処理
			// getCoffee_status >= 1（把持したが失敗 or 未把持)の場合、5秒間バックする仕様
			// ※nearCoffeeClient(leave)動作の代わりの暫定処理
			if(getCoffee_status >= 1){	
				diff_time = ros::Time::now().toSec() - start_time;
                cmd_vel_dock.linear.x = -0.1;
			    cmd_vel_dock.angular.z = 0;
                twist_pub.publish(cmd_vel_dock);

				if(diff_time > 5){
					modevoice.data = 35; 				// 035_leave_coffee音声を読み上げ	
					#ifdef MODEVOICE_ON
					modevoice_pub.publish(modevoice);
					#endif

	                cmd_vel_dock.linear.x =  0;
				    cmd_vel_dock.angular.z = 0;
	                twist_pub.publish(cmd_vel_dock);

					// 次の動作まで3秒待つ(仮仕様)
					ros::Duration(3).sleep();

					ioState = 2;
                    if(getCoffee_status == 1){
    					m_action = RULO_PLAN_STATE_MOVE_NEARCOFFEE;
	    				getCoffee_status = 0;
                    }else{
    					m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
	    				m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
                    }
			 	}
			}

			// 遷移前処理
			if(ioState==2){
				m_nearCoffeeClient.cancelGoal();
				cmd_vel_dock.linear.x =  0;
				cmd_vel_dock.angular.z = 0;
	            twist_pub.publish(cmd_vel_dock);
			}
			break;

		/*************************			
		*  ARM_CONTROL Mode
		*************************/	
		case RULO_PLAN_STATE_ARM_CONTROL:
			// 初回遷移時
			if(ioState==1){
				if(m_state != p_state){				// 別Stateからの遷移時のみ
					ROS_INFO("state : ARM_CONTROL");
					modevoice.data = 3;				// ARM_CONTROL音声読み上げ
				}
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);
				#endif

				// 次の動作まで3秒待つ(仮仕様)
				ros::Duration(3).sleep();				

				/*** ARM_CONTROL Client開始処理 **/
				// Goal座標情報を送信
				// ★一旦、ARM設定座標を既知座標として設定（最終は削除予定)
				arm_goal.gripper_position.pose.position.x = 0.15;	
				// targetobject最終y位置を見てpose位置微調整
				//arm_goal.gripper_position.pose.position.y = -0.04;
				if(m_coffee_pose.pose.position.y > 0 || m_coffee_pose.pose.position.y < 0.12){
					arm_goal.gripper_position.pose.position.y = m_coffee_pose.pose.position.y-0.09;
				}else{
					arm_goal.gripper_position.pose.position.y = -0.04;
				} 
				ROS_ERROR("setting (arm_goal.gripper_position.pose.position.y): %f", arm_goal.gripper_position.pose.position.y);
				arm_goal.gripper_position.pose.position.z = 0.1;
				m_armControlClient.sendGoal(arm_goal, boost::bind(&OnigiriPlanner::doneArmControlCb, this, _1, _2), 0, 0);
			}
			
			// 常時処理

			// 遷移前処理
			if(ioState==2){
				/*** ArmController Clientの cancel_goal処理 **/
				//コーヒー缶を把持していない場合のみ cancelGoal処理
				// (把持状態でcancelGoalすると缶を落としてしまう）
				if(getCoffee_status <= 1) m_armControlClient.cancelGoal();
				// 

			}
			break;


		/*************************			
		*  AUTO_DOCKING Mode
		*************************/	
		case RULO_PLAN_STATE_AUTO_DOCKING:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : AUTO_DOCKING");
				modevoice.data = 4;					//  AUTO_DOCKING音声読み上げ
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);
				#endif

				// AUTO_DOCKING時はマニュアルモードにする
				rulomode.data = "manual";
				mode_pub.publish(rulomode);

                //ブラシOFF
				rulo_msgs::BrushesPWM_cmd brush;
				brush.main_brush = 0;
				brush.side_brush = 0;
				brush.vacuum = 0;
				brush_pub.publish(brush);

				/*** AutoDocking Client開始処理 **/
				autodocking_goal.state = "start";
				m_autoDockingClient.sendGoal(autodocking_goal, boost::bind(&OnigiriPlanner::doneAutoDockingCb, this, _1, _2), 0, 0);
				
			}

			// 常時処理
			// 遷移前処理
			if(ioState==2){
				/*** AutoDocking Clientのcancel_goal処理 **/
                m_autoDockingClient.cancelGoal();
				//cmd_vel_dock.linear.x = 0;
			    //cmd_vel_dock.angular.z = 0;
                //twist_pub.publish(cmd_vel_dock);

				// AUTO_DOCKING遷移後はノーマルモードにする
				rulomode.data = "normal";
				mode_pub.publish(rulomode);
			}
	        break;

		/*************************			
		*  IN_CHARGE Mode
		*************************/	
		case RULO_PLAN_STATE_IN_CHARGE:
			// 初回遷移時
			if(ioState==1){
				ROS_INFO("state : IN_CHARGE");
				modevoice.data = 5;					//  IN_CHARGE
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);	//状態音声を音声読み上げ
				#endif

				// 次の動作まで5秒待つ(仮仕様)
				ros::Duration(5).sleep();

				// デモ完了の音声案内(遊び）
				modevoice.data = 70;					//  70_FINISH_DEMO
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);	//状態音声を音声読み上げ
				#endif

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
				if(p_state == RULO_PLAN_STATE_STANDBY){
					modevoice.data = 6;					//  MANUAL
				}else{
					modevoice.data = 12;				//  Retry
				}
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);	//状態音声を音声読み上げ
				#endif
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
				modevoice.data = 21; 				// 021_reach_goal音声を読み上げ	
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);
				#endif
			
				// 次の動作まで3秒待つ(仮仕様)
				ros::Duration(3).sleep();
				
				m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
				//goal_id++;
				global_goal_id = (global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1) ? 0 : global_goal_id+1); 
			}
			// MEANDER(蛇行)モードの場合、
			else if(m_goal_target == RULO_PLAN_KNOWNGOAL_MEANDER){
				// global_goalに到達しているのであれば、次のglobal_goal_idを指定する
				//　modevoice.data = 21;
				//　modevoice_pub.publish(modevoice);
				if(local_goal_id >=  meander_res.path_num-1){
					modevoice.data = 21; 				// 021_reach_goal音声を読み上げ	
					#ifdef MODEVOICE_ON
					modevoice_pub.publish(modevoice);
					#endif
			
					// 次の動作まで3秒待つ(仮仕様)
					ros::Duration(3).sleep();
		
					m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
					//goal_id++;
					global_goal_id = (global_goal_id == ((sizeof(goal_ptn)/sizeof(Point))-1) ? 0 : global_goal_id+1); 
					
					local_goal_id = 0;
				}
				// global_goalでなければ
				else{
					local_goal_id++;
				}
			}
			// コーヒ缶検出モードの場合、NEAR_COFFEE_STATEへ遷移
			//else if(m_goal_target == RULO_PLAN_KNOWNGOAL_COFFEE){
			//	m_action = RULO_PLAN_STATE_MOVE_NEARCOFFEE;
			//}
			// Docking検出モードの場合、AUTO_DOCKING_STATEへ遷移
			else if(m_goal_target == RULO_PLAN_KNOWNGOAL_DOCKING){
				m_action = RULO_PLAN_STATE_AUTO_DOCKING;
			}
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
			modevoice.data = 22; 				// 022_not_setgoal音声を読み上げ	
			#ifdef MODEVOICE_ON
			modevoice_pub.publish(modevoice);	//
			#endif
 
			// meanderモードでglobal_goalに到達していなければlocal_goal_idを更新する
			if(m_goal_target==RULO_PLAN_KNOWNGOAL_MEANDER && local_goal_id<meander_res.path_num-1) local_goal_id++;

			// /move_base/clear_costmaps サービスを発行
			if(clear_costmap_client.call(clear_costmap_req, clear_costmap_res))	ROS_INFO("clear_costmap!");

			// 次の動作まで3秒待つ(仮仕様)
			ros::Duration(3).sleep();

			ioState = 2;
		}
		/***************
		* GOAL未到着時 (要因ごとの詳細はこれから実装）
		***************/
		else{
            ROS_INFO("MOVE_KNONWGOAL  Failed : (%s)",goal_state.toString().c_str());
			modevoice.data = 23; 				// 023_lostgoal音声を読み上げ	
			#ifdef MODEVOICE_ON
			modevoice_pub.publish(modevoice);	//
			#endif
			
			// meanderモードでglobal_goalに到達していなければlocal_goal_idを更新する
			if(m_goal_target==RULO_PLAN_KNOWNGOAL_MEANDER && local_goal_id<meander_res.path_num-1) local_goal_id++;

			// /move_base/clear_costmaps サービスを発行
			if(clear_costmap_client.call(clear_costmap_req, clear_costmap_res))	ROS_INFO("clear_costmap!");

			// 次の動作まで3秒待つ(仮仕様)
			ros::Duration(3).sleep();

			ioState = 2;
		}

		return;
	}

	/***************************************************************************
	*  MoveNearCoffee_client state監視関数
	***************************************************************************/
	void doneMovingNearCoffeeCb(const actionlib::SimpleClientGoalState& goal_state, const rulo7_msgs::NearCoffeeResultConstPtr& result){

    	//** [debug用] Manualモード時は何もしない **//
		if(m_state == RULO_PLAN_STATE_MANUAL){
			return;
		}
		
    	/***************
		* NearCoffee到着時
		***************/
		if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("NearCoffee Succeeded : (%s)", goal_state.toString().c_str());
	
			if(getCoffee_status == 0){					// コーヒ缶把持前の場合、ARM_CONTROLにシフト
				modevoice.data = 31; 				// 031_reach_coffee音声を読み上げ	
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);
				#endif
			
				// 次の動作まで3秒待つ(仮仕様)
				ros::Duration(5).sleep();

				ioState = 2;
				m_action = RULO_PLAN_STATE_ARM_CONTROL;
			}else{									// コーヒ缶把持後の場合、MOVE_KNOWNGOALにシフト
				modevoice.data = 35; 				// 035_leave_coffee音声を読み上げ	
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);
				#endif

				// 次の動作まで3秒待つ(仮仕様)
				ros::Duration(3).sleep();

				ioState = 2;
                if(getCoffee_status == 1){
  					m_action = RULO_PLAN_STATE_MOVE_NEARCOFFEE;
    				getCoffee_status = 0;
                }else{
   					m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
    				m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
                }
			}

		}
		/***************
		* NearCoffeeキャンセル時
		***************/
		else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
			ROS_INFO("NearCoffee Canceled : (%s)", goal_state.toString().c_str());
		}
		/***************
		* NearCoffee未到着時 (要因ごとの詳細はこれから実装）
		***************/
		else{
			ROS_INFO("NearCoffee Failed : (%s)",goal_state.toString().c_str());

			modevoice.data = 33; 				// 033_lostcoffee音声を読み上げ	
			#ifdef MODEVOICE_ON
			modevoice_pub.publish(modevoice);	//
			#endif
			
			// 次の動作まで3秒待つ(仮仕様)
			ros::Duration(3).sleep();

			ioState = 2;
		}
			
		
		return;
	}

	/***************************************************************************
	*  ArmController_client state監視関数
	***************************************************************************/
	void doneArmControlCb(const actionlib::SimpleClientGoalState& goal_state, const arm_control::ArmControlResultConstPtr& result){

		//** [debug用] Manualモード時は何もしない **//
		if(m_state != RULO_PLAN_STATE_ARM_CONTROL){
			return;
		}

    	/***************
		* ARM GOAL到着時
		***************/
		if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("ARM Control Succeeded : (%s)", goal_state.toString().c_str());
			modevoice.data = 10; 				// 010_Success音声を読み上げ
			#ifdef MODEVOICE_ON
			modevoice_pub.publish(modevoice);
			#endif
			
			// 次の動作まで3秒待つ(仮仕様)
			ros::Duration(3).sleep();

			getCoffee_status = 2;
			m_action = RULO_PLAN_STATE_MOVE_NEARCOFFEE;
			ioState = 2;
			//m_action = RULO_PLAN_STATE_MOVE_KNOWNGOAL;
			//m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
		}
		/***************
		* ARM GOALキャンセル時
		***************/
		else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
			ROS_INFO("ARM Control Canceled : (%s)", goal_state.toString().c_str());
		}
		/***************
		* ARM GOAL未到着時 (要因ごとの詳細はこれから実装）
		***************/
		else{
			ROS_INFO("ARM Control Failed : (%s)",goal_state.toString().c_str());
			modevoice.data = 11; 				// 011_fail音声を読み上げ	
			#ifdef MODEVOICE_ON
			modevoice_pub.publish(modevoice);	//
			#endif

			// 次の動作まで3秒待つ(仮仕様)
			ros::Duration(3).sleep();

            // 失敗時は一度バックする仕様
  			m_action = RULO_PLAN_STATE_MOVE_NEARCOFFEE;
    		getCoffee_status = 1;			

			ioState = 2;
		}
		return;
	}
    	
	/***************************************************************************
	* AutoDocking_client state監視関数
	***************************************************************************/
	void doneAutoDockingCb(const actionlib::SimpleClientGoalState& goal_state, const rulo7_msgs::AutoDockingResultConstPtr& result){
	ROS_INFO("doneAutoDockingCb");

		return;

	}


	/***************************************************************************
	* detect_coffee動作
	* コーヒー缶検出時に本Callback関数読み出し
	* コーヒー缶を検出した場合、MOVE_NEARCOFFEEステートへの移行準備
	***************************************************************************/
	void TargetObjectCallback(const geometry_msgs::PoseStamped &target_obj){ 

		//** [debug用] Manualモード時は何もしない **//
		if(m_state == RULO_PLAN_STATE_MANUAL || m_state == RULO_PLAN_STATE_STANDBY){
			return;
		}

		// 一旦、コーヒ缶との距離判別をせずに、MOVE_NEARCOFFEE に遷移する様に作成
		// 距離に応じて、MOVE_KNOWNGOALを使うかは要検討
		if(m_state == RULO_PLAN_STATE_MOVE_KNOWNGOAL && getCoffee_status == 0){
			m_action = RULO_PLAN_STATE_MOVE_NEARCOFFEE;
			ioState = 2;
		}
		
		// 一旦coffee座標をクラス内変数にコピー
		// MOVE_NEARCOFFEE や ARM_Controller に渡す用として仮置き(使わなければ消す）
		m_coffee_pose = target_obj;
		return;
	}

	/***************************************************************************
	* detect_docking_IR動作
	* rulo7_planner/docking_irchar情報(1秒に1回)を検知時に本Callback関数読み出し
	* コーヒ缶把持状態でIR受光を検出した場合、AUTO_DOCKING状態に遷移
	***************************************************************************/
	void IrCharCallback(const rulo7_msgs::DockingIRchar &irchar){ 

		//** [debug用] Manualモード時は何もしない **//
		if(m_state == RULO_PLAN_STATE_MANUAL || m_state == RULO_PLAN_STATE_STANDBY){
			return;
		}

		p_dockir = m_dockir;
		m_dockir = irchar;

		//  コーヒ缶把持状態でIR受光を検出した場合、AUTO_DOCKING状態に遷移（本来)
		//  今は缶把持状態は
		if(m_state == RULO_PLAN_STATE_MOVE_KNOWNGOAL && getCoffee_status == 2){
		//if(m_state == RULO_PLAN_STATE_MOVE_KNOWNGOAL){
			//if(((int)irchar.left_det+(int)irchar.center_det+(int)irchar.right_det+(int)irchar.omni_det) >= 1){ 
            if(((int)irchar.left_det+(int)irchar.center_det+(int)irchar.right_det) >= 1){ 
				//ROS_INFO("Detect Docking IRchar [%f,%f,%f,%f]",irchar.left_pos*irchar.left_det, irchar.center_pos*irchar.center_det, irchar.right_pos*irchar.right_det, irchar.omni_pos*irchar.omni_det);
				m_action = RULO_PLAN_STATE_AUTO_DOCKING;
				ioState = 2;
			}
		}
		if(m_state == RULO_PLAN_STATE_AUTO_DOCKING){
			//ROS_INFO("Detect Docking IRchar [%f,%f,%f,%f]",irchar.left_pos*irchar.left_det, irchar.center_pos*irchar.center_det, irchar.right_pos*irchar.right_det, irchar.omni_pos*irchar.omni_det);
		 }
		return;

		}
		
	/***************************************************************************
	* detect_battery_check動作
	* バッテリー状態を検知して本Callback関数読み出し
	* もしAUTO_DOCKING状態で充電検知した場合、IN_CHARGE状態へ遷移
	***************************************************************************/
	void BatteryCallback(const rulo_msgs::Battery &battery){ 

		//** [debug用] Manualモード時は何もしない **//
		if(m_state == RULO_PLAN_STATE_MANUAL){
			return;
		}

		// AUTO_DOCKING状態で充電検知した場合、IN_CHARGE状態へ遷移
		//  	/mobile_base/event/batteryのstatus情報
		// 0：充電台未接続
		// 1：初期温度チェック
		// 2：初期電流値チェック
		// 3：充電中
		// 4：充電完了
		// 5：温度待機中
		// 6：充電異常
		if(m_state == RULO_PLAN_STATE_AUTO_DOCKING){
			//hirose add(17/12/6)
			if(battery.status==1 || battery.status==2){
				docking_cmdvel_zero_flag = 1;
			}else{
				docking_cmdvel_zero_flag = 0;
			}
			//hirose add(17/12/6) End

			if(battery.status==3 || battery.status==4){
				m_action = RULO_PLAN_STATE_IN_CHARGE;
				ioState = 2;
				ROS_INFO("battery.status:%d",battery.status);
			}
		}
		
		return;

	}

	/***************************************************************************
	* amcl_pose取得動作
	* 現在位置のマップ情報を取得する関数読み出し
	***************************************************************************/
	void AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &amcl_pose){ 
		m_pose = amcl_pose;
		//ROS_INFO("[m_pose] %0.3f,%0.3f,%0.3f",m_pose.pose.pose.position.x,m_pose.pose.pose.position.y,m_pose.pose.pose.orientation.z);
		return;
	}

	/***************************************************************************
	* GlobalMap取得動作
	* 情報を取得する関数読み出し
	***************************************************************************/
	//void GlobalMapCallback(const nav_msgs::OccupancyGridConstPtr &grid){
	//	global_map = *grid;
	//
	//	ROS_INFO("[GlobalMap] %0.3f[m]x%0.3f[m]",global_map.info.width*global_map.info.resolution, global_map.info.height*global_map.info.resolution);
	//}

	/***************************************************************************
	* str_state関数
	* 状態情報をString型に変換する
	***************************************************************************/
	std_msgs::String StrState(RULO_PLAN_STATE state){
		std_msgs::String msg;
		if(state == RULO_PLAN_STATE_STANDBY) msg.data="STANDBY";
		else if(state == RULO_PLAN_STATE_MOVE_KNOWNGOAL)  msg.data="MOVE_KNOWNGOAL";
		else if(state == RULO_PLAN_STATE_MOVE_NEARCOFFEE) msg.data="MOVE_NEARCOFFEE";
		else if(state == RULO_PLAN_STATE_ARM_CONTROL)  msg.data="ARM_CONTROL";
		else if(state == RULO_PLAN_STATE_AUTO_DOCKING) msg.data="AUTO_DOCKING";
		else if(state == RULO_PLAN_STATE_IN_CHARGE)    msg.data="IN_CHARGE";
		else if(state == RULO_PLAN_STATE_MANUAL)       msg.data="MANUAL";

		return msg;
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
			local_goal_id = 0;
			ioState = 1;
			getCoffee_status = 0;
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
					getCoffee_status = 2;
					m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
				}else{
					 global_goal_id++;
				}
			}
			else{
				getCoffee_status = 0;
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
					getCoffee_status = 2;
					m_goal_target = RULO_PLAN_KNOWNGOAL_DOCKING;
				}else{
					 global_goal_id--;
				}
			}			
			else{
				getCoffee_status = 0;
				m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
				global_goal_id = ((sizeof(goal_ptn)/sizeof(Point))-1);
			}
			//global_goal_id = (global_goal_id == 0 ? ((sizeof(goal_ptn)/sizeof(Point))-1)  : global_goal_id-1); 
			ioState = 2;
		}
    	// 十字ボタン(上/下) : NEARCOFFEE時 缶保持有無の変更
		if((joy_msg.axes[JOY_AXES_CURSOR_VERTICAL] == 1.0 || joy_msg.axes[JOY_AXES_CURSOR_VERTICAL] == -1.0)&& m_state==RULO_PLAN_STATE_MOVE_NEARCOFFEE){
			getCoffee_status=(getCoffee_status+1)%3;
			ROS_INFO("change getCoffee_status :%d", getCoffee_status);
			ioState = 1;
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
				modevoice.data = 60 + p_state ;
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);	//
				#endif
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
				modevoice.data = 60 + p_state ;
				#ifdef MODEVOICE_ON
				modevoice_pub.publish(modevoice);	//
				#endif
			}
		}
    	
		// ×ボタン : ブラシ回転
		rulo_msgs::BrushesPWM_cmd brush;

		if(joy_msg.buttons[JOY_BUTTON_BRUSH] == 1){
			brush_on = !brush_on;
		}
		if(brush_on){
	    	brush.main_brush = 32;
			brush.side_brush = 32;
			brush.vacuum = 32;
		}else{
		    brush.main_brush = 0;
			brush.side_brush = 0;
			brush.vacuum = 0;
		}
		brush_pub.publish(brush);

		// △ボタン : 緊急停止解除モード解除
		if(joy_msg.buttons[JOY_BUTTON_NORMAL_MODE] == 1){
			rulomode.data = "normal";
			mode_pub.publish(rulomode);
		}

		// 〇ボタン : MOVEKNOWN状態での周回モード/蛇行モードの遷移
		if(joy_msg.buttons[JOY_BUTTON_MEANDER_MODE] == 1){
			if(m_state==RULO_PLAN_STATE_MOVE_KNOWNGOAL && m_goal_target == RULO_PLAN_KNOWNGOAL_AROUND){
				m_goal_target = RULO_PLAN_KNOWNGOAL_MEANDER;
				ROS_INFO("change RULO_PLAN_KNOWNGOAL_MEANDER");
				local_goal_id = 0;
				ioState = 2;
			}else if(m_state==RULO_PLAN_STATE_MOVE_KNOWNGOAL && m_goal_target == RULO_PLAN_KNOWNGOAL_MEANDER){
				m_goal_target = RULO_PLAN_KNOWNGOAL_AROUND;
				ROS_INFO("change RULO_PLAN_KNOWNGOAL_AROUND");
				local_goal_id = 0;
				ioState = 2;
			}
		}
		//	// /move_base/clear_costmaps サービスを発行
		//	if(clear_costmap_client.call(clear_costmap_req)){
		//		ROS_INFO("clear_costmap!");
		//	}else{
		//		ROS_INFO("failed calling clear_costmap...");
		//	}
		//}
		
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
		private_nh_("~"),
		m_moveBaseClient("move_base",true),
		m_nearCoffeeClient("Rulo7NearCoffee",true),
		m_armControlClient("pick_coffee",true),
		m_autoDockingClient("Rulo7AutoDocking",true)
	{
		actmove_timer   = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&OnigiriPlanner::ActState, this));
		initialpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
		modevoice_pub   = nh_.advertise<std_msgs::Char>("/rulo7_planner/mode_voice", 10);
		joy_sub         = nh_.subscribe("joy", 10, &OnigiriPlanner::JoyConCallback, this);
		tgtobj_sub      = nh_.subscribe("/target_object", 10, &OnigiriPlanner::TargetObjectCallback, this);
		docking_irchar_sub = nh_.subscribe("rulo7_planner/docking_irchar", 10, &OnigiriPlanner::IrCharCallback, this);
		docking_battery_sub = nh_.subscribe("/mobile_base/event/battery", 10, &OnigiriPlanner::BatteryCallback, this);
		amclpose_sub        = nh_.subscribe("/amcl_pose", 10, &OnigiriPlanner::AmclPoseCallback, this);
		//globalmap_sub       = nh_.subscribe("/move_base/global_costmap/costmap", 10, &OnigiriPlanner::GlobalMapCallback, this);

		clear_costmap_client = nh_.serviceClient<std_srvs::Empty>("move_base/clear_costmaps", this);
		meander_client   = nh_.serviceClient<rulo7_msgs::MakeMeanderPlan>("rulo7_planner/make_meander_plan", this);

    	//state&action initialize
		m_state =  RULO_PLAN_STATE_STANDBY;
		p_state =  RULO_PLAN_STATE_STANDBY;
		m_action = RULO_PLAN_STATE_STANDBY;
		ioState = 1;



		//debug(Manualモード用)
		twist_pub = nh_.advertise<geometry_msgs::Twist>("Rulo/cmd_vel", 10);
		mode_pub  = nh_.advertise<std_msgs::String>("mobile_base/command/mode", 10);
		brush_pub = nh_.advertise<rulo_msgs::BrushesPWM_cmd>("mobile_base/command/brushesPWM_cmd", 10);
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;
		twist_pub.publish(twist);
		brush_on=0;

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
	twist_pub = nh_.advertise<geometry_msgs::Twist>("Rulo/cmd_vel", 10);
	geometry_msgs::Twist twist;
	twist.linear.x = 0.0;
	twist.angular.z = 0.0;
	twist_pub.publish(twist);

	return 0;
}


#include "urn_jaus_jss_mobility_LocalWaypointDriverClient/LocalWaypointDriverClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_LocalWaypointDriverClient
{



LocalWaypointDriverClient_ReceiveFSM::LocalWaypointDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "LocalWaypointDriverClient", 10.0),
  logger(cmp->get_logger().get_child("LocalWaypointDriverClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalWaypointDriverClient_ReceiveFSMContext(*this);

	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_travel_speed = 1.0;
	p_tf_frame_robot = "base_link";
	p_wp_tolerance = 1.0;
	p_hz = 0.0;
}



LocalWaypointDriverClient_ReceiveFSM::~LocalWaypointDriverClient_ReceiveFSM()
{
	delete context;
}

void LocalWaypointDriverClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_LocalWaypointDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_LocalWaypointDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "LocalWaypointDriverClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "LocalWaypointDriverClient_ReceiveFSM");
}


void LocalWaypointDriverClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "LocalWaypointDriverClient");
	cfg.declare_param<double>("travel_speed", p_travel_speed, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Initial travel speed used if a waypoint is set. This value can be changed by cmd_speed topic.",
		"Default: 1.0");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id of the robot.",
		"Default: 'base_link'");
	// cfg.declare_param<double>("waypoint_tolerance", p_wp_tolerance, true,
	// 	rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
	// 	"currently not used.",
	// 	"Default: 1.0");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 0.0");
	cfg.param("travel_speed", p_travel_speed, p_travel_speed);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
//	cfg.param("waypoint_tolerance", p_wp_tolerance, p_wp_tolerance);
	cfg.param("hz", p_hz, p_hz, false);
	//create ROS subscriber
	// p_sub_path = cfg.create_subscription<nav_msgs::msg::Path>("cmd_path", 1, &LocalWaypointDriverClient_ReceiveFSM::pCmdPath, this);
	p_sub_pose = cfg.create_subscription<geometry_msgs::msg::PoseStamped>("cmd_local_pose", 1, std::bind(&LocalWaypointDriverClient_ReceiveFSM::pCmdPose, this, std::placeholders::_1));
	p_sub_speed = cfg.create_subscription<std_msgs::msg::Float32>("cmd_speed", 1, std::bind(&LocalWaypointDriverClient_ReceiveFSM::pCmdSpeed, this, std::placeholders::_1));
	p_pub_path = cfg.create_publisher<nav_msgs::msg::Path>("local_waypath", 5);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:mobility:LocalWaypointDriver", 1, 0);
	this->set_event_name("local waypoint");
	p_tf_buffer = std::make_unique<tf2_ros::Buffer>(cmp->get_clock());
	p_tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);
}

void LocalWaypointDriverClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_local_waypoint_msg, p_hz);
}

void LocalWaypointDriverClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_local_waypoint_msg);
	stop_query(remote_addr);
}

void LocalWaypointDriverClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_local_waypoint_msg, remote_addr);
}

void LocalWaypointDriverClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
}

void LocalWaypointDriverClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportLocalWaypoint report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportLocalWaypointAction(report, transport_data);
}


void LocalWaypointDriverClient_ReceiveFSM::handleReportLocalWaypointAction(ReportLocalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double x, y, z = 0.0;
	double roll, pitch, yaw = 0.0;
	ReportLocalWaypoint::Body::LocalWaypointRec *wprec = msg.getBody()->getLocalWaypointRec();
	x = wprec->getX();
	y = wprec->getY();
	if (wprec->isZValid()) {
		z = wprec->getZ();
	}
	if (wprec->isRollValid()) {
		roll = wprec->getRoll();
	}
	if (wprec->isPitchValid()) {
		pitch = wprec->getPitch();
	}
	if (wprec->isYawValid()) {
		yaw = wprec->getYaw();
	}
	if (wprec->isWaypointToleranceValid()) {
	}

	RCLCPP_DEBUG(logger, "currentWaypointAction from %s - x: %.2f, y: %.2f, z: %.2f", sender.str().c_str(), x, y, z);
	RCLCPP_DEBUG(logger, "    roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);

	auto path = nav_msgs::msg::Path();
	path.header.stamp = cmp->now();
	path.header.frame_id = this->p_tf_frame_robot;

	tf2::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);

	auto pose = geometry_msgs::msg::PoseStamped();
	pose.header = path.header;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
	pose.pose.orientation.w = quat.w();
	path.poses.push_back(pose);
	this->p_pub_path->publish(path);
}

void LocalWaypointDriverClient_ReceiveFSM::handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	RCLCPP_DEBUG(logger, "currentReportTravelSpeedAction from %s, speed: %.2f", sender.str().c_str(), p_travel_speed);
}

void LocalWaypointDriverClient_ReceiveFSM::pCmdPath(const nav_msgs::msg::Path::SharedPtr msg)
{
	if (p_has_access) {
		SetLocalWaypoint cmd;
		bool transformed = false;
		float speed = p_travel_speed;
		if (msg->poses.size() > 0) {
			try {
				auto pose_in = msg->poses[0];
				if (pose_in.header.frame_id.empty()) {
					pose_in.header = msg->header;
				}
				p_tf_buffer->lookupTransform(p_tf_frame_robot, pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration(0.3));
				auto pose_out = geometry_msgs::msg::PoseStamped();
				p_tf_buffer->transform(pose_in, pose_out, p_tf_frame_robot);
				cmd.getBody()->getLocalWaypointRec()->setX(pose_out.pose.position.x);
				cmd.getBody()->getLocalWaypointRec()->setY(pose_out.pose.position.y);
				cmd.getBody()->getLocalWaypointRec()->setZ(pose_out.pose.position.z);
				double roll, pitch, yaw;
				tf2::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
				tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				if (!isnan(yaw)) {
					cmd.getBody()->getLocalWaypointRec()->setRoll(roll);
					cmd.getBody()->getLocalWaypointRec()->setPitch(pitch);
					cmd.getBody()->getLocalWaypointRec()->setYaw(yaw);
				}
				transformed = true;
			} catch (tf2::TransformException &ex) {
				printf ("Failure %s\n", ex.what()); //Print exception which was caught
				speed = 0.0;
			}
		} else {
			speed = 0.0;
		}
		SetTravelSpeed cmd_speed;
		RCLCPP_INFO(logger, "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
		cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
		sendJausMessage(cmd_speed, p_remote_addr);
		if (transformed || msg->poses.size() == 0) {
			RCLCPP_INFO(logger, "send Waypoint from Path [x: %.2f, y: %.2f] to %s",
					cmd.getBody()->getLocalWaypointRec()->getX(), cmd.getBody()->getLocalWaypointRec()->getY(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		}
	}
}

void LocalWaypointDriverClient_ReceiveFSM::pCmdPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (p_has_access) {
		SetLocalWaypoint cmd;
		SetTravelSpeed cmd_speed;
		float speed = p_travel_speed;
		try {
			auto pose_in = *msg;
			auto pose_out = geometry_msgs::msg::PoseStamped();
			if (!pose_in.header.frame_id.empty() && !p_tf_frame_robot.empty()) {
				p_tf_buffer->lookupTransform(p_tf_frame_robot, pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration(0.3));
				p_tf_buffer->transform(pose_in, pose_out, p_tf_frame_robot);
			} else {
				pose_out = pose_in;
				if (p_tf_frame_robot.empty()) {
					RCLCPP_WARN(logger, "p_tf_frame_robot is empty, forward local pose without transform to %s", p_remote_addr.str().c_str());
				} else if (pose_in.header.frame_id.empty()) {
					RCLCPP_WARN(logger, "pose_in.header.frame_id is empty, forward local pose without transform to %s", p_remote_addr.str().c_str());
				}
			}
			cmd.getBody()->getLocalWaypointRec()->setX(pose_out.pose.position.x);
			cmd.getBody()->getLocalWaypointRec()->setY(pose_out.pose.position.y);
			cmd.getBody()->getLocalWaypointRec()->setZ(pose_out.pose.position.z);
			double roll, pitch, yaw;
			tf2::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
			tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				cmd.getBody()->getLocalWaypointRec()->setRoll(roll);
				cmd.getBody()->getLocalWaypointRec()->setPitch(pitch);
				cmd.getBody()->getLocalWaypointRec()->setYaw(yaw);
			}
			RCLCPP_INFO(logger, "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
			RCLCPP_INFO(logger, "send Waypoint from Pose [x: %.2f, y: %.2f] to %s",
					cmd.getBody()->getLocalWaypointRec()->getX(), cmd.getBody()->getLocalWaypointRec()->getY(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		} catch (tf2::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
			speed = 0.0;
			RCLCPP_INFO(logger, "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
		}
	}
}

void LocalWaypointDriverClient_ReceiveFSM::pCmdSpeed(const std_msgs::msg::Float32::SharedPtr msg)
{
	if (p_has_access) {
		p_travel_speed = msg->data;
		SetTravelSpeed cmd;
		RCLCPP_DEBUG(logger, "set speed to %.2f on %s", p_travel_speed, p_remote_addr.str().c_str());
		cmd.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
		sendJausMessage(cmd, p_remote_addr);
	}
}




}

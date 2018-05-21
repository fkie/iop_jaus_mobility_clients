
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>
#include "urn_jaus_jss_mobility_LocalWaypointDriverClient/LocalWaypointDriverClient_ReceiveFSM.h"

#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_LocalWaypointDriverClient
{



LocalWaypointDriverClient_ReceiveFSM::LocalWaypointDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalWaypointDriverClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	p_travel_speed = 1.0;
	p_tf_frame_robot = "base_link";
	p_wp_tolerance = 1.0;
	p_has_access = false;
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
	iop::Config cfg("~LocalWaypointDriverClient");
	cfg.param("travel_speed", p_travel_speed, p_travel_speed);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
//	cfg.param("waypoint_tolerance", p_wp_tolerance, p_wp_tolerance);
	cfg.param("hz", p_hz, p_hz, false, false);
	//ROS_INFO_NAMED("LocalWaypointDriverClient", "  waypoint_tolerance: %.2f", p_wp_tolerance);
	//create ROS subscriber
	// p_sub_path = cfg.subscribe<nav_msgs::Path>("cmd_path", 1, &LocalWaypointDriverClient_ReceiveFSM::pCmdPath, this);
	p_sub_pose = cfg.subscribe<geometry_msgs::PoseStamped>("cmd_local_pose", 1, &LocalWaypointDriverClient_ReceiveFSM::pCmdPose, this);
	p_sub_speed = cfg.subscribe<std_msgs::Float32>("cmd_speed", 1, &LocalWaypointDriverClient_ReceiveFSM::pCmdSpeed, this);
	p_pub_path = cfg.advertise<nav_msgs::Path>("local_waypath", 5, true);
	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:LocalWaypointDriver", 1, 0);
}

void LocalWaypointDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:LocalWaypointDriver") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[LocalWaypointDriverClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void LocalWaypointDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void LocalWaypointDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void LocalWaypointDriverClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("LocalWaypointDriverClient", "create QUERY timer to get local waypoint from %s with %.2fHz", component.str().c_str(), p_hz);
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &LocalWaypointDriverClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("LocalWaypointDriverClient", "invalid hz %.2f for QUERY timer to get local waypoint from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("LocalWaypointDriverClient", "create EVENT to get local waypoint from %s with %.2fHz", component.str().c_str(), p_hz);
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_local_waypoint_msg, p_hz);
		sendJausMessage(p_query_local_waypoint_msg, component);
	}
}

void LocalWaypointDriverClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("LocalWaypointDriverClient", "cancel EVENT for local waypoint by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_local_waypoint_msg);
	}
}

void LocalWaypointDriverClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_local_waypoint_msg, p_remote_addr);
	}
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

	ROS_DEBUG_NAMED("LocalWaypointDriverClient", "currentWaypointAction from %s - x: %.2f, y: %.2f, z: %.2f", sender.str().c_str(), x, y, z);
	ROS_DEBUG_NAMED("LocalWaypointDriverClient", "    roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);

	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_robot;

	tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

	geometry_msgs::PoseStamped pose;
	pose.header = path.header;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
	pose.pose.orientation.w = quat.w();
	path.poses.push_back(pose);
	this->p_pub_path.publish(path);
}

void LocalWaypointDriverClient_ReceiveFSM::handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	ROS_DEBUG_NAMED("LocalWaypointDriverClient", "currentReportTravelSpeedAction from %s, speed: %.2f", sender.str().c_str(), p_travel_speed);
}

void LocalWaypointDriverClient_ReceiveFSM::pCmdPath(const nav_msgs::Path::ConstPtr& msg)
{
	if (p_has_access) {
		SetLocalWaypoint cmd;
		geometry_msgs::PointStamped point_out;
		bool transformed = false;
		float speed = p_travel_speed;
		if (msg->poses.size() > 0) {
			try {
				geometry_msgs::PoseStamped pose_in = msg->poses[0];
				if (pose_in.header.frame_id.empty()) {
					pose_in.header = msg->header;
				}
				tfListener.waitForTransform(p_tf_frame_robot, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
				geometry_msgs::PoseStamped pose_out;
				tfListener.transformPose(p_tf_frame_robot, pose_in, pose_out);
				cmd.getBody()->getLocalWaypointRec()->setX(pose_out.pose.position.x);
				cmd.getBody()->getLocalWaypointRec()->setY(pose_out.pose.position.y);
				cmd.getBody()->getLocalWaypointRec()->setZ(pose_out.pose.position.z);
				double roll, pitch, yaw;
				tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				if (!isnan(yaw)) {
					cmd.getBody()->getLocalWaypointRec()->setRoll(roll);
					cmd.getBody()->getLocalWaypointRec()->setPitch(pitch);
					cmd.getBody()->getLocalWaypointRec()->setYaw(yaw);
				}
				transformed = true;
			} catch (tf::TransformException &ex) {
				printf ("Failure %s\n", ex.what()); //Print exception which was caught
				speed = 0.0;
			}
		} else {
			speed = 0.0;
		}
		SetTravelSpeed cmd_speed;
		ROS_INFO_NAMED("LocalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
		cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
		sendJausMessage(cmd_speed, p_remote_addr);
		if (transformed || msg->poses.size() == 0) {
			ROS_INFO_NAMED("LocalWaypointDriverClient", "send Waypoint from Path [x: %.2f, y: %.2f] to %s",
					cmd.getBody()->getLocalWaypointRec()->getX(), cmd.getBody()->getLocalWaypointRec()->getY(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		}
	}
}

void LocalWaypointDriverClient_ReceiveFSM::pCmdPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (p_has_access) {
		SetLocalWaypoint cmd;
		geometry_msgs::PointStamped point_out;
		SetTravelSpeed cmd_speed;
		float speed = p_travel_speed;
		try {
			geometry_msgs::PoseStamped pose_in = *msg;
			geometry_msgs::PoseStamped pose_out;
			if (!pose_in.header.frame_id.empty() && !p_tf_frame_robot.empty()) {
				tfListener.waitForTransform(p_tf_frame_robot, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
				tfListener.transformPose(p_tf_frame_robot, pose_in, pose_out);
			} else {
				pose_out = pose_in;
				if (p_tf_frame_robot.empty()) {
					ROS_WARN_NAMED("LocalWaypointDriverClient", "p_tf_frame_robot is empty, forward local pose without transform to %s", p_remote_addr.str().c_str());
				} else if (pose_in.header.frame_id.empty()) {
					ROS_WARN_NAMED("LocalWaypointDriverClient", "pose_in.header.frame_id is empty, forward local pose without transform to %s", p_remote_addr.str().c_str());
				}
			}
			cmd.getBody()->getLocalWaypointRec()->setX(pose_out.pose.position.x);
			cmd.getBody()->getLocalWaypointRec()->setY(pose_out.pose.position.x);
			cmd.getBody()->getLocalWaypointRec()->setZ(pose_out.pose.position.z);
			double roll, pitch, yaw;
			tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				cmd.getBody()->getLocalWaypointRec()->setRoll(roll);
				cmd.getBody()->getLocalWaypointRec()->setPitch(pitch);
				cmd.getBody()->getLocalWaypointRec()->setYaw(yaw);
			}
			ROS_INFO_NAMED("LocalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
			ROS_INFO_NAMED("LocalWaypointDriverClient", "send Waypoint from Pose [x: %.2f, y: %.2f] to %s",
					cmd.getBody()->getLocalWaypointRec()->getX(), cmd.getBody()->getLocalWaypointRec()->getY(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		} catch (tf::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
			speed = 0.0;
			ROS_INFO_NAMED("LocalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
		}
	}
}

void LocalWaypointDriverClient_ReceiveFSM::pCmdSpeed(const std_msgs::Float32::ConstPtr& msg)
{
	if (p_has_access) {
		p_travel_speed = msg->data;
		SetTravelSpeed cmd;
		ROS_DEBUG_NAMED("LocalWaypointDriverClient", "set speed to %.2f on %s", p_travel_speed, p_remote_addr.str().c_str());
		cmd.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
		sendJausMessage(cmd, p_remote_addr);
	}
}




};

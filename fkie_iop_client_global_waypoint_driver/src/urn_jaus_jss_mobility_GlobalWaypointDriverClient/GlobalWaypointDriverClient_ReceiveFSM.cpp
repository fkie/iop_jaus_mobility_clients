
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>
#include "urn_jaus_jss_mobility_GlobalWaypointDriverClient/GlobalWaypointDriverClient_ReceiveFSM.h"

#include <gps_common/conversions.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_GlobalWaypointDriverClient
{



GlobalWaypointDriverClient_ReceiveFSM::GlobalWaypointDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalWaypointDriverClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	p_travel_speed = 1.0;
	p_tf_frame_world = "/world";
	p_utm_zone = "32U";
	p_wp_tolerance = 1.0;
	p_has_access = false;
	p_hz = 0.0;
}



GlobalWaypointDriverClient_ReceiveFSM::~GlobalWaypointDriverClient_ReceiveFSM()
{
	delete context;
}

void GlobalWaypointDriverClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalWaypointDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "GlobalWaypointDriverClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "GlobalWaypointDriverClient_ReceiveFSM");
	iop::Config cfg("~GlobalWaypointDriverClient");
	cfg.param("travel_speed", p_travel_speed, p_travel_speed);
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("utm_zone", p_utm_zone, p_utm_zone);
	cfg.param("waypoint_tolerance", p_wp_tolerance, p_wp_tolerance);
	cfg.param("hz", p_hz, p_hz, false, false);
	//ROS_INFO_NAMED("GlobalWaypointDriverClient", "  waypoint_tolerance: %.2f", p_wp_tolerance);
	//create ROS subscriber
	// p_sub_path = cfg.subscribe<nav_msgs::Path>("cmd_path", 1, &GlobalWaypointDriverClient_ReceiveFSM::pCmdPath, this);
	p_sub_pose = cfg.subscribe<geometry_msgs::PoseStamped>("cmd_pose", 1, &GlobalWaypointDriverClient_ReceiveFSM::pCmdPose, this);
	p_sub_fix = cfg.subscribe<sensor_msgs::NavSatFix>("cmd_fix", 1, &GlobalWaypointDriverClient_ReceiveFSM::pCmdFix, this);
	p_sub_speed = cfg.subscribe<std_msgs::Float32>("cmd_speed", 1, &GlobalWaypointDriverClient_ReceiveFSM::pCmdSpeed, this);
	p_pub_path = cfg.advertise<nav_msgs::Path>("global_waypoint", 5, true);
	p_sub_geopose = cfg.subscribe<geographic_msgs::GeoPoseStamped>("cmd_geopose", 1, &GlobalWaypointDriverClient_ReceiveFSM::pCmdGeoPose, this);
	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:GlobalWaypointDriver", 1, 0);
}

void GlobalWaypointDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:GlobalWaypointDriver") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[GlobalWaypointDriverClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void GlobalWaypointDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void GlobalWaypointDriverClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "create QUERY timer to get global waypoint from %s with %.2fHz", component.str().c_str(), p_hz);
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &GlobalWaypointDriverClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("GlobalWaypointDriverClient", "invalid hz %.2f for QUERY timer to get global waypoint from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("GlobalWaypointDriverClient", "create EVENT to get global waypoint from %s with %.2fHz", component.str().c_str(), p_hz);
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_global_waypoint_msg, p_hz);
		sendJausMessage(p_query_global_waypoint_msg, component);
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("GlobalWaypointDriverClient", "cancel EVENT for global waypoint by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_global_waypoint_msg);
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_global_waypoint_msg, p_remote_addr);
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportGlobalWaypoint report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportGlobalWaypointAction(report, transport_data);
}


void GlobalWaypointDriverClient_ReceiveFSM::handleReportGlobalWaypointAction(ReportGlobalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double lat, lon, alt = 0.0;
	double roll, pitch, yaw = 0.0;
	ReportGlobalWaypoint::Body::GlobalWaypointRec *wprec = msg.getBody()->getGlobalWaypointRec();
	lat = wprec->getLatitude();
	lon = wprec->getLongitude();
	if (wprec->isAltitudeValid()) {
		alt = wprec->getAltitude();
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

	ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "globalWaypointAction from %s - lat: %.2f, lon: %.2f", sender.str().c_str(), lat, lon);
	ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "    alt: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", alt, roll, pitch, yaw);

	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_world;

	if (lat > -90.0 && lon > -180.0 && lat < 90.0 && lon < 180.0) {
		double northing, easting;
		std::string zone;
		gps_common::LLtoUTM(lat, lon, northing, easting, zone);
		tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

		geometry_msgs::PoseStamped pose;
		pose.header = path.header;
		pose.pose.position.x = easting;
		pose.pose.position.y = northing;
		pose.pose.position.z = alt;
		pose.pose.orientation.x = quat.x();
		pose.pose.orientation.y = quat.y();
		pose.pose.orientation.z = quat.z();
		pose.pose.orientation.w = quat.w();
		path.poses.push_back(pose);
	}
	this->p_pub_path.publish(path);
}

void GlobalWaypointDriverClient_ReceiveFSM::handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "currentReportTravelSpeedAction from %s, speed: %.2f", sender.str().c_str(), p_travel_speed);
}


void GlobalWaypointDriverClient_ReceiveFSM::pCmdPath(const nav_msgs::Path::ConstPtr& msg)
{
	if (p_has_access) {
		SetGlobalWaypoint cmd;
		geometry_msgs::PointStamped point_out;
		bool transformed = false;
		float speed = p_travel_speed;
		if (msg->poses.size() > 0) {
			try {
				geometry_msgs::PoseStamped pose_in = msg->poses[0];
				if (pose_in.header.frame_id.empty()) {
					pose_in.header = msg->header;
				}
				tfListener.waitForTransform(p_tf_frame_world, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
				geometry_msgs::PoseStamped pose_out;
				tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
				double lat, lon = 0;
				gps_common::UTMtoLL(pose_out.pose.position.y, pose_out.pose.position.x, p_utm_zone.c_str(), lat, lon);
				cmd.getBody()->getGlobalWaypointRec()->setLatitude(lat);
				cmd.getBody()->getGlobalWaypointRec()->setLongitude(lon);
				cmd.getBody()->getGlobalWaypointRec()->setAltitude(pose_out.pose.position.z);
				double roll, pitch, yaw;
				tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				if (!isnan(yaw)) {
					cmd.getBody()->getGlobalWaypointRec()->setRoll(roll);
					cmd.getBody()->getGlobalWaypointRec()->setPitch(pitch);
					cmd.getBody()->getGlobalWaypointRec()->setYaw(yaw);
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
		ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
		cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
		sendJausMessage(cmd_speed, p_remote_addr);
		if (transformed || msg->poses.size() == 0) {
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "send Waypoint from Path [lat: %.2f, lon: %.2f] to %s",
					cmd.getBody()->getGlobalWaypointRec()->getLatitude(), cmd.getBody()->getGlobalWaypointRec()->getLongitude(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		}
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pCmdPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (p_has_access) {
		SetGlobalWaypoint cmd;
		geometry_msgs::PointStamped point_out;
		SetTravelSpeed cmd_speed;
		float speed = p_travel_speed;
		try {
			geometry_msgs::PoseStamped pose_in = *msg;
			tfListener.waitForTransform(p_tf_frame_world, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
			geometry_msgs::PoseStamped pose_out;
			tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
			double lat, lon = 0;
			gps_common::UTMtoLL(pose_out.pose.position.y, pose_out.pose.position.x, p_utm_zone.c_str(), lat, lon);
			cmd.getBody()->getGlobalWaypointRec()->setLatitude(lat);
			cmd.getBody()->getGlobalWaypointRec()->setLongitude(lon);
			cmd.getBody()->getGlobalWaypointRec()->setAltitude(pose_out.pose.position.z);
			double roll, pitch, yaw;
			tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				cmd.getBody()->getGlobalWaypointRec()->setRoll(roll);
				cmd.getBody()->getGlobalWaypointRec()->setPitch(pitch);
				cmd.getBody()->getGlobalWaypointRec()->setYaw(yaw);
			}
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "send Waypoint from Pose [lat: %.2f, lon: %.2f] to %s",
					cmd.getBody()->getGlobalWaypointRec()->getLatitude(), cmd.getBody()->getGlobalWaypointRec()->getLongitude(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		} catch (tf::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
			speed = 0.0;
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
		}
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pCmdFix(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	if (p_has_access) {
		SetGlobalWaypoint cmd;
		SetTravelSpeed cmd_speed;
		float speed = p_travel_speed;
		try {
			cmd.getBody()->getGlobalWaypointRec()->setLatitude(msg->latitude);
			cmd.getBody()->getGlobalWaypointRec()->setLongitude(msg->longitude);
			cmd.getBody()->getGlobalWaypointRec()->setAltitude(msg->altitude);
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "send Waypoint from NavSatFix [lat: %.2f, lon: %.2f] to %s",
					cmd.getBody()->getGlobalWaypointRec()->getLatitude(), cmd.getBody()->getGlobalWaypointRec()->getLongitude(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		} catch (tf::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
			speed = 0.0;
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
		}
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pCmdGeoPose(const geographic_msgs::GeoPoseStamped::ConstPtr& msg)
{
	if (p_has_access) {
		SetGlobalWaypoint cmd;
		SetTravelSpeed cmd_speed;
		float speed = p_travel_speed;
		try {
			cmd.getBody()->getGlobalWaypointRec()->setLatitude(msg->pose.position.latitude);
			cmd.getBody()->getGlobalWaypointRec()->setLongitude(msg->pose.position.longitude);
			cmd.getBody()->getGlobalWaypointRec()->setAltitude(msg->pose.position.altitude);
			double roll, pitch, yaw;
			tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				cmd.getBody()->getGlobalWaypointRec()->setRoll(roll);
				cmd.getBody()->getGlobalWaypointRec()->setPitch(pitch);
				cmd.getBody()->getGlobalWaypointRec()->setYaw(yaw);
			}
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "send Waypoint from Pose [lat: %.2f, lon: %.2f] to %s",
					cmd.getBody()->getGlobalWaypointRec()->getLatitude(), cmd.getBody()->getGlobalWaypointRec()->getLongitude(),
					p_remote_addr.str().c_str());
			sendJausMessage(cmd, p_remote_addr);
		} catch (tf::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
			speed = 0.0;
			ROS_INFO_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", speed, p_remote_addr.str().c_str());
			cmd_speed.getBody()->getTravelSpeedRec()->setSpeed(speed);
			sendJausMessage(cmd_speed, p_remote_addr);
		}
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pCmdSpeed(const std_msgs::Float32::ConstPtr& msg)
{
	if (p_has_access) {
		p_travel_speed = msg->data;
		SetTravelSpeed cmd;
		ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "set speed to %.2f on %s", p_travel_speed, p_remote_addr.str().c_str());
		cmd.getBody()->getTravelSpeedRec()->setSpeed(p_travel_speed);
		sendJausMessage(cmd, p_remote_addr);
	}
}




};

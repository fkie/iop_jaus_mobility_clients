/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#include "urn_jaus_jss_mobility_GlobalPoseSensorClient/GlobalPoseSensorClient_ReceiveFSM.h"
#include <fkie_iop_component/gps_conversions.h>
#include <fkie_iop_component/iop_config.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_GlobalPoseSensorClient
{



GlobalPoseSensorClient_ReceiveFSM::GlobalPoseSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "GlobalPoseSensorClient", 10.0),
  logger(cmp->get_logger().get_child("GlobalPoseSensorClient")),
  p_tf_broadcaster(cmp)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalPoseSensorClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_world = "world";
	p_tf_frame_anchor = "anchor";
	p_tf_frame_robot = "base_link";
	p_anchor_northing = 0.0;
	p_anchor_easting = 0.0;
	p_anchor_altitude = 0.0;
	p_publish_world_anchor = true;
	p_query_global_pose_msg.getBody()->getQueryGlobalPoseRec()->setPresenceVector(65535);
	p_hz = 10.0;
}



GlobalPoseSensorClient_ReceiveFSM::~GlobalPoseSensorClient_ReceiveFSM()
{

	delete context;
}

void GlobalPoseSensorClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalPoseSensorClient_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalPoseSensorClient_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "GlobalPoseSensorClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "GlobalPoseSensorClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "GlobalPoseSensorClient_ReceiveFSM");
}


void GlobalPoseSensorClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "GlobalPoseSensorClient");
	cfg.declare_param<std::string>("tf_frame_world", p_tf_frame_world, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id used in ROS for global coordinates.",
		"Default: 'world'");
	cfg.declare_param<std::string>("tf_frame_anchor", p_tf_frame_anchor, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id for achor between world and robot frame. It is usefull for visualization in RViz.",
		"Default: 'anchor'");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id of the robot.",
		"Default: 'base_link'");
	cfg.declare_param<double>("anchor_easting", p_anchor_easting, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Default easting coordinate for the anchor. It can be replaced by 'fix_anchor' topic.",
		"Default: 0.0");
	cfg.declare_param<double>("anchor_northing", p_anchor_northing, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Default northing coordinate for the anchor. It can be replaced by 'fix_anchor' topic.",
		"Default: 0.0");
	cfg.declare_param<double>("anchor_altitude", p_anchor_altitude, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Default altitude coordinate for the anchor. It can be replaced by 'fix_anchor' topic.",
		"Default: 0.0");
	cfg.declare_param<bool>("publish_world_anchor", p_publish_world_anchor, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Enables the publishing of 'world' to 'anchor' tf. You can disable the publishing of this tf by this service and publish tf itself.",
		"Default: true");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 10.0");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tf_frame_anchor", p_tf_frame_anchor, p_tf_frame_anchor);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("anchor_easting", p_anchor_easting, p_anchor_easting);
	cfg.param("anchor_northing", p_anchor_northing, p_anchor_northing);
	cfg.param("anchor_altitude", p_anchor_altitude, p_anchor_altitude);
	cfg.param("publish_world_anchor", p_publish_world_anchor, p_publish_world_anchor);
	cfg.param("hz", p_hz, p_hz, false);
	p_pub_navsatfix = cfg.create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
	p_pub_imu = cfg.create_publisher<sensor_msgs::msg::Imu>("imu", 1);
	p_pub_pose = cfg.create_publisher<geometry_msgs::msg::PoseStamped>("global_pose", 5);
	p_sub_anchorfix = cfg.create_subscription<sensor_msgs::msg::NavSatFix>("fix_anchor", 1, std::bind(&GlobalPoseSensorClient_ReceiveFSM::anchorFixReceived, this, std::placeholders::_1));
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:mobility:GlobalPoseSensor", 1, 0);
	this->set_event_name("global pose");
	this->set_query_before_event(true, 1.0);
	p_tf_anchor.transform.translation.x = p_anchor_easting;
	p_tf_anchor.transform.translation.y = p_anchor_northing;
	p_tf_anchor.transform.translation.z = p_anchor_altitude;
	p_tf_anchor.transform.rotation.x = 0;
	p_tf_anchor.transform.rotation.y = 0;
	p_tf_anchor.transform.rotation.z = 0;
	p_tf_anchor.transform.rotation.w = 1.0;
	p_tf_anchor.header.stamp = cmp->now();
	p_tf_anchor.header.frame_id = this->p_tf_frame_world;
	p_tf_anchor.child_frame_id = this->p_tf_frame_anchor;

	RCLCPP_DEBUG(logger, "update anchor tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_anchor.c_str());
}

void GlobalPoseSensorClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_global_pose_msg, p_hz);
}

void GlobalPoseSensorClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_global_pose_msg);
	stop_query(remote_addr);
}

void GlobalPoseSensorClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_global_pose_msg, remote_addr);
}

void GlobalPoseSensorClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
}

void GlobalPoseSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportGlobalPose report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportGlobalPoseAction(report, transport_data);
}

void GlobalPoseSensorClient_ReceiveFSM::handleReportGeomagneticPropertyAction(ReportGeomagneticProperty msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void GlobalPoseSensorClient_ReceiveFSM::handleReportGlobalPoseAction(ReportGlobalPose msg, Receive::Body::ReceiveRec transportData)
{
	auto fix = sensor_msgs::msg::NavSatFix();
	fix.latitude = msg.getBody()->getGlobalPoseRec()->getLatitude();
	fix.longitude = msg.getBody()->getGlobalPoseRec()->getLongitude();
	if (msg.getBody()->getGlobalPoseRec()->isAltitudeValid()) {
		fix.altitude = msg.getBody()->getGlobalPoseRec()->getAltitude();
	}
	fix.status.status = 1;
	if (msg.getBody()->getGlobalPoseRec()->isTimeStampValid()) {
		// get timestamp
		ReportGlobalPose::Body::GlobalPoseRec::TimeStamp *ts = msg.getBody()->getGlobalPoseRec()->getTimeStamp();
		iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		fix.header.stamp = stamp.ros_time;
	} else {
		fix.header.stamp = cmp->now();
	}

	p_pub_navsatfix->publish(fix);
	auto imu = sensor_msgs::msg::Imu();
	tf2::Quaternion quat;
	if (msg.getBody()->getGlobalPoseRec()->isYawValid()) {
		RCLCPP_DEBUG(logger, "quat from: %.2f, %.2f, %.2f", msg.getBody()->getGlobalPoseRec()->getRoll(), msg.getBody()->getGlobalPoseRec()->getPitch(), msg.getBody()->getGlobalPoseRec()->getYaw());
		quat.setRPY(msg.getBody()->getGlobalPoseRec()->getRoll(), msg.getBody()->getGlobalPoseRec()->getPitch(), msg.getBody()->getGlobalPoseRec()->getYaw());
		imu.orientation.x = quat.x();
		imu.orientation.y = quat.y();
		imu.orientation.z = quat.z();
		imu.orientation.w = quat.w();
		p_pub_imu->publish(imu);
	} else {
		quat.setRPY(0, 0, 0);
	}

	auto transform = geometry_msgs::msg::TransformStamped();
	double northing, easting;
	std::string zone;
	gps_common::LLtoUTM(fix.latitude, fix.longitude, northing, easting, zone);
	if (p_anchor_easting == 0 && p_anchor_northing == 0) {
		p_anchor_easting = easting;
		p_anchor_northing = northing;
		p_anchor_altitude = fix.altitude;
		p_tf_anchor.transform.translation.x = p_anchor_easting;
		p_tf_anchor.transform.translation.y = p_anchor_northing;
		p_tf_anchor.transform.translation.z = p_anchor_altitude;
		tf2::Quaternion q;
		q.setRPY(0, 0, 0);
		p_tf_anchor.transform.rotation.x = q.x();
		p_tf_anchor.transform.rotation.y = q.y();
		p_tf_anchor.transform.rotation.z = q.z();
		p_tf_anchor.transform.rotation.w = q.w();
		p_tf_anchor.header.stamp = cmp->now();
		p_tf_anchor.header.frame_id = this->p_tf_frame_world;
		p_tf_anchor.child_frame_id = this->p_tf_frame_anchor;
	}
	transform.transform.translation.x = easting - p_anchor_easting;
	transform.transform.translation.y = northing - p_anchor_northing;
	transform.transform.translation.z = fix.altitude - p_anchor_altitude;
	transform.transform.rotation.x = quat.x();
	transform.transform.rotation.y = quat.y();
	transform.transform.rotation.z = quat.z();
	transform.transform.rotation.w = quat.w();
	transform.header.stamp = cmp->now();
	transform.header.frame_id = this->p_tf_frame_anchor;
	transform.child_frame_id = this->p_tf_frame_robot;
	p_tf_anchor.header.stamp = transform.header.stamp;
	if (! transform.child_frame_id.empty()) {
		if (p_publish_world_anchor) {
			RCLCPP_DEBUG(logger, "update anchor tf %s -> %s, stamp: %d.%d", this->p_tf_frame_world.c_str(), this->p_tf_frame_anchor.c_str(), p_tf_anchor.header.stamp.sec, p_tf_anchor.header.stamp.nanosec);
			p_tf_broadcaster.sendTransform(p_tf_anchor);
		}
		RCLCPP_DEBUG(logger, "tf %s -> %s, stamp: %d.%d", this->p_tf_frame_anchor.c_str(), this->p_tf_frame_robot.c_str(), transform.header.stamp.sec, transform.header.stamp.nanosec);
		p_tf_broadcaster.sendTransform(transform);
	}
	// publish global pose
	auto pose = geometry_msgs::msg::PoseStamped();
	pose.pose.position.x = easting;
	pose.pose.position.y = northing;
	pose.pose.position.z = fix.altitude;
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
	pose.pose.orientation.w = quat.w();
	pose.header.stamp = transform.header.stamp;
	pose.header.frame_id = zone;
	p_pub_pose->publish(pose);
}

void GlobalPoseSensorClient_ReceiveFSM::anchorFixReceived(const sensor_msgs::msg::NavSatFix::SharedPtr fix)
{
	if (fix->status.status != -1) {
		std::string zone;
		if (!std::isnan(fix->altitude)) {
			p_anchor_altitude = fix->altitude;
		}
		gps_common::LLtoUTM(fix->latitude, fix->longitude, p_anchor_northing, p_anchor_easting, zone);
		p_tf_anchor.transform.translation.x = p_anchor_easting;
		p_tf_anchor.transform.translation.y = p_anchor_northing;
		p_tf_anchor.transform.translation.z = p_anchor_altitude;
		tf2::Quaternion q;
		q.setRPY(0, 0, 0);
		p_tf_anchor.transform.rotation.x = q.x();
		p_tf_anchor.transform.rotation.y = q.y();
		p_tf_anchor.transform.rotation.z = q.z();
		p_tf_anchor.transform.rotation.w = q.w();
		p_tf_anchor.header.stamp = cmp->now();
		p_tf_anchor.header.frame_id = this->p_tf_frame_world;
		p_tf_anchor.child_frame_id = this->p_tf_frame_anchor;
		RCLCPP_DEBUG(logger, "update anchor tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_anchor.c_str());
	}
}



}

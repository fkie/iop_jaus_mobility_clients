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

#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>
#include <fkie_iop_builder/timestamp.h>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_GlobalPoseSensorClient
{



GlobalPoseSensorClient_ReceiveFSM::GlobalPoseSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalPoseSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_tf_frame_world = "world";
	p_tf_frame_anchor = "anchor";
	p_tf_frame_robot = "base_link";
	p_anchor_northing = 0.0;
	p_anchor_easting = 0.0;
	p_anchor_altitude = 0.0;
	p_publish_world_anchor = true;
	p_query_global_pose_msg.getBody()->getQueryGlobalPoseRec()->setPresenceVector(65535);
	p_has_access = false;
	p_hz = 10.0;
}



GlobalPoseSensorClient_ReceiveFSM::~GlobalPoseSensorClient_ReceiveFSM()
{

	if (p_query_timer.isValid()) {
		p_query_timer.stop();
	}
	if (p_anchor_timer.isValid()) {
		p_anchor_timer.stop();
	}
	delete context;
}

void GlobalPoseSensorClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalPoseSensorClient_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalPoseSensorClient_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "GlobalPoseSensorClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "GlobalPoseSensorClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "GlobalPoseSensorClient_ReceiveFSM");
	iop::Config cfg("~GlobalPoseSensorClient");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tf_frame_anchor", p_tf_frame_anchor, p_tf_frame_anchor);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("anchor_easting", p_anchor_easting, p_anchor_easting);
	cfg.param("anchor_northing", p_anchor_northing, p_anchor_northing);
	cfg.param("anchor_altitude", p_anchor_altitude, p_anchor_altitude);
	cfg.param("publish_world_anchor", p_publish_world_anchor, p_publish_world_anchor);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_navsatfix = cfg.advertise<sensor_msgs::NavSatFix>("fix", 1, true);
	p_pub_imu = cfg.advertise<sensor_msgs::Imu>("imu", 1, true);
	p_pub_pose = cfg.advertise<geometry_msgs::PoseStamped>("global_pose", 5, true);
	p_sub_anchorfix = cfg.subscribe<sensor_msgs::NavSatFix>("fix_anchor", 1, &GlobalPoseSensorClient_ReceiveFSM::anchorFixReceived, this);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:GlobalPoseSensor", 1, 0);
	p_tf_anchor.transform.translation.x = p_anchor_easting;
	p_tf_anchor.transform.translation.y = p_anchor_northing;
	p_tf_anchor.transform.translation.z = p_anchor_altitude;
	p_tf_anchor.transform.rotation.x = 0;
	p_tf_anchor.transform.rotation.y = 0;
	p_tf_anchor.transform.rotation.z = 0;
	p_tf_anchor.transform.rotation.w = 1.0;
	p_tf_anchor.header.stamp = ros::Time::now();
	p_tf_anchor.header.frame_id = this->p_tf_frame_world;
	p_tf_anchor.child_frame_id = this->p_tf_frame_anchor;
	p_tf_anchor_robot.transform.rotation.w = 1.0;
	p_tf_anchor_robot.header.stamp = ros::Time::now();
	p_tf_anchor_robot.header.frame_id = this->p_tf_frame_anchor;
	p_tf_anchor_robot.child_frame_id = this->p_tf_frame_robot;
	p_tf_anchor_robot_ground = p_tf_anchor_robot;
	p_tf_anchor_robot_ground.child_frame_id = this->p_tf_frame_robot + "_ground";

	ROS_DEBUG_NAMED("GlobalPoseSensorClient", "update anchor tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_anchor.c_str());
	if (p_publish_world_anchor) {
		// publish anchor->robot tf all time if no global pose supported by robot
		p_anchor_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &GlobalPoseSensorClient_ReceiveFSM::pAnchorRobotCallback, this);
	}
}

void GlobalPoseSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:GlobalPoseSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[GlobalPoseSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void GlobalPoseSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void GlobalPoseSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
//	if (p_publish_world_anchor) {
		// restart anchor->robot tf if no global pose supported by robot switched to
//		p_anchor_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &GlobalPoseSensorClient_ReceiveFSM::pAnchorRobotCallback, this);
//	}
}

void GlobalPoseSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	// if (p_anchor_timer.isValid()) {
	// 	p_anchor_timer.stop();
	// }
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("GlobalPoseSensorClient", "create QUERY timer to get global pose from %s", component.str().c_str());
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &GlobalPoseSensorClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("GlobalPoseSensorClient", "invalid hz %.2f for QUERY timer to get global pose from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("GlobalPoseSensorClient", "create EVENT to get global pose from %s", component.str().c_str());
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_global_pose_msg, p_hz);
	}
}

void GlobalPoseSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("GlobalPoseSensorClient", "cancel EVENT for global pose by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_global_pose_msg);
	}
	// if (p_anchor_timer.isValid()) {
	// 	p_anchor_timer.stop();
	// }
}

void GlobalPoseSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_global_pose_msg, p_remote_addr);
	}
}

void GlobalPoseSensorClient_ReceiveFSM::pAnchorRobotCallback(const ros::TimerEvent& event)
{
	p_tf_anchor_robot.header.stamp = ros::Time::now();
	p_tf_anchor_robot_ground.header.stamp = ros::Time::now();
	p_tf_broadcaster.sendTransform(p_tf_anchor_robot);
	p_tf_broadcaster.sendTransform(p_tf_anchor_robot_ground);
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
	sensor_msgs::NavSatFix fix;
	fix.latitude = msg.getBody()->getGlobalPoseRec()->getLatitude();
	fix.longitude = msg.getBody()->getGlobalPoseRec()->getLongitude();
	if (msg.getBody()->getGlobalPoseRec()->isAltitudeValid()) {
		fix.altitude = msg.getBody()->getGlobalPoseRec()->getAltitude();
	}
	fix.status.status = 1;
	if (msg.getBody()->getGlobalPoseRec()->isTimeStampValid()) {
		// get timestamp
		ReportGlobalPose::Body::GlobalPoseRec::TimeStamp *ts = msg.getBody()->getGlobalPoseRec()->getTimeStamp();
		iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		fix.header.stamp = stamp.ros_time;
	} else {
		fix.header.stamp = ros::Time::now();
	}

	p_pub_navsatfix.publish(fix);
	sensor_msgs::Imu imu;
	tf::Quaternion quat;
	tf::Quaternion quat_yaw;
	if (msg.getBody()->getGlobalPoseRec()->isYawValid()) {
		ROS_DEBUG_NAMED("GlobalPoseSensorClient", "quat from: %.2f, %.2f, %.2f", msg.getBody()->getGlobalPoseRec()->getRoll(), msg.getBody()->getGlobalPoseRec()->getPitch(), msg.getBody()->getGlobalPoseRec()->getYaw());
		quat = tf::createQuaternionFromRPY(msg.getBody()->getGlobalPoseRec()->getRoll(), msg.getBody()->getGlobalPoseRec()->getPitch(), msg.getBody()->getGlobalPoseRec()->getYaw());
		imu.orientation.x = quat.x();
		imu.orientation.y = quat.y();
		imu.orientation.z = quat.z();
		imu.orientation.w = quat.w();
		p_pub_imu.publish(imu);
		quat_yaw = tf::createQuaternionFromRPY(0, 0, msg.getBody()->getGlobalPoseRec()->getYaw());
	} else {
		quat = tf::createQuaternionFromRPY(0, 0, 0);
		quat_yaw = tf::createQuaternionFromRPY(0, 0, 0);
	}

	geometry_msgs::TransformStamped transform;
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
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
		p_tf_anchor.transform.rotation.x = q.x();
		p_tf_anchor.transform.rotation.y = q.y();
		p_tf_anchor.transform.rotation.z = q.z();
		p_tf_anchor.transform.rotation.w = q.w();
		p_tf_anchor.header.stamp = ros::Time::now();
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
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = this->p_tf_frame_anchor;
	transform.child_frame_id = this->p_tf_frame_robot;
	p_tf_anchor.header.stamp = transform.header.stamp;
	if (! transform.child_frame_id.empty()) {
		if (p_publish_world_anchor) {
			ROS_DEBUG_NAMED("GlobalPoseSensorClient", "update anchor tf %s -> %s, stamp: %d.%d", this->p_tf_frame_world.c_str(), this->p_tf_frame_anchor.c_str(), p_tf_anchor.header.stamp.sec, p_tf_anchor.header.stamp.nsec);
			p_tf_broadcaster.sendTransform(p_tf_anchor);
		} else {
			transform.header.frame_id = this->p_tf_frame_world;
		}
		ROS_DEBUG_NAMED("GlobalPoseSensorClient", "tf %s -> %s, stamp: %d.%d", this->p_tf_frame_anchor.c_str(), this->p_tf_frame_robot.c_str(), transform.header.stamp.sec, transform.header.stamp.nsec);
		p_tf_anchor_robot = transform;
		p_tf_broadcaster.sendTransform(transform);
		transform.transform.rotation.x = quat_yaw.x();
		transform.transform.rotation.y = quat_yaw.y();
		transform.transform.rotation.z = quat_yaw.z();
		transform.transform.rotation.w = quat_yaw.w();
		transform.child_frame_id = this->p_tf_frame_robot + "_ground";
		ROS_DEBUG_NAMED("GlobalPoseSensorClient", "tf %s -> %s, stamp: %d.%d", this->p_tf_frame_anchor.c_str(), transform.child_frame_id.c_str(), transform.header.stamp.sec, transform.header.stamp.nsec);
		p_tf_anchor_robot_ground = transform;
		p_tf_broadcaster.sendTransform(transform);
	}
	// publish global pose
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = easting;
	pose.pose.position.y = northing;
	pose.pose.position.z = fix.altitude;
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
	pose.pose.orientation.w = quat.w();
	pose.header.stamp = transform.header.stamp;
	pose.header.frame_id = zone;
	p_pub_pose.publish(pose);
}

void GlobalPoseSensorClient_ReceiveFSM::anchorFixReceived(const sensor_msgs::NavSatFix::ConstPtr& fix)
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
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
		p_tf_anchor.transform.rotation.x = q.x();
		p_tf_anchor.transform.rotation.y = q.y();
		p_tf_anchor.transform.rotation.z = q.z();
		p_tf_anchor.transform.rotation.w = q.w();
		p_tf_anchor.header.stamp = ros::Time::now();
		p_tf_anchor.header.frame_id = this->p_tf_frame_world;
		p_tf_anchor.child_frame_id = this->p_tf_frame_anchor;
		ROS_DEBUG_NAMED("GlobalPoseSensorClient", "update anchor tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_anchor.c_str());
	}
}



};

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


#include "urn_jaus_jss_mobility_LocalPoseSensorClient/LocalPoseSensorClient_ReceiveFSM.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <fkie_iop_builder/timestamp.h>
#include <fkie_iop_builder/util.h>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>

using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_LocalPoseSensorClient
{



LocalPoseSensorClient_ReceiveFSM::LocalPoseSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalPoseSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	p_tf_frame_odom = "odom";
	p_tf_frame_robot = "base_link";
	p_send_inverse_trafo = true;
	p_query_local_pose_msg.getBody()->getQueryLocalPoseRec()->setPresenceVector(65535);
	p_has_access = false;
	p_hz = 10.0;
}



LocalPoseSensorClient_ReceiveFSM::~LocalPoseSensorClient_ReceiveFSM()
{
	delete context;
}

void LocalPoseSensorClient_ReceiveFSM::setupNotifications()
{
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_LocalPoseSensorClient_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControlClient_ReceiveFSM");
	pAccessControlClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_LocalPoseSensorClient_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControlClient_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "LocalPoseSensorClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving_Ready", "LocalPoseSensorClient_ReceiveFSM");
	registerNotification("Receiving", pAccessControlClient_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControlClient_ReceiveFSM_Receiving", "LocalPoseSensorClient_ReceiveFSM");
	iop::Config cfg("~LocalPoseSensorClient");
	cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("send_inverse_trafo", p_send_inverse_trafo, p_send_inverse_trafo);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_pose = cfg.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
	p_pub_odom = cfg.advertise<nav_msgs::Odometry>("odom", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:LocalPoseSensor", 1, 0);
}

void LocalPoseSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:LocalPoseSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[LocalPoseSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void LocalPoseSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void LocalPoseSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void LocalPoseSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("LocalPoseSensorClient", "create QUERY timer to get local pose from %s", component.str().c_str());
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &LocalPoseSensorClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("LocalPoseSensorClient", "invalid hz %.2f for QUERY timer to get local pose from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("LocalPoseSensorClient", "create EVENT to get local pose from %s", component.str().c_str());
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_local_pose_msg, p_hz);
	}
}

void LocalPoseSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("LocalPoseSensorClient", "cancel event for local pose %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_local_pose_msg);
	}
}

void LocalPoseSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_local_pose_msg, p_remote_addr);
	}
}

void LocalPoseSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportLocalPose report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportLocalPoseAction(report, transport_data);
}

void LocalPoseSensorClient_ReceiveFSM::handleReportLocalPoseAction(ReportLocalPose msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	try {
		ReportLocalPose::Body::LocalPoseRec *pose = msg.getBody()->getLocalPoseRec();
		// send quaternion
		tf2::Quaternion q;
		q.setRPY(pround(pose->getRoll()), pround(pose->getPitch()), pround(pose->getYaw()));
		tf2::Vector3 r(pround(pose->getX()), pround(pose->getY()), pround(pose->getZ()));
		tf2::Transform transform(q, r);
		geometry_msgs::TransformStamped tf_msg;
		tf_msg.header.stamp = ros::Time::now();
		ReportLocalPose::Body::LocalPoseRec::TimeStamp *ts = pose->getTimeStamp();
		iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		tf_msg.header.stamp = ros::Time::now();
		if (!p_send_inverse_trafo) {
			tf_msg.transform.translation.x = transform.getOrigin().getX();
			tf_msg.transform.translation.y = transform.getOrigin().getY();
			tf_msg.transform.translation.z = transform.getOrigin().getZ();
			tf_msg.transform.rotation.x = transform.getRotation().getX();
			tf_msg.transform.rotation.y = transform.getRotation().getY();
			tf_msg.transform.rotation.z = transform.getRotation().getZ();
			tf_msg.transform.rotation.w = transform.getRotation().getW();
			tf_msg.header.frame_id = this->p_tf_frame_odom;
			tf_msg.child_frame_id = this->p_tf_frame_robot;
			ROS_DEBUG_NAMED("LocalPoseSensorClient", "tf %s -> %s", this->p_tf_frame_odom.c_str(), this->p_tf_frame_robot.c_str());
		} else {
			tf2::Transform inverse = transform.inverse();
			tf_msg.transform.translation.x = inverse.getOrigin().getX();
			tf_msg.transform.translation.y = inverse.getOrigin().getY();
			tf_msg.transform.translation.z = inverse.getOrigin().getZ();
			tf_msg.transform.rotation.x = inverse.getRotation().getX();
			tf_msg.transform.rotation.y = inverse.getRotation().getY();
			tf_msg.transform.rotation.z = inverse.getRotation().getZ();
			tf_msg.transform.rotation.w = inverse.getRotation().getW();
			tf_msg.header.frame_id = this->p_tf_frame_robot;
			tf_msg.child_frame_id = this->p_tf_frame_odom;
			ROS_DEBUG_NAMED("LocalPoseSensorClient", "tf %s i-> %s", this->p_tf_frame_robot.c_str(), this->p_tf_frame_odom.c_str());
		}
		if (! tf_msg.child_frame_id.empty()) {
			p_tf_broadcaster.sendTransform(tf_msg);
		}

		// send pose stamped
		geometry_msgs::PoseStamped ros_pose;
		ros_pose.pose.position.x = pose->getX();
		ros_pose.pose.position.y = pose->getY();
		ros_pose.pose.position.z = pose->getZ();
		ros_pose.pose.orientation.x = q.x();
		ros_pose.pose.orientation.y = q.y();
		ros_pose.pose.orientation.z = q.z();
		ros_pose.pose.orientation.w = q.w();
		ros_pose.header.stamp = stamp.ros_time;
		ros_pose.header.frame_id = this->p_tf_frame_odom;
		p_pub_pose.publish(ros_pose);

		// send odometry
		nav_msgs::Odometry ros_odom;
		ros_odom.pose.pose.position.x = pose->getX();
		ros_odom.pose.pose.position.y = pose->getY();
		ros_odom.pose.pose.position.z = pose->getZ();
		ros_odom.pose.pose.orientation.x = q.x();
		ros_odom.pose.pose.orientation.y = q.y();
		ros_odom.pose.pose.orientation.z = q.z();
		ros_odom.pose.pose.orientation.w = q.w();
		ros_odom.header.stamp = stamp.ros_time;
		ros_odom.header.frame_id = this->p_tf_frame_odom;
		p_pub_odom.publish(ros_odom);
	} catch (std::exception &e) {
		ROS_WARN("LocalPoseSensorClient: can not publish tf, pose or odometry: %s", e.what());
	}
}





};

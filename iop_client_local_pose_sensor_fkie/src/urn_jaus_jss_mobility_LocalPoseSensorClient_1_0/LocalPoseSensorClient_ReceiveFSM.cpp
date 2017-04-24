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


#include "urn_jaus_jss_mobility_LocalPoseSensorClient_1_0/LocalPoseSensorClient_ReceiveFSM.h"

#include <tf/transform_datatypes.h>
#include <iop_builder_fkie/timestamp.h>

using namespace JTS;

namespace urn_jaus_jss_mobility_LocalPoseSensorClient_1_0
{



LocalPoseSensorClient_ReceiveFSM::LocalPoseSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient_1_0::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM)
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
	p_pnh = ros::NodeHandle("~");
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

	p_pnh.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	ROS_INFO("  tf_frame_odom: %s", p_tf_frame_odom.c_str());
	p_pnh.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	ROS_INFO("  tf_frame_robot: %s", p_tf_frame_robot.c_str());
	p_pub_pose = p_nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
	p_pub_odom = p_nh.advertise<nav_msgs::Odometry>("odom", 1, true);
	p_ocu_control_layer_slave.set_access_state_handler(&LocalPoseSensorClient_ReceiveFSM::pAccessStateHandler, this);
	p_ocu_control_layer_slave.init(*(jausRouter->getJausAddress()), "urn:jaus:jss:mobility:LocalPoseSensor", 1, 0);
}

void LocalPoseSensorClient_ReceiveFSM::pAccessStateHandler(JausAddress &address, unsigned char code)
{
	if (code == OcuControlSlave::ACCESS_STATE_CONTROL_ACCEPTED) {
		// create event
		ROS_INFO_NAMED("LocalPoseSensorClient", "create event to get local pose from %d.%d.%d",
				address.getSubsystemID(), address.getNodeID(), address.getComponentID());
		pEventsClient_ReceiveFSM->create_event(&LocalPoseSensorClient_ReceiveFSM::pHandleEventReportLocalPose, this, address, p_query_local_pose_msg, 10.0, 1);
	} else if (code == OcuControlSlave::ACCESS_CONTROL_RELEASE) {
		pEventsClient_ReceiveFSM->cancel_event(address, p_query_local_pose_msg);
		ROS_INFO_NAMED("LocalPoseSensorClient", "cancel event for local pose by %d.%d.%d",
				address.getSubsystemID(), address.getNodeID(), address.getComponentID());
	}
}

void LocalPoseSensorClient_ReceiveFSM::pHandleEventReportLocalPose(Receive::Body::ReceiveRec &transport_data, urn_jaus_jss_core_EventsClient_1_0::Event &msg)
{
	ReportLocalPose report;
	report.decode(msg.getBody()->getEventRec()->getReportMessage()->getData());
	handleReportLocalPoseAction(report, transport_data);
}

void LocalPoseSensorClient_ReceiveFSM::handleReportLocalPoseAction(ReportLocalPose msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	try {
		geometry_msgs::TransformStamped tf_msg;
		ReportLocalPose::Body::LocalPoseRec *pose = msg.getBody()->getLocalPoseRec();
		tf_msg.transform.translation.x = pose->getX();
		tf_msg.transform.translation.y = pose->getY();
		tf_msg.transform.translation.z = pose->getZ();
		double roll = pose->getRoll();
		double pitch = pose->getPitch();
		double yaw = pose->getYaw();
		tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
		tf_msg.transform.rotation.x = q.x();
		tf_msg.transform.rotation.y = q.y();
		tf_msg.transform.rotation.z = q.z();
		tf_msg.transform.rotation.w = q.w();
		ReportLocalPose::Body::LocalPoseRec::TimeStamp *ts = pose->getTimeStamp();
		iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		tf_msg.header.stamp = stamp.ros_time;
		tf_msg.header.frame_id = this->p_tf_frame_odom;
		tf_msg.child_frame_id = this->p_tf_frame_robot;
		ROS_DEBUG_NAMED("RangeSensorClient", "tf %s -> %s", this->p_tf_frame_robot.c_str(), this->p_tf_frame_robot.c_str());
		if (! tf_msg.child_frame_id.empty()) {
			p_tf_broadcaster.sendTransform(tf_msg);
		}
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

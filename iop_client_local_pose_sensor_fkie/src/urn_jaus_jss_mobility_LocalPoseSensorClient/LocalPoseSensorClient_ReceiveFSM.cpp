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

#include <tf/transform_datatypes.h>
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>

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
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:LocalPoseSensor", 1, 0);
}

void LocalPoseSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:LocalPoseSensor") == 0) {
		p_control_addr = component;
		ROS_INFO_NAMED("LocalPoseSensorClient", "create event to get local pose from %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
		pEventsClient_ReceiveFSM->create_event(&LocalPoseSensorClient_ReceiveFSM::pHandleEventReportLocalPose, this, component, p_query_local_pose_msg, 10.0, 1);
	} else {
		ROS_WARN_STREAM("[LocalPoseSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void LocalPoseSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("LocalPoseSensorClient", "create monitor event to get local pose from %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pEventsClient_ReceiveFSM->create_event(&LocalPoseSensorClient_ReceiveFSM::pHandleEventReportLocalPose, this, component, p_query_local_pose_msg, 10.0, 1);
}

void LocalPoseSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_control_addr = JausAddress(0);
	ROS_INFO_NAMED("LocalPoseSensorClient", "cancel event for local pose by %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pEventsClient_ReceiveFSM->cancel_event(component, p_query_local_pose_msg);
}

void LocalPoseSensorClient_ReceiveFSM::pHandleEventReportLocalPose(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata)
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

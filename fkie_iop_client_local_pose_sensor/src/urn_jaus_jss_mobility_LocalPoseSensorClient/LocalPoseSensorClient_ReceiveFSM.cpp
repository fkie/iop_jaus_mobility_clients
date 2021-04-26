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
#include <fkie_iop_builder/util.h>
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_LocalPoseSensorClient
{



LocalPoseSensorClient_ReceiveFSM::LocalPoseSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "LocalPoseSensorClient", 10.0),
  logger(cmp->get_logger().get_child("LocalPoseSensorClient")),
  p_tf_broadcaster(cmp)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalPoseSensorClient_ReceiveFSMContext(*this);

	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_odom = "odom";
	p_tf_frame_robot = "base_link";
	p_send_inverse_trafo = true;
	p_query_local_pose_msg.getBody()->getQueryLocalPoseRec()->setPresenceVector(65535);
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
}


void LocalPoseSensorClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "LocalPoseSensorClient");
	cfg.declare_param<std::string>("tf_frame_odom", p_tf_frame_odom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the odometry frame id.",
		"Default: 'odom'");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id of the robot.",
		"Default: 'base_link'");
	cfg.declare_param<bool>("send_inverse_trafo", p_send_inverse_trafo, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
		"Sets the transform direction while publish TF frame. True: tf_frame_robot -> tf_frame_odom",
		"Default: true");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 10.0");
	cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("send_inverse_trafo", p_send_inverse_trafo, p_send_inverse_trafo);
	cfg.param("hz", p_hz, p_hz, false);
	p_pub_pose = cfg.create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
	p_pub_odom = cfg.create_publisher<nav_msgs::msg::Odometry>("odom", 1);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:mobility:LocalPoseSensor", 1, 0);
	this->set_event_name("local pose");
}

void LocalPoseSensorClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_local_pose_msg, p_hz);
}

void LocalPoseSensorClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_local_pose_msg);
	stop_query(remote_addr);
}

void LocalPoseSensorClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_local_pose_msg, remote_addr);
}

void LocalPoseSensorClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
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
		auto tf_msg = geometry_msgs::msg::TransformStamped();
		tf_msg.header.stamp = cmp->now();
		ReportLocalPose::Body::LocalPoseRec::TimeStamp *ts = pose->getTimeStamp();
		iop::Timestamp stamp = cmp->from_iop(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		tf_msg.header.stamp = cmp->now();
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
			RCLCPP_DEBUG(logger, "tf %s -> %s", this->p_tf_frame_odom.c_str(), this->p_tf_frame_robot.c_str());
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
			RCLCPP_DEBUG(logger, "tf %s i-> %s", this->p_tf_frame_robot.c_str(), this->p_tf_frame_odom.c_str());
		}
		if (! tf_msg.child_frame_id.empty()) {
			p_tf_broadcaster.sendTransform(tf_msg);
		}

		// send pose stamped
		auto ros_pose = geometry_msgs::msg::PoseStamped();
		ros_pose.pose.position.x = pose->getX();
		ros_pose.pose.position.y = pose->getY();
		ros_pose.pose.position.z = pose->getZ();
		ros_pose.pose.orientation.x = q.x();
		ros_pose.pose.orientation.y = q.y();
		ros_pose.pose.orientation.z = q.z();
		ros_pose.pose.orientation.w = q.w();
		ros_pose.header.stamp = stamp.ros_time;
		ros_pose.header.frame_id = this->p_tf_frame_odom;
		p_pub_pose->publish(ros_pose);

		// send odometry
		auto ros_odom = nav_msgs::msg::Odometry();
		ros_odom.pose.pose.position.x = pose->getX();
		ros_odom.pose.pose.position.y = pose->getY();
		ros_odom.pose.pose.position.z = pose->getZ();
		ros_odom.pose.pose.orientation.x = q.x();
		ros_odom.pose.pose.orientation.y = q.y();
		ros_odom.pose.pose.orientation.z = q.z();
		ros_odom.pose.pose.orientation.w = q.w();
		ros_odom.header.stamp = stamp.ros_time;
		ros_odom.header.frame_id = this->p_tf_frame_odom;
		p_pub_odom->publish(ros_odom);
	} catch (std::exception &e) {
		RCLCPP_WARN(logger, "LocalPoseSensorClient: can not publish tf, pose or odometry: %s", e.what());
	}
}





}

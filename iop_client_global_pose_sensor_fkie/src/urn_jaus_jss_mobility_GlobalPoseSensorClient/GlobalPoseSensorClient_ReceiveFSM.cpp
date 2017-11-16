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

#include <tf/transform_datatypes.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>


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
	p_tf_frame_robot = "base_link";
	p_query_global_pose_msg.getBody()->getQueryGlobalPoseRec()->setPresenceVector(65535);
	p_has_access = false;
	p_hz = 10.0;
}



GlobalPoseSensorClient_ReceiveFSM::~GlobalPoseSensorClient_ReceiveFSM()
{

	if (p_query_timer.isValid()) {
		p_query_timer.stop();
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
	iop::Config cfg("~GlobalWaypointDriverClient");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_navsatfix = cfg.advertise<sensor_msgs::NavSatFix>("fix", 1, true);
	p_pub_imu = cfg.advertise<sensor_msgs::Imu>("imu", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:GlobalPoseSensor", 1, 0);
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
}

void GlobalPoseSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
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
}

void GlobalPoseSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_global_pose_msg, p_remote_addr);
	}
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
	/// Insert User Code HERE
	sensor_msgs::NavSatFix fix;
	fix.latitude = msg.getBody()->getGlobalPoseRec()->getLatitude();
	fix.longitude = msg.getBody()->getGlobalPoseRec()->getLongitude();
	fix.altitude = msg.getBody()->getGlobalPoseRec()->getAltitude();
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
	if (msg.getBody()->getGlobalPoseRec()->isYawValid()) {
		quat.setRPY(msg.getBody()->getGlobalPoseRec()->getRoll(), msg.getBody()->getGlobalPoseRec()->getPitch(), msg.getBody()->getGlobalPoseRec()->getYaw());
		imu.orientation.x = quat.x();
		imu.orientation.y = quat.y();
		imu.orientation.z = quat.z();
		imu.orientation.w = quat.w();
		p_pub_imu.publish(imu);
	}

	tf::StampedTransform transform;
	tf::Transform btTrans;
	double northing, easting;
	std::string zone;
	gps_common::LLtoUTM(fix.latitude, fix.longitude, northing, easting, zone);
	tf::Vector3 translation(easting, northing, 0.0);
	btTrans = tf::Transform(quat, translation);
	transform.stamp_ = fix.header.stamp;
	transform.setData(btTrans);
	transform.frame_id_ = this->p_tf_frame_world;
	transform.child_frame_id_ = this->p_tf_frame_robot;
	ROS_DEBUG_NAMED("GlobalPoseSensorClient", "tf %s -> %s", this->p_tf_frame_world.c_str(), this->p_tf_frame_robot.c_str());
	if (! transform.child_frame_id_.empty()) {
		p_tf_broadcaster.sendTransform(transform);
	}

}





};

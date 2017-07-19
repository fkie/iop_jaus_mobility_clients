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
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <iop_builder_fkie/timestamp.h>
#include <iop_ocu_slavelib_fkie/Slave.h>


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
	ros::NodeHandle pnh;
	p_pub_navsatfix = pnh.advertise<sensor_msgs::NavSatFix>("fix", 1, true);
	p_pub_imu = pnh.advertise<sensor_msgs::Imu>("imu", 1, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:GlobalPoseSensor", 1, 0);
}

void GlobalPoseSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:GlobalPoseSensor") == 0) {
		p_control_addr = component;
		ROS_INFO_NAMED("LocalPoseSensorClient", "create event to get global pose from %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
		pEventsClient_ReceiveFSM->create_event(&GlobalPoseSensorClient_ReceiveFSM::pHandleEventReportGlobalPose, this, component, p_query_global_pose_msg, 10.0, 1);
	} else {
		ROS_WARN_STREAM("[LocalPoseSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void GlobalPoseSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("LocalPoseSensorClient", "create monitor event to get global pose from %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pEventsClient_ReceiveFSM->create_event(&GlobalPoseSensorClient_ReceiveFSM::pHandleEventReportGlobalPose, this, component, p_query_global_pose_msg, 10.0, 1);
}

void GlobalPoseSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_control_addr = JausAddress(0);
	ROS_INFO_NAMED("LocalPoseSensorClient", "cancel event for global pose by %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pEventsClient_ReceiveFSM->cancel_event(component, p_query_global_pose_msg);
}

void GlobalPoseSensorClient_ReceiveFSM::pHandleEventReportGlobalPose(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata)
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
		fix.header.stamp  = stamp.ros_time;
	}

	p_pub_navsatfix.publish(fix);
	if (msg.getBody()->getGlobalPoseRec()->isYawValid()) {
		tf::Quaternion q = tf::createQuaternionFromRPY(msg.getBody()->getGlobalPoseRec()->getRoll(), msg.getBody()->getGlobalPoseRec()->getPitch(), msg.getBody()->getGlobalPoseRec()->getYaw());
		sensor_msgs::Imu imu;
		imu.orientation.x = q.x();
		imu.orientation.y = q.y();
		imu.orientation.z = q.z();
		imu.orientation.w = q.w();
		p_pub_imu.publish(imu);
	}
}





};

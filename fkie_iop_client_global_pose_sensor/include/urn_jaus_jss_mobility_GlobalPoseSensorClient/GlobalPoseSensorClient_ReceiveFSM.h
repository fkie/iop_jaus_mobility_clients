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


#ifndef GLOBALPOSESENSORCLIENT_RECEIVEFSM_H
#define GLOBALPOSESENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_GlobalPoseSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_GlobalPoseSensorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "GlobalPoseSensorClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_mobility_GlobalPoseSensorClient
{

class DllExport GlobalPoseSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	GlobalPoseSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~GlobalPoseSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportGeomagneticPropertyAction(ReportGeomagneticProperty msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportGlobalPoseAction(ReportGlobalPose msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);

	GlobalPoseSensorClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	std::string p_tf_frame_world;
	std::string p_tf_frame_anchor;
	std::string p_tf_frame_robot;
	tf2_ros::TransformBroadcaster p_tf_broadcaster;
	geometry_msgs::msg::TransformStamped p_tf_anchor;
	double p_anchor_northing, p_anchor_easting, p_anchor_altitude;
	bool p_publish_world_anchor;

	iop::Timer p_query_timer;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr p_pub_navsatfix;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr p_pub_imu;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr p_pub_pose;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr p_sub_anchorfix;
	double p_hz;

	urn_jaus_jss_mobility_GlobalPoseSensorClient::QueryGlobalPose p_query_global_pose_msg;
	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback();
	void anchorFixReceived(const sensor_msgs::msg::NavSatFix::SharedPtr fix);


};

}

#endif // GLOBALPOSESENSORCLIENT_RECEIVEFSM_H

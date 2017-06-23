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

#ifndef LOCALPOSESENSORCLIENT_RECEIVEFSM_H
#define LOCALPOSESENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_LocalPoseSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalPoseSensorClient/InternalEvents/InternalEventsSet.h"

typedef JTS::Receive Receive;
typedef JTS::Send Send;

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/Messages/MessageSet.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "LocalPoseSensorClient_ReceiveFSM_sm.h"
#include <iop_ocu_control_layerlib_fkie/OcuControlLayerSlave.h>


namespace urn_jaus_jss_mobility_LocalPoseSensorClient
{

class DllExport LocalPoseSensorClient_ReceiveFSM : public JTS::StateMachine
{
public:
	LocalPoseSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~LocalPoseSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportLocalPoseAction(ReportLocalPose msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	LocalPoseSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	std::string p_tf_frame_odom;
	std::string p_tf_frame_robot;

	ros::NodeHandle p_nh;
	ros::NodeHandle p_pnh;
	tf2_ros::TransformBroadcaster p_tf_broadcaster;
	ros::Publisher p_pub_odom;
	ros::Publisher p_pub_pose;

	urn_jaus_jss_mobility_LocalPoseSensorClient::QueryLocalPose p_query_local_pose_msg;
	OcuControlLayerSlave p_ocu_control_layer_slave;
	void pAccessStateHandler(JausAddress &address, unsigned char code);
	void pHandleEventReportLocalPose(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata);

};

};

#endif // LOCALPOSESENSORCLIENT_RECEIVEFSM_H

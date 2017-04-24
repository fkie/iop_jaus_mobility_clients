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
#include "urn_jaus_jss_mobility_LocalPoseSensorClient_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalPoseSensorClient_1_0/InternalEvents/InternalEventsSet.h"

typedef JTS::Receive_1_0 Receive;
typedef JTS::Send_1_0 Send;

#include "urn_jaus_jss_core_Transport_1_0/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient_1_0/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_core_AccessControlClient_1_0/AccessControlClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "LocalPoseSensorClient_ReceiveFSM_sm.h"
#include <iop_ocu_control_layerlib_1_0_fkie/OcuControlLayerSlave.h>


namespace urn_jaus_jss_mobility_LocalPoseSensorClient_1_0
{

class DllExport LocalPoseSensorClient_ReceiveFSM : public JTS::StateMachine
{
public:
	LocalPoseSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient_1_0::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~LocalPoseSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportLocalPoseAction(ReportLocalPose msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	LocalPoseSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient_1_0::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;

	std::string p_tf_frame_odom;
	std::string p_tf_frame_robot;

	ros::NodeHandle p_nh;
	ros::NodeHandle p_pnh;
	tf2_ros::TransformBroadcaster p_tf_broadcaster;
	ros::Publisher p_pub_odom;
	ros::Publisher p_pub_pose;

	urn_jaus_jss_mobility_LocalPoseSensorClient_1_0::QueryLocalPose p_query_local_pose_msg;
	OcuControlLayerSlave p_ocu_control_layer_slave;
	void pAccessStateHandler(JausAddress &address, unsigned char code);
	void pHandleEventReportLocalPose(Receive::Body::ReceiveRec &transport_data, urn_jaus_jss_core_EventsClient_1_0::Event &msg);

};

};

#endif // LOCALPOSESENSORCLIENT_RECEIVEFSM_H

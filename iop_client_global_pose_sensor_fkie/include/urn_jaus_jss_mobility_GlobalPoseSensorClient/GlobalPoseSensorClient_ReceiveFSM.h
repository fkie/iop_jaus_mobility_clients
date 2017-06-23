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
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"

typedef JTS::Receive Receive;
typedef JTS::Send Send;

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/Messages/MessageSet.h"


#include <ros/ros.h>

#include "GlobalPoseSensorClient_ReceiveFSM_sm.h"
#include <iop_ocu_control_layerlib_fkie/OcuControlLayerSlave.h>

namespace urn_jaus_jss_mobility_GlobalPoseSensorClient
{

class DllExport GlobalPoseSensorClient_ReceiveFSM : public JTS::StateMachine
{
public:
	GlobalPoseSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM);
	virtual ~GlobalPoseSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportGeomagneticPropertyAction(ReportGeomagneticProperty msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportGlobalPoseAction(ReportGlobalPose msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	GlobalPoseSensorClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	ros::Publisher p_pub_navsatfix;
	ros::Publisher p_pub_imu;

	urn_jaus_jss_mobility_GlobalPoseSensorClient::QueryGlobalPose p_query_global_pose_msg;
	OcuControlLayerSlave p_ocu_control_layer_slave;
	void pAccessStateHandler(JausAddress &address, unsigned char code);
	void pHandleEventReportGlobalPose(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata);

};

};

#endif // GLOBALPOSESENSORCLIENT_RECEIVEFSM_H

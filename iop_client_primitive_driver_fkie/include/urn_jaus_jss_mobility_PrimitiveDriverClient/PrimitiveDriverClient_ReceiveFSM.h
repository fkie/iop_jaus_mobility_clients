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


#ifndef PRIMITIVEDRIVERCLIENT_RECEIVEFSM_H
#define PRIMITIVEDRIVERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_PrimitiveDriverClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_PrimitiveDriverClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"


#include "PrimitiveDriverClient_ReceiveFSM_sm.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int16.h>
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>


namespace urn_jaus_jss_mobility_PrimitiveDriverClient
{

class DllExport PrimitiveDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface
{
public:
	PrimitiveDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~PrimitiveDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportWrenchEffortAction(ReportWrenchEffort msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);

	PrimitiveDriverClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;

	JausAddress p_control_addr;
	ros::NodeHandle p_nh;
	ros::Subscriber p_cmd_sub;
	bool p_use_stamped;
	bool p_invert_yaw;
	double p_invert_yaw_factor;

	void cmdReceived(const geometry_msgs::Twist::ConstPtr& joy);
	void cmdStampedReceived(const geometry_msgs::TwistStamped::ConstPtr& joy);

};

};

#endif // PRIMITIVEDRIVERCLIENT_RECEIVEFSM_H

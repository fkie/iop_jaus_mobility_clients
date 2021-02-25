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

#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "PrimitiveDriverClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int16.hpp>
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>


namespace urn_jaus_jss_mobility_PrimitiveDriverClient
{

class DllExport PrimitiveDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface
{
public:
	PrimitiveDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PrimitiveDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportWrenchEffortAction(ReportWrenchEffort msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false) {}
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false) {}

	PrimitiveDriverClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	JausAddress p_remote_addr;
	bool p_has_access;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr p_cmd_sub;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr p_cmd_stamped_sub;
	bool p_use_stamped;
	bool p_invert_yaw;
	double p_invert_yaw_factor;
	double p_max_linear;
	double p_max_angular;


	void cmdReceived(const geometry_msgs::msg::Twist::SharedPtr joy);
	void cmdStampedReceived(const geometry_msgs::msg::TwistStamped::SharedPtr joy);
	double p_scale(double value, double max);

};

}

#endif // PRIMITIVEDRIVERCLIENT_RECEIVEFSM_H

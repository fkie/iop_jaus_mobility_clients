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

#include <fkie_iop_ocu_slavelib/Slave.h>
#include "urn_jaus_jss_mobility_PrimitiveDriverClient/PrimitiveDriverClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_PrimitiveDriverClient
{



PrimitiveDriverClient_ReceiveFSM::PrimitiveDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PrimitiveDriverClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	p_has_access = false;
	p_use_stamped = true;
	p_invert_yaw = true;
	p_invert_yaw_factor = -1.0;
	p_max_linear = 1.0;
	p_max_angular = 1.5;
}



PrimitiveDriverClient_ReceiveFSM::~PrimitiveDriverClient_ReceiveFSM()
{
	delete context;
}

void PrimitiveDriverClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PrimitiveDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PrimitiveDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "PrimitiveDriverClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "PrimitiveDriverClient_ReceiveFSM");
	iop::Config cfg("~PrimitiveDriverClient");
	cfg.param("use_stamped", p_use_stamped, true);
	cfg.param("invert_yaw", p_invert_yaw, true);
	double max_linear, max_angular;
	cfg.param("max_linear", max_linear, p_max_linear);
	cfg.param("max_angular", max_angular, p_max_angular);
	if (max_linear != 0) {
		p_max_linear = max_linear;
	}
	if (max_angular != 0) {
		p_max_angular = max_angular;
	}
	if (!p_invert_yaw) p_invert_yaw_factor = 1.0;
	//create ROS subscriber
	if (p_use_stamped) {
		p_cmd_sub = cfg.subscribe<geometry_msgs::TwistStamped>("joy_cmd_vel", 1, &PrimitiveDriverClient_ReceiveFSM::cmdStampedReceived, this);
	} else {
		p_cmd_sub = cfg.subscribe<geometry_msgs::Twist>("joy_cmd_vel", 1, &PrimitiveDriverClient_ReceiveFSM::cmdReceived, this);
	}
	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:PrimitiveDriver", 1, 0);
}

void PrimitiveDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:PrimitiveDriver") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[ClientPrimitiveDriver] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void PrimitiveDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	// no events supported by PrimitiveDriver, ignore
}

void PrimitiveDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}


void PrimitiveDriverClient_ReceiveFSM::handleReportWrenchEffortAction(ReportWrenchEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void PrimitiveDriverClient_ReceiveFSM::cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd)
{
	if (p_has_access) {
		SetWrenchEffort msg;
		msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortX(p_scale(cmd->linear.x, p_max_linear));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortY(p_scale(cmd->linear.y, p_max_linear));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortZ(p_scale(cmd->linear.z, p_max_linear));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortX(p_scale(cmd->angular.x, p_max_angular));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortY(p_scale(cmd->angular.y, p_max_angular));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortZ(p_scale(cmd->angular.z, p_max_angular) * p_invert_yaw_factor);
		sendJausMessage(msg, p_remote_addr);
	}

}


void PrimitiveDriverClient_ReceiveFSM::cmdStampedReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd)
{
	if (p_has_access) {
		SetWrenchEffort msg;
		msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortX(p_scale(cmd->twist.linear.x, p_max_linear));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortY(p_scale(cmd->twist.linear.y, p_max_linear));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortZ(p_scale(cmd->twist.linear.z, p_max_linear));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortX(p_scale(cmd->twist.angular.x, p_max_angular));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortY(p_scale(cmd->twist.angular.y, p_max_angular));
		msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortZ(p_scale(cmd->twist.angular.z, p_max_angular) * p_invert_yaw_factor);
		sendJausMessage(msg, p_remote_addr);
	}
//  const geometry_msgs::Twist::ConstPtr const_twist(&cmd->twist);
//  cmdReceived(const_twist);
}

double PrimitiveDriverClient_ReceiveFSM::p_scale(double value, double max)
{
	double result = value / max * 100;
	if (result > 100) {
		result = 100;
	} else if (result < -100) {
		result = -100;
	}
	return result;
}

};

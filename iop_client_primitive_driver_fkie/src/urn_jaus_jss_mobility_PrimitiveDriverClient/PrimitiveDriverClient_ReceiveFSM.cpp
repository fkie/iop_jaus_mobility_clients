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

#include "urn_jaus_jss_mobility_PrimitiveDriverClient/PrimitiveDriverClient_ReceiveFSM.h"




using namespace JTS;

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
	p_use_stamped = true;
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

	ros::NodeHandle pnh_("~");
	pnh_.param("use_stamped", p_use_stamped, true);
	ROS_INFO_STREAM("[ClientPrimitiveDriver ROS param] use_stamped: " << p_use_stamped);
	//create ROS subscriber
	if (p_use_stamped) {
		p_cmd_sub = p_nh.subscribe<geometry_msgs::TwistStamped>("joy_cmd_vel", 1, &PrimitiveDriverClient_ReceiveFSM::cmdStampedReceived, this);
	} else {
		p_cmd_sub = p_nh.subscribe<geometry_msgs::Twist>("joy_cmd_vel", 1, &PrimitiveDriverClient_ReceiveFSM::cmdReceived, this);
	}
	// initialize the control layer, which handles the access control staff
	p_ocu_control_layer_slave.init(*(jausRouter->getJausAddress()), "urn:jaus:jss:mobility:PrimitiveDriver", 1, 0);
}

void PrimitiveDriverClient_ReceiveFSM::handleReportWrenchEffortAction(ReportWrenchEffort msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}


void PrimitiveDriverClient_ReceiveFSM::cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd)
{
	JausAddress address = p_ocu_control_layer_slave.get_control_address();
	if (address.get() != 0) {
		if (p_ocu_control_layer_slave.has_access()) {
			SetWrenchEffort msg;
			msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortX(cmd->linear.x*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortY(cmd->linear.y*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortZ(cmd->linear.z*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortX(cmd->angular.x*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortY(cmd->angular.y*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortZ(cmd->angular.z*100);
			sendJausMessage(msg, address);
		}
	}

}


void PrimitiveDriverClient_ReceiveFSM::cmdStampedReceived(const geometry_msgs::TwistStamped::ConstPtr& cmd)
{
	JausAddress address = p_ocu_control_layer_slave.get_control_address();
	if (address.get() != 0) {
		if (p_ocu_control_layer_slave.has_access()) {
			SetWrenchEffort msg;
			msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortX(cmd->twist.linear.x*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortY(cmd->twist.linear.y*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveLinearEffortZ(cmd->twist.linear.z*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortX(cmd->twist.angular.x*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortY(cmd->twist.angular.y*100);
			msg.getBody()->getWrenchEffortRec()->setPropulsiveRotationalEffortZ(cmd->twist.angular.z*100);
			sendJausMessage(msg, address);
		}
//		} else {
//			p_ocu_control_layer_slave.resume();
//		}
	}
//  const geometry_msgs::Twist::ConstPtr const_twist(&cmd->twist);
//  cmdReceived(const_twist);
}


};

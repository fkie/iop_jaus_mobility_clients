

#include "urn_jaus_jss_mobility_VelocityStateSensorClient/VelocityStateSensorClient_ReceiveFSM.h"


#include <fkie_iop_builder/timestamp.h>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <fkie_iop_component/iop_config.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_VelocityStateSensorClient
{



VelocityStateSensorClient_ReceiveFSM::VelocityStateSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new VelocityStateSensorClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	p_tf_frame_robot = "base_link";
	p_use_odom = true;
	p_query_velocity_state_msg.getBody()->getQueryVelocityStateRec()->setPresenceVector(65535);
	p_has_access = false;
	p_hz = 10.0;
}



VelocityStateSensorClient_ReceiveFSM::~VelocityStateSensorClient_ReceiveFSM()
{
	delete context;
}

void VelocityStateSensorClient_ReceiveFSM::setupNotifications()
{
	pEventsClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_VelocityStateSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	pEventsClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_VelocityStateSensorClient_ReceiveFSM_Receiving_Ready", "EventsClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving_Ready", "VelocityStateSensorClient_ReceiveFSM");
	registerNotification("Receiving", pEventsClient_ReceiveFSM->getHandler(), "InternalStateChange_To_EventsClient_ReceiveFSM_Receiving", "VelocityStateSensorClient_ReceiveFSM");
	iop::Config cfg("~VelocityStateSensorClient");
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("hz", p_hz, p_hz, false, false);
	p_pub_odom = cfg.advertise<nav_msgs::Odometry>("velocity_state_odom", 5, true);
	p_pub_twist = cfg.advertise<geometry_msgs::TwistStamped>("velocity_state_twist", 5, true);
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:VelocityStateSensor", 1, 0);

}

void VelocityStateSensorClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:VelocityStateSensor") == 0) {
		p_remote_addr = component;
		p_has_access = true;
	} else {
		ROS_WARN_STREAM("[VelocityStateSensorClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void VelocityStateSensorClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void VelocityStateSensorClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
}

void VelocityStateSensorClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("VelocityStateSensorClient", "create QUERY timer to get velocity state from %s", component.str().c_str());
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &VelocityStateSensorClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("VelocityStateSensorClient", "invalid hz %.2f for QUERY timer to get velocity state from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("VelocityStateSensorClient", "create EVENT to get velocity state from %s", component.str().c_str());
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_velocity_state_msg, p_hz);
	}
}

void VelocityStateSensorClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("VelocityStateSensorClient", "cancel EVENT for velocity state by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_velocity_state_msg);
	}
}

void VelocityStateSensorClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_velocity_state_msg, p_remote_addr);
	}
}

void VelocityStateSensorClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportVelocityState report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportVelocityStateAction(report, transport_data);
}

void VelocityStateSensorClient_ReceiveFSM::handleReportVelocityStateAction(ReportVelocityState msg, Receive::Body::ReceiveRec transportData)
{
	nav_msgs::Odometry odom;
	geometry_msgs::TwistStamped twist;
	odom.header.frame_id = p_tf_frame_robot;
	twist.header.frame_id = p_tf_frame_robot;
	if (msg.getBody()->getReportVelocityStateRec()->isTimeStampValid()) {
		// get timestamp
		ReportVelocityState::Body::ReportVelocityStateRec::TimeStamp *ts = msg.getBody()->getReportVelocityStateRec()->getTimeStamp();
		iop::Timestamp stamp(ts->getDay(), ts->getHour(), ts->getMinutes(), ts->getSeconds(), ts->getMilliseconds());
		odom.header.stamp = stamp.ros_time;
		twist.header.stamp = stamp.ros_time;
	} else {
		odom.header.stamp = ros::Time::now();
		twist.header.stamp = ros::Time::now();
	}

	double roll, pitch, yaw = 0.0;
	if (msg.getBody()->getReportVelocityStateRec()->isPitchRateValid()) {
		pitch = msg.getBody()->getReportVelocityStateRec()->getPitchRate();
	}
	if (msg.getBody()->getReportVelocityStateRec()->isYawRateValid()) {
		yaw = msg.getBody()->getReportVelocityStateRec()->getYawRate();
	}
	if (msg.getBody()->getReportVelocityStateRec()->isRollRateValid()) {
		roll = msg.getBody()->getReportVelocityStateRec()->getRollRate();
	}
	odom.twist.twist.angular.x = pitch;
	odom.twist.twist.angular.y = roll;
	odom.twist.twist.angular.z = yaw;
	twist.twist.angular.x = pitch;
	twist.twist.angular.y = roll;
	twist.twist.angular.z = yaw;
	double x, y, z = 0.0;
	if (msg.getBody()->getReportVelocityStateRec()->isVelocity_XValid()) {
		x = msg.getBody()->getReportVelocityStateRec()->getVelocity_X();
	}
	if (msg.getBody()->getReportVelocityStateRec()->isVelocity_YValid()) {
		y = msg.getBody()->getReportVelocityStateRec()->getVelocity_Y();
	}
	if (msg.getBody()->getReportVelocityStateRec()->isVelocity_ZValid()) {
		z = msg.getBody()->getReportVelocityStateRec()->getVelocity_Z();
	}
	odom.twist.twist.linear.x = x;
	odom.twist.twist.linear.y = y;
	odom.twist.twist.linear.z = z;
	twist.twist.linear.x = x;
	twist.twist.linear.y = y;
	twist.twist.linear.z = z;

	// publish
	p_pub_twist.publish(twist);
	p_pub_odom.publish(odom);
}

};

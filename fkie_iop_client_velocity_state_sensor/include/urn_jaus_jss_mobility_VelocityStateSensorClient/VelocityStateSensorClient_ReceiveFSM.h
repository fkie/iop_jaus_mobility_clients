

#ifndef VELOCITYSTATESENSORCLIENT_RECEIVEFSM_H
#define VELOCITYSTATESENSORCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_VelocityStateSensorClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_VelocityStateSensorClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "VelocityStateSensorClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>


#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_mobility_VelocityStateSensorClient
{

class DllExport VelocityStateSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	VelocityStateSensorClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~VelocityStateSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportVelocityStateAction(ReportVelocityState msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);



	VelocityStateSensorClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	bool p_use_odom;
	std::string p_tf_frame_robot;
	iop::Timer p_query_timer;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_pub_odom;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr p_pub_twist;
	double p_hz;

	urn_jaus_jss_mobility_VelocityStateSensorClient::QueryVelocityState p_query_velocity_state_msg;
	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback();

};

}

#endif // VELOCITYSTATESENSORCLIENT_RECEIVEFSM_H

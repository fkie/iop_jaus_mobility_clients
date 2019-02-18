

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

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"


#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include "VelocityStateSensorClient_ReceiveFSM_sm.h"
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_mobility_VelocityStateSensorClient
{

class DllExport VelocityStateSensorClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	VelocityStateSensorClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM);
	virtual ~VelocityStateSensorClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

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
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;

	bool p_use_odom;
	std::string p_tf_frame_robot;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Publisher p_pub_odom;
	ros::Publisher p_pub_twist;
	double p_hz;

	urn_jaus_jss_mobility_VelocityStateSensorClient::QueryVelocityState p_query_velocity_state_msg;
	JausAddress p_remote_addr;
	bool p_has_access;
	void pQueryCallback(const ros::TimerEvent& event);

};

};

#endif // VELOCITYSTATESENSORCLIENT_RECEIVEFSM_H

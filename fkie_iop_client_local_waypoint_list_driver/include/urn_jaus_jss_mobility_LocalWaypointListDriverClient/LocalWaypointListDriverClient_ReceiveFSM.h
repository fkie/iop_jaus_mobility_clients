

#ifndef LOCALWAYPOINTLISTDRIVERCLIENT_RECEIVEFSM_H
#define LOCALWAYPOINTLISTDRIVERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_LocalWaypointListDriverClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalWaypointListDriverClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ListManagerClient/ListManagerClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

#include "LocalWaypointListDriverClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_LocalWaypointListDriverClient
{

class DllExport LocalWaypointListDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	LocalWaypointListDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_ListManagerClient::ListManagerClient_ReceiveFSM* pListManagerClient_ReceiveFSM);
	virtual ~LocalWaypointListDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportActiveElementAction(ReportActiveElement msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportLocalWaypointAction(ReportLocalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	void create_events(std::string service_uri, JausAddress component, bool by_query=false);
	void cancel_events(std::string service_uri, JausAddress component, bool by_query=false);
	/// Guard Methods



	LocalWaypointListDriverClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_core_ListManagerClient::ListManagerClient_ReceiveFSM* pListManagerClient_ReceiveFSM;

	JausAddress p_remote_addr;
	ros::NodeHandle p_nh;
	ros::Timer p_query_timer;
	ros::Subscriber p_sub_path;
	ros::Subscriber p_sub_speed;
	ros::Publisher p_pub_path;
	float p_travel_speed;
	float p_wp_tolerance;
	std::string p_tf_frame_robot;
	QueryLocalWaypoint p_query_local_waypoint_msg;
	tf::TransformListener tfListener;
	bool p_has_access;
	double p_hz;
	bool p_new_rospath_received;

	void pCmdPath(const nav_msgs::Path::ConstPtr& msg);
	void pCmdPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pCmdSpeed(const std_msgs::Float32::ConstPtr& msg);
	void pQueryCallback(const ros::TimerEvent& event);
	void pListState(bool success, unsigned int count);

};

};

#endif // GLOBALWAYPOINTLISTDRIVERCLIENT_RECEIVEFSM_H

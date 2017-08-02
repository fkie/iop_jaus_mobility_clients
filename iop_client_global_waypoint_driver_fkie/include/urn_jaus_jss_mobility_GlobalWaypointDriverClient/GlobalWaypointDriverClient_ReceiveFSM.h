

#ifndef GLOBALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H
#define GLOBALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_GlobalWaypointDriverClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_GlobalWaypointDriverClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <iop_ocu_slavelib_fkie/SlaveHandlerInterface.h>

#include "GlobalWaypointDriverClient_ReceiveFSM_sm.h"

namespace urn_jaus_jss_mobility_GlobalWaypointDriverClient
{

class DllExport GlobalWaypointDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface
{
public:
	GlobalWaypointDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM);
	virtual ~GlobalWaypointDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void handleReportGlobalWaypointAction(ReportGlobalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData);


	/// SlaveHandlerInterface Methods
	void control_allowed(std::string service_uri, JausAddress component, unsigned char authority);
	void enable_monitoring_only(std::string service_uri, JausAddress component);
	void access_deactivated(std::string service_uri, JausAddress component);
	/// Guard Methods



	GlobalWaypointDriverClient_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;

	JausAddress p_control_addr;
	ros::NodeHandle p_nh;
	ros::NodeHandle p_pnh;
	message_filters::Subscriber<nav_msgs::Path> p_sub_path;
	tf::MessageFilter<nav_msgs::Path> * tf_filter_path;
	message_filters::Subscriber<geometry_msgs::PoseStamped> p_sub_pose;
	tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_pose;
//	ros::Subscriber p_sub_path;
//	ros::Subscriber p_sub_pose;
	ros::Subscriber p_sub_speed;
	ros::Publisher p_pub_path;
	float p_travel_speed;
	float p_wp_tolerance;
	std::string p_tf_frame_world;
	std::string p_utm_zone;
	QueryGlobalWaypoint p_query_global_waypoint_msg;
	tf::TransformListener tfListener;

	void pCmdPath(const nav_msgs::Path::ConstPtr& msg);
	void pCmdPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pCmdSpeed(const std_msgs::Float32::ConstPtr& msg);
	void pHandleReportGlobalWaypoint(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata);

};

};

#endif // GLOBALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H

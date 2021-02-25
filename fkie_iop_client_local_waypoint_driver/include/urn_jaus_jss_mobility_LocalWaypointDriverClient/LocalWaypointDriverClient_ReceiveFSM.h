

#ifndef LOCALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H
#define LOCALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_mobility_LocalWaypointDriverClient/Messages/MessageSet.h"
#include "urn_jaus_jss_mobility_LocalWaypointDriverClient/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "LocalWaypointDriverClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include "message_filters/subscriber.h"
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_mobility_LocalWaypointDriverClient
{

class DllExport LocalWaypointDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	LocalWaypointDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~LocalWaypointDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
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



	LocalWaypointDriverClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	JausAddress p_remote_addr;
	iop::Timer p_query_timer;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr p_sub_path;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr p_sub_pose;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr p_sub_speed;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_path;
	float p_travel_speed;
	float p_wp_tolerance;
	std::string p_tf_frame_robot;
	QueryLocalWaypoint p_query_local_waypoint_msg;
	std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> p_tf_listener;
	bool p_has_access;
	double p_hz;

	void pCmdPath(const nav_msgs::msg::Path::SharedPtr msg);
	void pCmdPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void pCmdSpeed(const std_msgs::msg::Float32::SharedPtr msg);
	void pQueryCallback();
};

}

#endif // GLOBALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H

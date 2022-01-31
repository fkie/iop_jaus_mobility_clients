

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

#include "urn_jaus_jss_core_ManagementClient/ManagementClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControlClient/AccessControlClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_EventsClient/EventsClient_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "GlobalWaypointDriverClient_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geographic_msgs/msg/geo_path.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "message_filters/subscriber.h"
#include <fkie_iop_ocu_slavelib/SlaveHandlerInterface.h>
#include <fkie_iop_events/EventHandlerInterface.h>

namespace urn_jaus_jss_mobility_GlobalWaypointDriverClient
{

class DllExport GlobalWaypointDriverClient_ReceiveFSM : public JTS::StateMachine, public iop::ocu::SlaveHandlerInterface, public iop::EventHandlerInterface
{
public:
	GlobalWaypointDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~GlobalWaypointDriverClient_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void handleReportGlobalWaypointAction(ReportGlobalWaypoint msg, Receive::Body::ReceiveRec transportData);
	virtual void handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData);


	/// EventHandlerInterface Methods
	void event(JausAddress reporter, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata);

	/// SlaveHandlerInterface Methods
	void register_events(JausAddress remote_addr, double hz);
	void unregister_events(JausAddress remote_addr);
	void send_query(JausAddress remote_addr);
	void stop_query(JausAddress remote_addr);
	/// Guard Methods



	GlobalWaypointDriverClient_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM;
	urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM;
	urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr p_sub_path;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr p_sub_pose;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr p_sub_speed;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr p_sub_fix;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr p_pub_path;
	rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr p_sub_geopose;
	float p_travel_speed;
	float p_wp_tolerance;
	std::string p_tf_frame_world;
	std::string p_utm_zone;
	QueryGlobalWaypoint p_query_global_waypoint_msg;
	std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> p_tf_listener;
	double p_hz;

	void pCmdGeoPoseStamped(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
	void pCmdPath(const nav_msgs::msg::Path::SharedPtr msg);
	void pCmdPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void pCmdSpeed(const std_msgs::msg::Float32::SharedPtr msg);
	void pCmdFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

}

#endif // GLOBALWAYPOINTDRIVERCLIENT_RECEIVEFSM_H


#include <iop_ocu_slavelib_fkie/Slave.h>
#include "urn_jaus_jss_mobility_GlobalWaypointDriverClient/GlobalWaypointDriverClient_ReceiveFSM.h"

#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>



using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_GlobalWaypointDriverClient
{



GlobalWaypointDriverClient_ReceiveFSM::GlobalWaypointDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalWaypointDriverClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	tf_filter_path = NULL;
	tf_filter_pose = NULL;
	p_pnh = ros::NodeHandle("~");
	p_travel_speed = 0.0;
	p_tf_frame_world = "/world";
	p_utm_zone = "32U";
	p_wp_tolerance = 1.0;
}



GlobalWaypointDriverClient_ReceiveFSM::~GlobalWaypointDriverClient_ReceiveFSM()
{
	delete context;
}

void GlobalWaypointDriverClient_ReceiveFSM::setupNotifications()
{
	pManagementClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	pManagementClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalWaypointDriverClient_ReceiveFSM_Receiving_Ready", "ManagementClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving_Ready", "GlobalWaypointDriverClient_ReceiveFSM");
	registerNotification("Receiving", pManagementClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ManagementClient_ReceiveFSM_Receiving", "GlobalWaypointDriverClient_ReceiveFSM");

	p_pnh.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	ROS_INFO_NAMED("GlobalWaypointDriverClient", "  tf_frame_world: %s", p_tf_frame_world.c_str());
	p_pnh.param("utm_zone", p_utm_zone, p_utm_zone);
	ROS_INFO_NAMED("GlobalWaypointDriverClient", "  utm_zone: %s", p_utm_zone.c_str());
	p_pnh.param("waypoint_tolerance", p_wp_tolerance, p_wp_tolerance);
	//ROS_INFO_NAMED("GlobalWaypointDriverClient", "  waypoint_tolerance: %.2f", p_wp_tolerance);
	//create ROS subscriber
	p_sub_path.subscribe(p_nh, "cmd_path", 10);
	tf_filter_path = new tf::MessageFilter<nav_msgs::Path>(p_sub_path, tfListener, p_tf_frame_world, 10);
	tf_filter_path->registerCallback( boost::bind(&GlobalWaypointDriverClient_ReceiveFSM::pCmdPath, this, _1) );
	p_sub_pose.subscribe(p_nh, "cmd_pose", 10);
	tf_filter_pose = new tf::MessageFilter<geometry_msgs::PoseStamped>(p_sub_pose, tfListener, p_tf_frame_world, 10);
	tf_filter_pose->registerCallback( boost::bind(&GlobalWaypointDriverClient_ReceiveFSM::pCmdPose, this, _1) );

	p_sub_speed = p_nh.subscribe<std_msgs::Float32>("cmd_speed", 1, &GlobalWaypointDriverClient_ReceiveFSM::pCmdSpeed, this);
	p_pub_path = p_nh.advertise<nav_msgs::Path>("global_waypoints", 5, true);
	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:GlobalWaypointDriver", 1, 0);
}

void GlobalWaypointDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:GlobalWaypointDriver") == 0) {
		p_control_addr = component;
		ROS_INFO_NAMED("GlobalWaypointDriverClient", "create event to get global waypoints from %d.%d.%d",
				component.getSubsystemID(), component.getNodeID(), component.getComponentID());
		pEventsClient_ReceiveFSM->create_event(&GlobalWaypointDriverClient_ReceiveFSM::pHandleReportGlobalWaypoint, this, component, p_query_global_waypoint_msg, 10.0, 1);
		sendJausMessage(p_query_global_waypoint_msg, component);
	} else {
		ROS_WARN_STREAM("[GlobalWaypointDriverClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	ROS_INFO_NAMED("GlobalWaypointDriverClient", "create monitor event to get global waypoint from %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pEventsClient_ReceiveFSM->create_event(&GlobalWaypointDriverClient_ReceiveFSM::pHandleReportGlobalWaypoint, this, component, p_query_global_waypoint_msg, 10.0, 1);
	sendJausMessage(p_query_global_waypoint_msg, component);
}

void GlobalWaypointDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_control_addr = JausAddress(0);
	ROS_INFO_NAMED("GlobalWaypointDriverClient", "cancel event for global waypoint by %d.%d.%d",
			component.getSubsystemID(), component.getNodeID(), component.getComponentID());
	pEventsClient_ReceiveFSM->cancel_event(component, p_query_global_waypoint_msg);
}

void GlobalWaypointDriverClient_ReceiveFSM::pHandleReportGlobalWaypoint(JausAddress &sender, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportGlobalWaypoint report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	handleReportGlobalWaypointAction(report, transport_data);
}


void GlobalWaypointDriverClient_ReceiveFSM::handleReportGlobalWaypointAction(ReportGlobalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	double lat, lon, alt = 0.0;
	double roll, pitch, yaw = 0.0;
	ReportGlobalWaypoint::Body::GlobalWaypointRec *wprec = msg.getBody()->getGlobalWaypointRec();
	lat = wprec->getLatitude();
	lon = wprec->getLongitude();
	if (wprec->isAltitudeValid()) {
		alt = wprec->getAltitude();
	}
	if (wprec->isRollValid()) {
		roll = wprec->getRoll();
	}
	if (wprec->isPitchValid()) {
		pitch = wprec->getPitch();
	}
	if (wprec->isYawValid()) {
		yaw = wprec->getYaw();
	}
	if (wprec->isWaypointToleranceValid()) {
	}

	ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "currentWaypointAction from %d.%d.%d - lat: %.2f, lon: %.2f", subsystem_id, node_id, component_id, lat, lon);
	ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "    alt: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", alt, roll, pitch, yaw);

	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_world;

	if (lat != 0.0 && lon != 0.0) {
		double northing, easting;
		std::string zone;
		gps_common::LLtoUTM(lat, lon, northing, easting, zone);
		tf::Quaternion quat;
		quat.setRPY(roll, pitch, yaw);

		geometry_msgs::PoseStamped pose;
		pose.header = path.header;
		pose.pose.position.x = easting;
		pose.pose.position.y = northing;
		pose.pose.position.z = alt;
		pose.pose.orientation.x = quat.x();
		pose.pose.orientation.y = quat.y();
		pose.pose.orientation.z = quat.z();
		pose.pose.orientation.w = quat.w();
		path.poses.push_back(pose);
	}

	this->p_pub_path.publish(path);
}

void GlobalWaypointDriverClient_ReceiveFSM::handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	uint16_t subsystem_id = transportData.getSourceID()->getSubsystemID();
	uint8_t node_id = transportData.getSourceID()->getNodeID();
	uint8_t component_id = transportData.getSourceID()->getComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	double speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	ROS_DEBUG_NAMED("GlobalWaypointDriverClient", "currentReportTravelSpeedAction from %d.%d.%d, speed: %.2f", subsystem_id, node_id, component_id, speed);
}


void GlobalWaypointDriverClient_ReceiveFSM::pCmdPath(const nav_msgs::Path::ConstPtr& msg)
{
	if (p_control_addr.get() != 0) {
		SetGlobalWaypoint cmd;
		geometry_msgs::PointStamped point_out;
		for (unsigned int i = 0; i < msg->poses.size(); i++) {
			try {
				geometry_msgs::PoseStamped pose_in = msg->poses[i];
				if (pose_in.header.frame_id.empty()) {
					pose_in.header = msg->header;
				}
				geometry_msgs::PoseStamped pose_out;
				tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
				double lat, lon = 0;
				gps_common::UTMtoLL(pose_out.pose.position.y, pose_out.pose.position.x, p_utm_zone.c_str(), lat, lon);
				cmd.getBody()->getGlobalWaypointRec()->setLatitude(lat);
				cmd.getBody()->getGlobalWaypointRec()->setLongitude(lon);
				cmd.getBody()->getGlobalWaypointRec()->setAltitude(pose_out.pose.position.z);
				double roll, pitch, yaw;
				tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				if (!isnan(yaw)) {
					cmd.getBody()->getGlobalWaypointRec()->setRoll(roll);
					cmd.getBody()->getGlobalWaypointRec()->setPitch(pitch);
					cmd.getBody()->getGlobalWaypointRec()->setYaw(yaw);
				}
			} catch (tf::TransformException &ex) {
				printf ("Failure %s\n", ex.what()); //Print exception which was caught
			}
		}
		ROS_INFO_NAMED("GlobalWaypointDriverClient", "send Waypoint from Path [lat: %.2f, lon: %.2f] to %d.%d.%d",
				cmd.getBody()->getGlobalWaypointRec()->getLatitude(), cmd.getBody()->getGlobalWaypointRec()->getLongitude(),
				p_control_addr.getSubsystemID(), p_control_addr.getNodeID(), p_control_addr.getComponentID());
		sendJausMessage(cmd, p_control_addr);
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pCmdPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (p_control_addr.get() != 0) {
		SetGlobalWaypoint cmd;
		geometry_msgs::PointStamped point_out;
		try {
			geometry_msgs::PoseStamped pose_in = *msg;
			geometry_msgs::PoseStamped pose_out;
			tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
			double lat, lon = 0;
			gps_common::UTMtoLL(pose_out.pose.position.y, pose_out.pose.position.x, p_utm_zone.c_str(), lat, lon);
			cmd.getBody()->getGlobalWaypointRec()->setLatitude(lat);
			cmd.getBody()->getGlobalWaypointRec()->setLongitude(lon);
			cmd.getBody()->getGlobalWaypointRec()->setAltitude(pose_out.pose.position.z);
			double roll, pitch, yaw;
			tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				cmd.getBody()->getGlobalWaypointRec()->setRoll(roll);
				cmd.getBody()->getGlobalWaypointRec()->setPitch(pitch);
				cmd.getBody()->getGlobalWaypointRec()->setYaw(yaw);
			}
		} catch (tf::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
		ROS_INFO_NAMED("GlobalWaypointDriverClient", "send Waypoint from Pose [lat: %.2f, lon: %.2f] to %d.%d.%d",
				cmd.getBody()->getGlobalWaypointRec()->getLatitude(), cmd.getBody()->getGlobalWaypointRec()->getLongitude(),
				p_control_addr.getSubsystemID(), p_control_addr.getNodeID(), p_control_addr.getComponentID());
		sendJausMessage(cmd, p_control_addr);
	}
}

void GlobalWaypointDriverClient_ReceiveFSM::pCmdSpeed(const std_msgs::Float32::ConstPtr& msg)
{
	if (p_control_addr.get() != 0) {
		SetTravelSpeed cmd;
		cmd.getBody()->getTravelSpeedRec()->setSpeed(msg->data);
		sendJausMessage(cmd, p_control_addr);
	}
}




};



#include "urn_jaus_jss_mobility_GlobalWaypointListDriverClient/GlobalWaypointListDriverClient_ReceiveFSM.h"

#include <iop_ocu_slavelib_fkie/Slave.h>
#include <iop_component_fkie/iop_config.h>
#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_GlobalWaypointListDriverClient
{



GlobalWaypointListDriverClient_ReceiveFSM::GlobalWaypointListDriverClient_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_ListManagerClient::ListManagerClient_ReceiveFSM* pListManagerClient_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new GlobalWaypointListDriverClient_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	this->pListManagerClient_ReceiveFSM = pListManagerClient_ReceiveFSM;
	p_travel_speed = 1.0;
	p_tf_frame_world = "/world";
	p_utm_zone = "32U";
	p_wp_tolerance = 1.0;
	p_has_access = false;
	p_hz = 0.0;
	p_new_rospath_received = false;
}



GlobalWaypointListDriverClient_ReceiveFSM::~GlobalWaypointListDriverClient_ReceiveFSM()
{
	delete context;
}

void GlobalWaypointListDriverClient_ReceiveFSM::setupNotifications()
{
	pListManagerClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_GlobalWaypointListDriverClient_ReceiveFSM_Receiving_Ready", "ListManagerClient_ReceiveFSM");
	pListManagerClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_GlobalWaypointListDriverClient_ReceiveFSM_Receiving_Ready", "ListManagerClient_ReceiveFSM");
	registerNotification("Receiving_Receiving_Ready", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving", "GlobalWaypointListDriverClient_ReceiveFSM");
	registerNotification("Receiving_Receiving", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving", "GlobalWaypointListDriverClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving_Ready", "GlobalWaypointListDriverClient_ReceiveFSM");
	registerNotification("Receiving", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving", "GlobalWaypointListDriverClient_ReceiveFSM");
	pListManagerClient_ReceiveFSM->add_state_handler(&GlobalWaypointListDriverClient_ReceiveFSM::pListState, this);
	iop::Config cfg("~GlobalWaypointListDriverClient");
	cfg.param("travel_speed", p_travel_speed, p_travel_speed);
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("utm_zone", p_utm_zone, p_utm_zone);
	cfg.param("waypoint_tolerance", p_wp_tolerance, p_wp_tolerance);
	cfg.param("hz", p_hz, p_hz, false, false);
	//ROS_INFO_NAMED("GlobalWaypointDriverClient", "  waypoint_tolerance: %.2f", p_wp_tolerance);
	//create ROS subscriber
	p_sub_path = cfg.subscribe<nav_msgs::Path>("cmd_global_path", 1, &GlobalWaypointListDriverClient_ReceiveFSM::pCmdPath, this);
	p_sub_speed = cfg.subscribe<std_msgs::Float32>("cmd_speed", 1, &GlobalWaypointListDriverClient_ReceiveFSM::pCmdSpeed, this);
	p_pub_path = cfg.advertise<nav_msgs::Path>("global_waypoint", 5, true);
	// initialize the control layer, which handles the access control staff
	Slave &slave = Slave::get_instance(*(jausRouter->getJausAddress()));
	slave.add_supported_service(*this, "urn:jaus:jss:mobility:GlobalWaypointListDriver", 1, 0);
}

void GlobalWaypointListDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	if (service_uri.compare("urn:jaus:jss:mobility:GlobalWaypointListDriver") == 0) {
		p_remote_addr = component;
		p_has_access = true;
		pListManagerClient_ReceiveFSM->set_remote(p_remote_addr);
	} else {
		ROS_WARN_STREAM("[GlobalWaypointDriverListClient] unexpected control allowed for " << service_uri << " received, ignored!");
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::enable_monitoring_only(std::string service_uri, JausAddress component)
{
	p_remote_addr = component;
}

void GlobalWaypointListDriverClient_ReceiveFSM::access_deactivated(std::string service_uri, JausAddress component)
{
	p_has_access = false;
	p_remote_addr = JausAddress(0);
	pListManagerClient_ReceiveFSM->release_remote();
}

void GlobalWaypointListDriverClient_ReceiveFSM::create_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		if (p_hz > 0) {
			ROS_INFO_NAMED("GlobalWaypointListDriverClient", "create QUERY timer to get global waypoints from %s with %.2fHz", component.str().c_str(), p_hz);
			p_query_timer = p_nh.createTimer(ros::Duration(1.0 / p_hz), &GlobalWaypointListDriverClient_ReceiveFSM::pQueryCallback, this);
		} else {
			ROS_WARN_NAMED("GlobalWaypointListDriverClient", "invalid hz %.2f for QUERY timer to get global waypoints from %s", p_hz, component.str().c_str());
		}
	} else {
		ROS_INFO_NAMED("GlobalWaypointListDriverClient", "create EVENT to get global waypoints from %s with %.2fHz", component.str().c_str(), p_hz);
		pEventsClient_ReceiveFSM->create_event(*this, component, p_query_global_waypoint_msg, p_hz);
		sendJausMessage(p_query_global_waypoint_msg, component);
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::cancel_events(std::string service_uri, JausAddress component, bool by_query)
{
	if (by_query) {
		p_query_timer.stop();
	} else {
		ROS_INFO_NAMED("GlobalWaypointListDriverClient", "cancel EVENT for global waypoints by %s", component.str().c_str());
		pEventsClient_ReceiveFSM->cancel_event(*this, component, p_query_global_waypoint_msg);
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::pQueryCallback(const ros::TimerEvent& event)
{
	if (p_remote_addr.get() != 0) {
		sendJausMessage(p_query_global_waypoint_msg, p_remote_addr);
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportGlobalWaypoint report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	try {
		handleReportGlobalWaypointAction(report, transport_data);
	} catch (std::exception &e) {
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::handleReportActiveElementAction(ReportActiveElement msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void GlobalWaypointListDriverClient_ReceiveFSM::handleReportGlobalWaypointAction(ReportGlobalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
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

	ROS_DEBUG_NAMED("GlobalWaypointListDriverClient", "currentWaypointAction from %s - lat: %.2f, lon: %.2f", sender.str().c_str(), lat, lon);
	ROS_DEBUG_NAMED("GlobalWaypointListDriverClient", "    alt: %.2f, roll: %.2f, pitch: %.2f, yaw: %.2f", alt, roll, pitch, yaw);

	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = this->p_tf_frame_world;

	if (lat > -90.0 && lon > -180.0 && lat < 90.0 && lon < 180.0) {
		double northing, easting;
		std::string zone;
		gps_common::LLtoUTM(lat, lon, northing, easting, zone);
		tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

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
	// perhaps there is also GlobalWaypointDriverClient and it can handle this message
	// throw std::exception();
}

void GlobalWaypointListDriverClient_ReceiveFSM::handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	ROS_DEBUG_NAMED("GlobalWaypointListDriverClient", "currentReportTravelSpeedAction from %s, speed: %.2f", sender.str().c_str(), p_travel_speed);
	// perhaps there is also GlobalWaypointDriverClient and it can handle this message
	throw std::exception();
}

void GlobalWaypointListDriverClient_ReceiveFSM::pCmdPath(const nav_msgs::Path::ConstPtr& msg)
{
	if (p_has_access) {
		pListManagerClient_ReceiveFSM->clear();
		for (unsigned int i = 0; i < msg->poses.size(); i++) {
			try {
				SetGlobalWaypoint cmd;
				geometry_msgs::PoseStamped pose_in = msg->poses[i];
				if (pose_in.header.frame_id.empty()) {
					pose_in.header = msg->header;
				}
				tfListener.waitForTransform(p_tf_frame_world, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
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
				urn_jaus_jss_core_ListManagerClient::SetElement::Body::SetElementSeq::ElementList::ElementRec rec;
				unsigned char buf[cmd.getSize()];
				cmd.encode(buf);
				rec.getElementData()->set(0, cmd.getSize(), buf);
				pListManagerClient_ReceiveFSM->push_back(rec);
			} catch (tf::TransformException &ex) {
				printf ("Failure %s\n", ex.what()); //Print exception which was caught
			}
		}
		// after all points are transfered to the driver, we will be informed by list manager. After this we send the ExecuteList command.
		p_new_rospath_received = true;
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::pCmdSpeed(const std_msgs::Float32::ConstPtr& msg)
{
	if (p_has_access) {
		p_travel_speed = msg->data;
		ROS_DEBUG_NAMED("GlobalWaypointListDriverClient", "change travel speed to %.2f", p_travel_speed);
		ExecuteList cmd;
		cmd.getBody()->getExecuteListRec()->setElementUID(65535);
		cmd.getBody()->getExecuteListRec()->setSpeed(p_travel_speed);
		sendJausMessage(cmd, p_remote_addr);
	}
}

void GlobalWaypointListDriverClient_ReceiveFSM::pListState(bool success, unsigned int count)
{
	if (success && count > 0) {
		if (p_new_rospath_received) {
			ROS_INFO_NAMED("GlobalWaypointListDriverClient", "execute list with %d points with speed %.2f on %s", count, p_travel_speed, p_remote_addr.str().c_str());
			ExecuteList cmd;
			cmd.getBody()->getExecuteListRec()->setElementUID(0);
			cmd.getBody()->getExecuteListRec()->setSpeed(p_travel_speed);
			sendJausMessage(cmd, p_remote_addr);
		}
	} else if (!success) {
		ROS_INFO_NAMED("GlobalWaypointListDriverClient", "errors while  transfer points occurred, stop and clear all points");
		pListManagerClient_ReceiveFSM->clear();
		ExecuteList cmd;
		cmd.getBody()->getExecuteListRec()->setElementUID(65535);
		cmd.getBody()->getExecuteListRec()->setSpeed(0);
		sendJausMessage(cmd, p_remote_addr);
	}
	p_new_rospath_received = false;
}

};

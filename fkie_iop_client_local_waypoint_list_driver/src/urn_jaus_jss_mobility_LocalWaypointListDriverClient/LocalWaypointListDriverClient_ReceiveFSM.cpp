

#include "urn_jaus_jss_mobility_LocalWaypointListDriverClient/LocalWaypointListDriverClient_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>
#include <fkie_iop_component/gps_conversions.h>
#include <fkie_iop_ocu_slavelib/Slave.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace JTS;
using namespace iop::ocu;

namespace urn_jaus_jss_mobility_LocalWaypointListDriverClient
{



LocalWaypointListDriverClient_ReceiveFSM::LocalWaypointListDriverClient_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_ListManagerClient::ListManagerClient_ReceiveFSM* pListManagerClient_ReceiveFSM, urn_jaus_jss_core_ManagementClient::ManagementClient_ReceiveFSM* pManagementClient_ReceiveFSM, urn_jaus_jss_core_AccessControlClient::AccessControlClient_ReceiveFSM* pAccessControlClient_ReceiveFSM, urn_jaus_jss_core_EventsClient::EventsClient_ReceiveFSM* pEventsClient_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: SlaveHandlerInterface(cmp, "LocalWaypointListDriverClient", 1.0),
  logger(cmp->get_logger().get_child("LocalWaypointListDriverClient"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new LocalWaypointListDriverClient_ReceiveFSMContext(*this);

	this->pListManagerClient_ReceiveFSM = pListManagerClient_ReceiveFSM;
	this->pManagementClient_ReceiveFSM = pManagementClient_ReceiveFSM;
	this->pAccessControlClient_ReceiveFSM = pAccessControlClient_ReceiveFSM;
	this->pEventsClient_ReceiveFSM = pEventsClient_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_travel_speed = 1.0;
	p_tf_frame_robot = "base_link";
	p_wp_tolerance = 1.0;
	p_hz = 0.0;
	p_new_rospath_received = false;
}



LocalWaypointListDriverClient_ReceiveFSM::~LocalWaypointListDriverClient_ReceiveFSM()
{
	delete context;
}

void LocalWaypointListDriverClient_ReceiveFSM::setupNotifications()
{
	pListManagerClient_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_LocalWaypointListDriverClient_ReceiveFSM_Receiving_Ready", "ListManagerClient_ReceiveFSM");
	pListManagerClient_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_LocalWaypointListDriverClient_ReceiveFSM_Receiving_Ready", "ListManagerClient_ReceiveFSM");
	registerNotification("Receiving_Receiving_Ready", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving", "LocalWaypointListDriverClient_ReceiveFSM");
	registerNotification("Receiving_Receiving", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving", "LocalWaypointListDriverClient_ReceiveFSM");
	registerNotification("Receiving_Ready", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving_Ready", "LocalWaypointListDriverClient_ReceiveFSM");
	registerNotification("Receiving", pListManagerClient_ReceiveFSM->getHandler(), "InternalStateChange_To_ListManagerClient_ReceiveFSM_Receiving", "LocalWaypointListDriverClient_ReceiveFSM");
}


void LocalWaypointListDriverClient_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "LocalWaypointListDriverClient");
	pListManagerClient_ReceiveFSM->add_state_handler(&LocalWaypointListDriverClient_ReceiveFSM::pListState, this);
	cfg.declare_param<double>("travel_speed", p_travel_speed, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Initial travel speed used if a waypoint is set. This value can be changed by cmd_speed topic.",
		"Default: 1.0");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id of the robot.",
		"Default: 'base_link'");
	cfg.declare_param<double>("waypoint_tolerance", p_wp_tolerance, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"currently not used.",
		"Default: 1.0");
	cfg.declare_param<double>("hz", p_hz, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"Sets how often the reports are requested. If use_queries is True hz must be greather then 0. In this case each time a Query message is sent to get a report. If use_queries is False an event is created to get Reports. In this case 0 disables the rate and an event of type on_change will be created.",
		"Default: 0.0");
	cfg.param("travel_speed", p_travel_speed, p_travel_speed);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("waypoint_tolerance", p_wp_tolerance, p_wp_tolerance);
	cfg.param("hz", p_hz, p_hz, false);
	//create ROS subscriber
	p_sub_path = cfg.create_subscription<nav_msgs::msg::Path>("cmd_local_path", 1, std::bind(&LocalWaypointListDriverClient_ReceiveFSM::pCmdPath, this, std::placeholders::_1));
	p_sub_speed = cfg.create_subscription<std_msgs::msg::Float32>("cmd_speed", 1, std::bind(&LocalWaypointListDriverClient_ReceiveFSM::pCmdSpeed, this, std::placeholders::_1));
	p_pub_path = cfg.create_publisher<nav_msgs::msg::Path>("local_waypoint", 5);
	// initialize the control layer, which handles the access control staff
	this->set_rate(p_hz);
	this->set_supported_service(*this, "urn:jaus:jss:mobility:LocalWaypointListDriver", 1, 0);
	this->set_event_name("local waypoints");
	p_tf_buffer = std::make_unique<tf2_ros::Buffer>(cmp->get_clock());
	p_tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);
}

void LocalWaypointListDriverClient_ReceiveFSM::control_allowed(std::string service_uri, JausAddress component, unsigned char authority)
{
	SlaveHandlerInterface::control_allowed(service_uri, component, authority);
	if (service_uri.compare("urn:jaus:jss:mobility:LocalWaypointListDriver") == 0) {
		pListManagerClient_ReceiveFSM->set_remote(component);
	}
}

void LocalWaypointListDriverClient_ReceiveFSM::register_events(JausAddress remote_addr, double hz)
{
	pEventsClient_ReceiveFSM->create_event(*this, remote_addr, p_query_local_waypoint_msg, p_hz);
}

void LocalWaypointListDriverClient_ReceiveFSM::unregister_events(JausAddress remote_addr)
{
	pEventsClient_ReceiveFSM->cancel_event(*this, remote_addr, p_query_local_waypoint_msg);
	stop_query(remote_addr);
}

void LocalWaypointListDriverClient_ReceiveFSM::send_query(JausAddress remote_addr)
{
	sendJausMessage(p_query_local_waypoint_msg, remote_addr);
}

void LocalWaypointListDriverClient_ReceiveFSM::stop_query(JausAddress remote_addr)
{
	pListManagerClient_ReceiveFSM->release_remote();
}

void LocalWaypointListDriverClient_ReceiveFSM::event(JausAddress sender, unsigned short query_msg_id, unsigned int reportlen, const unsigned char* reportdata)
{
	ReportLocalWaypoint report;
	report.decode(reportdata);
	Receive::Body::ReceiveRec transport_data;
	transport_data.setSrcSubsystemID(sender.getSubsystemID());
	transport_data.setSrcNodeID(sender.getNodeID());
	transport_data.setSrcComponentID(sender.getComponentID());
	try {
		handleReportLocalWaypointAction(report, transport_data);
	} catch (std::exception &e) {
	}
}

void LocalWaypointListDriverClient_ReceiveFSM::handleReportActiveElementAction(ReportActiveElement msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void LocalWaypointListDriverClient_ReceiveFSM::handleReportLocalWaypointAction(ReportLocalWaypoint msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	double x, y, z = 0.0;
	double roll, pitch, yaw = 0.0;
	ReportLocalWaypoint::Body::LocalWaypointRec *wprec = msg.getBody()->getLocalWaypointRec();
	x = wprec->getX();
	y = wprec->getY();
	if (wprec->isZValid()) {
		z = wprec->getZ();
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

	RCLCPP_DEBUG(logger, "currentWaypointAction from %s - x: %.2f, y: %.2f, z: %.2f", sender.str().c_str(), x, y, z);
	RCLCPP_DEBUG(logger, "    roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);

	auto path = nav_msgs::msg::Path();
	path.header.stamp = cmp->now();
	path.header.frame_id = this->p_tf_frame_robot;

	tf2::Quaternion quat;
	quat.setRPY(roll, pitch, yaw);

	auto pose = geometry_msgs::msg::PoseStamped();
	pose.header = path.header;
	pose.pose.position.x = x;
	pose.pose.position.y = y;
	pose.pose.position.z = z;
	pose.pose.orientation.x = quat.x();
	pose.pose.orientation.y = quat.y();
	pose.pose.orientation.z = quat.z();
	pose.pose.orientation.w = quat.w();
	path.poses.push_back(pose);

	this->p_pub_path->publish(path);
	// perhaps there is also LocalWaypointDriverClient and it can handle this message
	// throw std::exception();
}

void LocalWaypointListDriverClient_ReceiveFSM::handleReportTravelSpeedAction(ReportTravelSpeed msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	p_travel_speed = msg.getBody()->getTravelSpeedRec()->getSpeed();
	RCLCPP_DEBUG(logger, "currentReportTravelSpeedAction from %s, speed: %.2f", sender.str().c_str(), p_travel_speed);
	// perhaps there is also LocalWaypointDriverClient and it can handle this message
	throw std::exception();
}

void LocalWaypointListDriverClient_ReceiveFSM::pCmdPath(const nav_msgs::msg::Path::SharedPtr msg)
{
	if (p_has_access) {
		pListManagerClient_ReceiveFSM->clear();
		for (unsigned int i = 0; i < msg->poses.size(); i++) {
			try {
				SetLocalWaypoint cmd;
				auto pose_in = msg->poses[i];
				if (pose_in.header.frame_id.empty()) {
					pose_in.header = msg->header;
				}
				auto pose_out = geometry_msgs::msg::PoseStamped();
				if (!pose_in.header.frame_id.empty() && !p_tf_frame_robot.empty()) {
					p_tf_buffer->lookupTransform(p_tf_frame_robot, pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration(0.3));
					p_tf_buffer->transform(pose_in, pose_out, p_tf_frame_robot);
				} else {
					pose_out = pose_in;
					if (p_tf_frame_robot.empty()) {
						RCLCPP_WARN(logger, "p_tf_frame_robot is empty, forward local pose without transform to %s", p_remote_addr.str().c_str());
					} else if (pose_in.header.frame_id.empty()) {
						RCLCPP_WARN(logger, "pose_in.header.frame_id is empty, forward local pose without transform to %s", p_remote_addr.str().c_str());
					}
				}
				cmd.getBody()->getLocalWaypointRec()->setX(pose_out.pose.position.x);
				cmd.getBody()->getLocalWaypointRec()->setY(pose_out.pose.position.y);
				cmd.getBody()->getLocalWaypointRec()->setZ(pose_out.pose.position.z);
				double roll, pitch, yaw;
				tf2::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
				tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
				if (!isnan(yaw)) {
					cmd.getBody()->getLocalWaypointRec()->setRoll(roll);
					cmd.getBody()->getLocalWaypointRec()->setPitch(pitch);
					cmd.getBody()->getLocalWaypointRec()->setYaw(yaw);
				}
				urn_jaus_jss_core_ListManagerClient::SetElement::Body::SetElementSeq::ElementList::ElementRec rec;
				unsigned char* buf = new unsigned char(cmd.getSize());
				cmd.encode(buf);
				rec.getElementData()->set(0, cmd.getSize(), buf);
				pListManagerClient_ReceiveFSM->push_back(rec);
				delete[] buf;
			} catch (tf2::TransformException &ex) {
				printf ("Failure %s\n", ex.what()); //Print exception which was caught
			}
		}
		pListManagerClient_ReceiveFSM->send_list();
		p_new_rospath_received = true;
		// after all points are transfered to the driver, we will be informed by list manager. After this we send the ExecuteList command.
	}
}

void LocalWaypointListDriverClient_ReceiveFSM::pCmdSpeed(const std_msgs::msg::Float32::SharedPtr msg)
{
	if (p_has_access) {
		p_travel_speed = msg->data;
		RCLCPP_DEBUG(logger, "change travel speed to %.2f", p_travel_speed);
		ExecuteList cmd;
		cmd.getBody()->getExecuteListRec()->setElementUID(65535);
		cmd.getBody()->getExecuteListRec()->setSpeed(p_travel_speed);
		sendJausMessage(cmd, p_remote_addr);
	}
}

void LocalWaypointListDriverClient_ReceiveFSM::pListState(bool success, unsigned int count)
{
	if (success && count > 0) {
		if (p_new_rospath_received) {
			RCLCPP_INFO(logger, "execute list with %d points with speed %.2f on %s", count, p_travel_speed, p_remote_addr.str().c_str());
			ExecuteList cmd;
			cmd.getBody()->getExecuteListRec()->setElementUID(0);
			cmd.getBody()->getExecuteListRec()->setSpeed(p_travel_speed);
			sendJausMessage(cmd, p_remote_addr);
		}
	} else if (!success) {
		RCLCPP_INFO(logger, "errors while  transfer points occurred, stop and clear all points");
		pListManagerClient_ReceiveFSM->clear();
		ExecuteList cmd;
		cmd.getBody()->getExecuteListRec()->setElementUID(65535);
		cmd.getBody()->getExecuteListRec()->setSpeed(0);
		sendJausMessage(cmd, p_remote_addr);
	}
	p_new_rospath_received = false;
}

}

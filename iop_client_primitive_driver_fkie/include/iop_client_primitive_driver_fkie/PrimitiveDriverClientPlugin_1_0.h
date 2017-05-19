/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#ifndef PRIMITIVEDRIVERCLIENTPLUGIN_1_0_H
#define PRIMITIVEDRIVERCLIENTPLUGIN_1_0_H

#include "urn_jaus_jss_mobility_PrimitiveDriverClient_1_0/PrimitiveDriverClientService.h"
#include "urn_jaus_jss_core_AccessControlClient_1_0/AccessControlClientService.h"
#include "urn_jaus_jss_core_EventsClient_1_0/EventsClientService.h"
#include "urn_jaus_jss_core_ManagementClient_1_0/ManagementClientService.h"
#include "urn_jaus_jss_core_Transport_1_0/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport PrimitiveDriverClientPlugin_1_0 : public PluginInterface
{
public:
	PrimitiveDriverClientPlugin_1_0();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);

protected:
	urn_jaus_jss_mobility_PrimitiveDriverClient_1_0::PrimitiveDriverClientService *p_my_service;
	urn_jaus_jss_core_ManagementClient_1_0::ManagementClientService *p_base_service;
	urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClientService *p_accesscontrol_service;
	urn_jaus_jss_core_EventsClient_1_0::EventsClientService *p_events_service;
	urn_jaus_jss_core_Transport_1_0::TransportService *p_transport_service;

};

};

#endif

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


#ifndef LOCALPOSESENSORCLIENTPLUGIN_1_0_H
#define LOCALPOSESENSORCLIENTPLUGIN_1_0_H

#include "urn_jaus_jss_mobility_LocalPoseSensorClient_1_0/LocalPoseSensorClientService.h"
#include "urn_jaus_jss_core_AccessControlClient_1_0/AccessControlClientService.h"
#include "urn_jaus_jss_core_EventsClient_1_0/EventsClientService.h"
#include "urn_jaus_jss_core_Transport_1_0/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport LocalPoseSensorClientPlugin_1_0 : public PluginInterface
{
public:
	LocalPoseSensorClientPlugin_1_0();

	JTS::Service* get_iop_service();
	const std::type_info & get_iop_service_type();
	const std::type_info & get_base_service_type();
	void create_jts_service(JTS::JausRouter* jaus_router);
	virtual ~LocalPoseSensorClientPlugin_1_0();

	jVariableLengthString get_service_uri() { return "urn:jaus:jss:mobility:LocalPoseSensorClient"; }
	jUnsignedByte get_version_number_major() { return 1; }
	jUnsignedByte get_version_number_minor() { return 0; }

protected:
	urn_jaus_jss_mobility_LocalPoseSensorClient_1_0::LocalPoseSensorClientService *p_my_service;
	urn_jaus_jss_core_AccessControlClient_1_0::AccessControlClientService *p_base_service;
	urn_jaus_jss_core_EventsClient_1_0::EventsClientService *p_events_service;
	urn_jaus_jss_core_Transport_1_0::TransportService *p_transport_service;

};

};

#endif

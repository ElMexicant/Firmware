/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "land.h"
#include "navigator.h"

extern orb_advert_t mavlink_log_pub;//rickyremove

Land::Land(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name)
{
	/* load initial params */
	updateParams();
}

Land::~Land()
{
}

void
Land::on_inactive()
{
}

void//rickyremove
Land::on_activation()
{
	// subscribe to relivant topics
	initialize_topics();
	_new_vehicleLocalPosition = false;
	_new_beaconPosition = false;

	// update topics for the first time
	update_topics();

	mavlink_and_console_log_info(&mavlink_log_pub, "[land] on_active");//rickyremove ctrl+d
	// set current mission item to Land
	set_land_item(&_mission_item, true);//rickyremove precland-->land
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_can_loiter_at_sp(false);

	pos_sp_triplet->current.position_valid = true;
	pos_sp_triplet->current.x = 0;
	pos_sp_triplet->current.y = 0;
	pos_sp_triplet->current.z = 0;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

	_navigator->set_position_setpoint_triplet_updated();
}

void//rickyremove
Land::on_active()
{
	update_topics();

	if (is_mission_item_reached() && !_navigator->get_mission_result()->finished) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[land] done");//rickyremove ctrl+d
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();
		set_idle_item(&_mission_item);

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);

		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
Land::initialize_topics()
{
	// subscribe to vehiclePosition and beaconPosition topics 
	_vehicleLocalPositionSub = orb_subscribe(ORB_ID(vehicle_local_position));
	_beaconPositionSub = orb_subscribe(ORB_ID(beacon_position));
	_distanceSub = orb_subscribe(ORB_ID(distance_sensor));
}

void
Land::update_topics()
{
	_new_vehicleLocalPosition = _orb_update(ORB_ID(vehicle_local_position), _vehicleLocalPositionSub, 			&_vehicleLocalPosition);
	_new_beaconPosition = _orb_update(ORB_ID(beacon_position), _beaconPositionSub, &_beacon_position);
	_new_distance = _orb_update(ORB_ID(distance_sensor), _distanceSub, &_ground_distance);
}

bool
Land::_orb_update(const struct orb_metadata *meta, int handle, void *buffer)
{
	bool newData = false;

	// check if there is new data to grab
	if (orb_check(handle, &newData) != OK) {
		return false;
	}

	if (!newData) {
		return false;
	}

	if (orb_copy(meta, handle, buffer) != OK) {
		return false;
	}

	return true;
}

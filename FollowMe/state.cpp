
#include "state.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;
extern mavlink_channel_t upstream_channel;
extern mavlink_channel_t downstream_channel;

void FMStateMachine::on_upstream_command_long(mavlink_command_long_t* pkt) {
  switch(pkt->command) {
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_MISSION_START:
      /* clear out FM control of vehicle */
      _on_user_override();
    break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      /* i guess do nothing? */
    break;
  }
}

void FMStateMachine::on_upstream_set_mode(mavlink_set_mode_t* pkt) {
  /* mode is set in pkt->custom_mode */
  _vehicle_mode = (int8_t) pkt->custom_mode;
  /* clear out FM control of vehicle */
  _on_user_override();
}

void FMStateMachine::on_downstream_heartbeat(mavlink_heartbeat_t* pkt) {
  /* if mode has changed from last set_mode, the user has triggered a change
   * via RC switch.
   * clear out FM control of vehicle */
  bool pktarmed = ((pkt->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0);
  int8_t pktmode = (int8_t) pkt->custom_mode;
  if ((pktarmed != _vehicle_armed) || (pktmode != _vehicle_mode)) {
    _on_user_override();
  }
  /* update heartbeat millis */
  _last_vehicle_hb_millis = hal.scheduler->millis();
  /* update local state */ 
  _vehicle_armed = pktarmed;
  _vehicle_mode = pktmode;
}

void FMStateMachine::on_downstream_global_position_int(mavlink_global_position_int_t* pkt) {
  /* Keep track of vehicle's latest lat, lon, altitude */
}

void FMStateMachine::on_button_activate() {
  if (_guiding) return;
  /* This action is allowed to swing the state to start guide mode. */
  if (_check_guide_valid()) {
    _set_guide_offset();
    _send_guide();
    _guiding = true;
    hal.console->println_P(PSTR("Button activated, entering guided mode"));
  } else {
    hal.console->println_P(PSTR("Button activated but insufficient conditions"
          "for entering guided mode"));
  }
}

void FMStateMachine::on_button_cancel() {
  if (!_guiding) return;
  _send_loiter();
  _guiding = false;
}

void FMStateMachine::on_loop(GPS* gps) {
  uint32_t now = hal.scheduler->millis();
  if (_last_run_millis + _loop_period < now) return;
  _last_run_millis = now;

  if (gps != NULL) {
    _update_local_gps(gps);
  }

  if (_guiding) {
    _send_guide();
  }
}

bool FMStateMachine::_check_guide_valid() {
  uint32_t now = hal.scheduler->millis();

  bool vehicle_gps_valid = (_vehicle_gps_fix == 3);
  bool vehicle_hb_valid = (now - _last_vehicle_hb_millis) < 2000;

  bool vehicle_mode_valid = _vehicle_armed 
                          && ( (_vehicle_mode == MODE_LOITER)
                             ||(_vehicle_mode == MODE_ALT_HOLD)
                             ||(_vehicle_mode == MODE_AUTO)
                             );

  return _local_gps_valid
      && vehicle_gps_valid
      && vehicle_hb_valid
      && vehicle_mode_valid;
}

void FMStateMachine::_update_local_gps(GPS* gps) {
  /* Cause an on_fault_cancel if when local gps has transitioned form 
   * valid to invalid. */
  if (_local_gps_valid && !(gps->status() == GPS::GPS_OK)) {
    _on_fault_cancel();
  } 

  _local_gps_valid = (gps->status() == GPS::GPS_OK);
  if (gps->new_data) {
    _local_gps_lat      = gps->latitude;
    _local_gps_lat      = gps->longitude;
    _local_gps_altitude = gps->altitude;
    gps->new_data = false;
  }
}

void FMStateMachine::_set_guide_offset() {
  hal.console->println_P(PSTR("Placeholder: Should set guide mode offset here."));
}

void FMStateMachine::_on_fault_cancel() {
  _send_loiter();
  _guiding = false;
}

void FMStateMachine::_on_user_override() {
  hal.console->println_P(PSTR("User GCS or RC override of FollowMe"));
  _guiding = false;
}

void FMStateMachine::_send_guide() {
  hal.console->println_P(PSTR("Placeholder: Send guide waypoint packet"));
}

void FMStateMachine::_send_loiter() {
  hal.console->println_P(PSTR("Placeholder: Send setmode loiter packet"));
}

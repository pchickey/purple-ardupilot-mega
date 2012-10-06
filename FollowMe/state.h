
#ifndef __FOLLOWME_STATE_H__
#define __FOLLOWME_STATE_H__

#include <GCS_MAVLink.h>
#include <AP_GPS.h>

#include "arducopter_defines.h"

class FMStateMachine {
public:
  FMStateMachine() :
    _last_run_millis(0),
    _loop_period(500),
    _last_vehicle_hb_millis(0),
    _vehicle_armed(false),
    _vehicle_mode(MODE_NUM_MODES),
    _vehicle_gps_fix(0),
    _vehicle_lat(0),
    _vehicle_lon(0),
    _vehicle_altitude(0)
  {}
  
  void on_downstream_heartbeat(mavlink_heartbeat_t *pkt);
  void on_downstream_global_position_int(mavlink_global_position_int_t* pkt);

  void on_upstream_command_long(mavlink_command_long_t *pkt);
  void on_upstream_set_mode(mavlink_set_mode_t* pkt);

  void on_loop(GPS* gps);

  void on_button_activate();
  void on_button_cancel();

private:
  /* _send_guide: Send a guide waypoint packet upstream. */
  void _send_guide();

  /* _send_loiter: Send a setmode loiter packet upstream. */
  void _send_loiter();

  /* Calculate whether we have sufficient conditions to enter guide mode. */
  bool _check_guide_valid();

  /* _update_local_gps: Get device's current GPS status and location. Called
   * periodically. Can activate _on_fault_cancel(); */
  void _update_local_gps(GPS* gps);

  void _on_user_override();
  void _on_fault_cancel();

  void _set_guide_offset();

  /* Set once a start guide mode packet has been sent.
   * Unset whenever we stop guiding. */
  bool _guiding;

  /* Scheduling the on_loop periodic updater. */
  uint32_t _last_run_millis;
  uint32_t _loop_period;
  uint32_t _last_vehicle_hb_millis;

  int8_t  _vehicle_mode;
  bool    _vehicle_armed;
  uint8_t _vehicle_gps_fix;
  int32_t _vehicle_lat;
  int32_t _vehicle_lon;
  int32_t _vehicle_altitude;

  /* If local gps becomes non-valid, and we're guiding, put the vehicle into
   * loiter. */
  bool _local_gps_valid;
  int32_t _local_gps_lat;
  int32_t _local_gps_lon;
  int32_t _local_gps_altitude;
};

#endif // __FOLLOWME_STATE_H__


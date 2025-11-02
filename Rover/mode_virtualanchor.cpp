// mode_virtualanchor.cpp
#include "Rover.h"

bool ModeVirtualAnchor::_enter()
{
    // Initialize state
    anchor_is_set = false;
    virtual_anchor_state = e_VIRTUAL_ANCHOR_STATE::IDLE;

    // Initialize waypoint navigation library
    g2.wp_nav.init();

    // Reset PID controller state
    reset_pid();

    // Stop the vehicle initially
    g2.motors.set_throttle(0.0f);

    gcs().send_text(MAV_SEVERITY_INFO, "Virtual Anchor mode entered");

    return true;
}

bool ModeVirtualAnchor::set_anchor_point(const Location &loc)
{
    // Convert location to NE frame
    if (!loc.get_vector_xy_from_origin_NE_m(anchor_point_NE)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Virtual Anchor: Failed to set anchor");
        return false;
    }
    
    anchor_location = loc;
    anchor_is_set = true;
    anchor_set_time_ms = AP_HAL::millis();
    
    float distance = get_distance_to_anchor();
    gcs().send_text(MAV_SEVERITY_INFO, "Virtual Anchor set %.1fm away", distance);
    
    return true;
}

bool ModeVirtualAnchor::set_anchor_and_navigate(const Location &loc)
{
    if (!set_anchor_point(loc)) {
        return false;
    }

    // Set waypoint navigation directly
    if (!g2.wp_nav.set_desired_location(anchor_location)) {
        return false;
    }

    virtual_anchor_state = e_VIRTUAL_ANCHOR_STATE::GOTO_ANCHOR;
    gcs().send_text(MAV_SEVERITY_INFO, "Virtual Anchor: navigating to anchor point");
    return true;
}

bool ModeVirtualAnchor::set_anchor_here()
{
    Location current_loc;
    if (!rover.ahrs.get_location(current_loc)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Virtual Anchor: No position available");
        return false;
    }
    
    return set_anchor_point(current_loc);
}

void ModeVirtualAnchor::update()
{
    switch (virtual_anchor_state)
    {
    case e_VIRTUAL_ANCHOR_STATE::IDLE:
        // No anchor set - stop motors and maintain current heading
        g2.motors.set_throttle(0.0f);
        calc_steering_to_heading(ahrs.yaw_sensor, 0.0f);
        break;
    
    case e_VIRTUAL_ANCHOR_STATE::GOTO_ANCHOR:
        // navigate to anchor point using wp_nav
        if (!g2.wp_nav.reached_destination()) {
            // update navigation controller
            navigate_to_waypoint();
        } else {
            // reached the anchor point, switch to anchored state
            virtual_anchor_state = e_VIRTUAL_ANCHOR_STATE::ANCHORED;
            reset_pid();  // Reset PID for clean start in anchored state
            gcs().send_text(MAV_SEVERITY_INFO, "Virtual Anchor: reached anchor point");
        }
        break;

    case e_VIRTUAL_ANCHOR_STATE::ANCHORED:
        calculate_anchor_control();
        break;
    }
}

void ModeVirtualAnchor::calculate_anchor_control()
{
    // Get current position
    Vector2p current_pos_NE;
    if (!rover.ahrs.get_relative_position_NE_origin(current_pos_NE)) {
        // No position - stop everything and maintain heading
        g2.motors.set_throttle(0.0f);
        calc_steering_to_heading(ahrs.yaw_sensor, 0.0f);
        reset_pid();  // Reset PID when position is lost
        return;
    }

    // Calculate distance and bearing to anchor
    Vector2p to_anchor = anchor_point_NE - current_pos_NE;
    float distance_to_anchor = to_anchor.length();
    float bearing_to_anchor_cd = degrees(atan2f(to_anchor.y, to_anchor.x)) * 100.0f;

    // Get parameters
    float rope_length = g2.virtual_anchor_rope_len;
    float tolerance = g2.virtual_anchor_tolerance;
    float max_speed = g2.virtual_anchor_speed;
    float min_thrust = g2.virtual_anchor_min_thrust;

    // PID gains
    float kp = g2.virtual_anchor_pid_p;
    float ki = g2.virtual_anchor_pid_i;
    float kd = g2.virtual_anchor_pid_d;
    float imax = g2.virtual_anchor_pid_imax;

    // Calculate distance error (positive = too far, negative = too close)
    float distance_error = distance_to_anchor - rope_length;

    // ALWAYS face the anchor point
    calc_steering_to_heading(bearing_to_anchor_cd, 0.0f);

    // Calculate dt for PID
    uint32_t now_ms = AP_HAL::millis();
    float dt = 0.1f;  // Default 100ms
    if (_pid_last_update_ms != 0) {
        dt = (now_ms - _pid_last_update_ms) * 0.001f;  // Convert to seconds
        dt = constrain_float(dt, 0.01f, 1.0f);  // Limit dt to reasonable range
    }
    _pid_last_update_ms = now_ms;

    // Three-zone hybrid control
    float desired_speed = 0.0f;

    if (fabsf(distance_error) > tolerance) {
        // ZONE 1 & 3: Outside tolerance - use full PID control

        // P term: Proportional to current error
        float p_term = kp * distance_error;

        // I term: Accumulate integral
        _pid_integrator += ki * distance_error * dt;
        // Anti-windup: clamp integrator
        _pid_integrator = constrain_float(_pid_integrator, -imax, imax);

        // D term: Rate of change of error
        float d_term = 0.0f;
        if (_pid_last_update_ms != 0 && dt > 0.001f) {
            float error_rate = (distance_error - _pid_last_error) / dt;
            d_term = kd * error_rate;
        }

        // Store error for next derivative calculation
        _pid_last_error = distance_error;

        // Calculate total PID output
        desired_speed = p_term + _pid_integrator + d_term;

        // Constrain to max speed
        desired_speed = constrain_float(desired_speed, -max_speed, max_speed);

        // Apply throttle with avoidance
        calc_throttle(desired_speed, true);

    } else {
        // ZONE 2: Within tolerance - minimum thrust for heading control only
        // This allows drift within the tolerance band (power efficient)
        // Essential for azimuth thrusters that need thrust to steer

        // Reset integrator when in deadband to prevent windup
        _pid_integrator = 0.0f;
        _pid_last_error = distance_error;  // Keep tracking error

        if (min_thrust > 0.01f) {
            // Apply minimum thrust for heading control (azimuth thrusters)
            calc_throttle(min_thrust, true);
        } else {
            // Traditional steering - no thrust needed
            g2.motors.set_throttle(0.0f);
        }
    }

    // Log status periodically
    if (now_ms - _last_log_ms > 1000) {
        gcs().send_text(MAV_SEVERITY_INFO, "VANC: dist=%.1fm err=%.2fm spd=%.2f I=%.2f",
                       distance_to_anchor, distance_error, desired_speed, _pid_integrator);
        _last_log_ms = now_ms;
    }
}

float ModeVirtualAnchor::get_distance_to_anchor() const
{
    if (!anchor_is_set) {
        return 0.0f;
    }
    
    Vector2p current_pos_NE;
    if (!rover.ahrs.get_relative_position_NE_origin(current_pos_NE)) {
        return 0.0f;
    }
    
    return (current_pos_NE - anchor_point_NE).length();
}

float ModeVirtualAnchor::get_bearing_to_anchor() const
{
    if (!anchor_is_set) {
        return 0.0f;
    }
    
    Vector2p current_pos_NE;
    if (!rover.ahrs.get_relative_position_NE_origin(current_pos_NE)) {
        return 0.0f;
    }
    
    Vector2p diff = anchor_point_NE - current_pos_NE;
    return atan2f(diff.y, diff.x);
}

bool ModeVirtualAnchor::get_desired_location(Location& destination) const
{
    if (!anchor_is_set) {
        return false;
    }

    destination = anchor_location;
    return true;
}

float ModeVirtualAnchor::wp_bearing() const
{
    return degrees(get_bearing_to_anchor());
}

float ModeVirtualAnchor::nav_bearing() const
{
    return degrees(get_bearing_to_anchor());
}

float ModeVirtualAnchor::crosstrack_error() const
{
    if (!anchor_is_set) {
        return 0.0f;
    }
    return get_distance_to_anchor() - g2.virtual_anchor_rope_len;
}

void ModeVirtualAnchor::reset_pid()
{
    _pid_integrator = 0.0f;
    _pid_last_error = 0.0f;
    _pid_last_update_ms = 0;
    _last_log_ms = 0;
}

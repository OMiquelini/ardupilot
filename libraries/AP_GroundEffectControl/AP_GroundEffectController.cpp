/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Code by Sebastian Quilter

#include <AP_HAL/AP_HAL.h>
#include "AP_GroundEffectController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if HAL_GROUND_EFFECT_ENABLED

constexpr uint32_t RESET_TIMEOUT_MICROS{1000000};

const AP_Param::GroupInfo GroundEffectController::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Is the ground effect controller available or not
    // @Description: Toggles the ground effect controller availability on and off
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, GroundEffectController, _ACTIVE,0 , AP_PARAM_FLAG_ENABLE),

    // @Param: _throttle_pid_P
    // @DisplayName: P gain
    // @Description: P gain. A 1 m/s error from desired airspeed changes throttle by this many percent.
    // @Range: 0.0 200.0
    // @User: Standard
    AP_SUBGROUPINFO(_throttle_pid, "_THR_", 2, GroundEffectController, PID),

    // @Param: _pitch_pid_P
    // @DisplayName: P gain
    // @Description: P gain. A 1 meter error from desired alt changes sets the vehicle pitch to this value.
    // @User: Standard
    AP_SUBGROUPINFO(_pitch_pid, "_PITCH_", 3, GroundEffectController, PID),

    // @Param: _THR_REF
    // @DisplayName: Ground Effect desired throttle (percentage)
    // @Description:
    // @Range: 0.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_THR_REF", 4, GroundEffectController, _THR_REF, 35.0),

    // @Param: _THR_MIN
    // @DisplayName: Ground Effect minimum throttle (percentage)
    // @Description:
    // @Range: 0.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_THR_MIN", 5, GroundEffectController, _THR_MIN, 20.0),

    // @Param: _THR_MAX
    // @DisplayName: Ground Effect maximum throttle (percentage)
    // @Description:
    // @Range: 0.0 100.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_THR_MAX", 6, GroundEffectController, _THR_MAX, 50.0),

    // @Param: _ALT_REF
    // @DisplayName: Ground Effect desired altitude (meters)
    // @Description:
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("_ALT_REF", 7, GroundEffectController, _ALT_REF, 0.45),

    // @Param: _CUTOFF_FRQ
    // @DisplayName: Rangefinder Complementary Filter Cutoff Frequency
    // @Description: Lower values will trust the rangefinder less.
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_CUTOFF_FRQ", 8, GroundEffectController, _CUTOFF_FREQ, 0.1, 0),

    // @Param: _LIM_ROLL
    // @DisplayName: Max roll angle (degrees)
    // @Description: Max roll allowed in auto mode while going towards a ground effect waypoint. 0 to disable.
    // @Range: 0.0 45.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_LIM_ROLL", 9, GroundEffectController, _LIM_ROLL, 10.0),

    // @Param: _AIRSPEED
    // @DisplayName: Airspeed Aimed
    // @Description: Desired airspeed during ground effect flight
    AP_GROUPINFO("_AIRSPEED", 10, GroundEffectController, _AIMED_AIRSPEED, 7.0),

    // @Param: _TURN
    // @DisplayName: Enable turn controll
    // @Description: Enable roll limits to controll turns in ground effect flight
    AP_GROUPINFO("_TURN", 11, GroundEffectController, _ENABLE_TURN, 0),

    // @Param: _WING_SPAN
    // @DisplayName: Wing span
    // @Description: Aircraft wing span to calculate max roll within ground effect flight to avoid touching the water
    AP_GROUPINFO("_WING_SPAN", 12, GroundEffectController, _WING_SPAN, 1),

    // @Param: _SPD_PARAM
    //@Description:
    AP_GROUPINFO("_SPD_PARAM", 13, GroundEffectController, _SPD_PARAM, 1),

    AP_GROUPINFO("_ENABLE_THR", 14, GroundEffectController, _ENABLE_THR, 0),

    // @Param; _VERT_SPD
    // @DisplayName: Vertical Speed (m/s)
    // @Description: Aimed vertical speed on landing
    AP_GROUPINFO("_VERT_SPD", 15, GroundEffectController, _VERT_SPD, 1.5),

    // @Param; _FLARE_ANG
    // @DisplayName: Flare angle in degrees * 100
    // @Description: Aimed flare angle on landing
    AP_GROUPINFO("_FLARE_ANG", 16, GroundEffectController, _FLARE_ANG, 650),

    AP_GROUPEND
};

bool GroundEffectController::user_request_enable(bool enable)
{
    if(enable){
        if(!_ACTIVE || !_rangefinder->has_orientation(ROTATION_PITCH_270)){
            _enabled = false;
            return false;
        }
    }
    _enabled = enable;
    return true;
}

void GroundEffectController::reset()
{
    _altFilter.set_cutoff_frequency(_CUTOFF_FREQ);
    _altFilter.reset();

    _pitch_pid.reset_I();
    _throttle_pid.reset_I();
    return;
}

int32_t GroundEffectController::get_auto_lim_roll_cd()
{
    if(_LIM_ROLL <= 0.0001f){
        return INT32_MAX;
    }
    return MIN(_LIM_ROLL, get_max_roll())*100;
}

float GroundEffectController::turn_correction()
{
    float correction=0;
    Vector3f pos_offset=_rangefinder->get_pos_offset_orient(ROTATION_PITCH_270);
    correction=(_rangefinder->distance_orient(ROTATION_PITCH_270)*_ahrs->get_rotation_body_to_ned().c.z)-pos_offset.y*_ahrs->sin_roll()-pos_offset.x*_ahrs->sin_pitch();
    return correction;
}

void GroundEffectController::altitude_adjustment(float ref)
{
    alt_adjust = (ref+(1-_ahrs->cos_roll()))*0.5;
    return;
}

float GroundEffectController::get_max_roll()
{
    return MAX(((safe_asin(turn_correction()/_WING_SPAN))*(180/3.141592))-1, 0);
}

int GroundEffectController::turn_limit_on()
{
    if(_ENABLE_TURN)
        return 1;
    else
        return 0;
}

void GroundEffectController::speed_adjustment(float ref)
{
    spd_aimed = _AIMED_AIRSPEED+ref*_SPD_PARAM;
    return;
}

bool GroundEffectController::throttle_ctrl_enabled()
{
    if(_ENABLE_THR)
        return true;
    else
        return false;
}

void GroundEffectController::cruise(float alt_error, float airspeed_error, float reading)
{
    _pitch = _pitch_pid.get_pid(alt_error);
    _throttle = _throttle_pid.get_pid(airspeed_error);
    _throttle = constrain_int16(_throttle, _THR_MIN, _THR_MAX);
    
    return;
}

void GroundEffectController::land_seq(float alt_error, float airspeed_error, float reading)
{
    if(reading>1)//caso a altitude atual seja maior que 1 metro, o comando de land é ignorado e o voo continua em cruise, até que a altitude diminua
    {
        spd_error_aux=0; //variável auxiliar para reduzir a velocidade alvo para cada iteração da função, aqui está sendo resetada para evitar que assuma valores muito altos
        alt_error_aux=0; //variável auxiliar para reduzir a altitude alvo para cada iteração da função, aqui está sendo resetada para evitar que assuma valores muito altos       
        cruise(alt_error, airspeed_error, reading);
    }
    else if(reading >=0.15 && reading <=1)//caso a altitude esteja entre 15 cm e 1 m, começa sequencia de pouso, diminuindo a altitude e velocidade alvo gradativamente
    {
        alt_error_aux+=0.005;//para cada iteração da função land_seq, a altitude alvo é diminuida em 0,5 cm, assim, totalizando 0,25 m/s
        spd_error_aux+=0.02;//para cada iteração da função, a velocidade alvo é diminuida em 0,02 m, totalizando 1m/s
        _pitch=_pitch_pid.get_pid(alt_error-alt_error_aux);
        _throttle = _throttle_pid.get_pid(airspeed_error+spd_error_aux);
        _throttle = constrain_int16(_throttle, _THR_MIN, _THR_MAX);
    }
    else//caso a altitude seja menor que 15 cm, executa o flare e diminui o throttle para o mínimo programado
    {
        alt_error_aux=0;//reset na variável auxiliar de altitude
        spd_error_aux=0;//reset na variável auxiliar de velocidade
        _pitch = _FLARE_ANG*100;//flare de acordo com o ângulo nos parâmetros
        _throttle = _THR_MIN;//throttle mínimo
    }
    
    return;
}

void GroundEffectController::update(bool land)
{
    uint32_t time = AP_HAL::micros();
    if(time - _last_time_called > RESET_TIMEOUT_MICROS){
        reset();
    }
    _last_time_called = time;

    if(_rangefinder->status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good)
    {
        if(_ENABLE_TURN)
        {
            _last_good_rangefinder_reading = turn_correction();
        }
        else
        {
            _last_good_rangefinder_reading = _rangefinder->distance_orient(ROTATION_PITCH_270);
        }
    }
    
    float alt_error, ahrs_negative_alt, airspeed_measured, airspeed_error, reading;
    if(_ahrs->airspeed_estimate(airspeed_measured))
        airspeed_error = spd_aimed - airspeed_measured;
    else
    {
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,"AIRSPEED RUIM");
    airspeed_error = 0;
    }

    // DCM altitude is not good. If EKF alt is not available, just use raw rangefinder data
    if(_ahrs->get_active_AHRS_type() > 0 && _ahrs->get_relative_position_D_origin(ahrs_negative_alt)){
        _altFilter.apply(_last_good_rangefinder_reading, -ahrs_negative_alt, time);
        reading=_altFilter.get();
        alt_error = _ALT_REF + alt_adjust - reading;
    } else {
        reading=_last_good_rangefinder_reading;
        alt_error = _ALT_REF + alt_adjust - reading;
    }

    //update pitch and throttle, land or cruise
    if(!land) {
        //takeoff and cruise
        cruise(alt_error, airspeed_error, reading);
    } else {
        //land
        land_seq(alt_error, airspeed_error, reading);
    }
    return;
}
GroundEffectController *GroundEffectController::_singleton;

#endif // HAL_GROUND_EFFECT_ENABLED

#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    //plane.update_fbwb_speed_height();
    if(plane.channel_throttle->in_trim_dz()){
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
    } else {
        float gnd_throttle=plane.g2.ground_effect_controller.get_throttle();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, gnd_throttle);
    }
    plane.nav_pitch_cd += plane.g2.ground_effect_controller.get_pitch();
}


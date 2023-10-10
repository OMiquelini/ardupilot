#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

bool message_gndef = false;
int cont=0;
void ModeGNDEF::update()
{    
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max_cd;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min_cd);
    }
    
#if HAL_GROUND_EFFECT_ENABLED         
    //ajuste de altura de referencia com input do piloto (175)
    RC_Channel *chan_alt = rc().find_channel_for_option(RC_Channel::AUX_FUNC::GNDEF_POT_ALT);
    float pot_alt = chan_alt->norm_input_ignore_trim();
    plane.g2.ground_effect_controller.altitude_adjustment(pot_alt);

    //ajuste de velocidade de referência com input do piloto (176)
    //RC_Channel *chan_spd = rc().find_channel_for_option(RC_Channel::AUX_FUNC::GNDEF_POT_SPD);
    //TODO: chave de velocidade ser a mesma de throttle
    float pot_spd = plane.channel_throttle->norm_input_ignore_trim();//chan_spd->norm_input_ignore_trim();
    plane.g2.ground_effect_controller.speed_adjustment(pot_spd);

    //ativar pouso em efeito solo (177)
    RC_Channel *chan_lnd = rc().find_channel_for_option(RC_Channel::AUX_FUNC::GNDEF_LAND);
    bool gndef_land = chan_lnd->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;

    if(gndef_land && !(plane.channel_pitch->in_trim_dz()))//TODO: se stick de pitch não estiver trimado, abortar pouso
    {
        plane.g2.ground_effect_controller.landing();
    }
    else
    {
        plane.g2.ground_effect_controller.update();
    }

    //Mensagem ao entrar em modo de efeito solo
    if (message_gndef==false)
    {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"Ground effect enabled");
        message_gndef=true;
    }
    else if(cont==500)
    {
        message_gndef=false;
        cont=0;
    }
    else
        cont++;

    // Se o controle estiver com throttle em 0, ignora o valor do controlador pid e escreve throttle = 0
    if(plane.channel_throttle->in_trim_dz()){
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
    } else {
        float gnd_throttle=plane.g2.ground_effect_controller.get_throttle();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, gnd_throttle);
    }

    plane.nav_pitch_cd += plane.g2.ground_effect_controller.get_pitch(); // Note that this stacks
#endif
plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
if (plane.fly_inverted()) {
    plane.nav_pitch_cd = -plane.nav_pitch_cd;
}
if (plane.failsafe.rc_failsafe && plane.g.fs_action_short == FS_ACTION_SHORT_FBWA) {
    // FBWA failsafe glide
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;
    SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::MIN);
}
}

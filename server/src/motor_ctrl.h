/*
  Copyright 2019 coyote009

  This file is part of coyote_racer.

  coyote_racer is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  coyote_racer is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with coyote_racer.  If not, see <http://www.gnu.org/licenses/>.

 */

#ifndef MOTOR_CTRL_H_
#define MOTOR_CTRL_H_

#define MC_CMD_IDLE 0
#define MC_CMD_POS  1
#define MC_CMD_NEG  2

#define MC_THROT_IDLE MC_CMD_IDLE
#define MC_THROT_FWD  MC_CMD_POS
#define MC_THROT_BWD  MC_CMD_NEG

#define MC_STEER_IDLE MC_CMD_IDLE
#define MC_STEER_RGT  MC_CMD_POS
#define MC_STEER_LFT  MC_CMD_NEG

struct motor_ctrl_data
{
    int enable_motor;
    int ports[4];

    pthread_t thread;
    pthread_mutex_t mutex;

    int exit_thread;
    int cmd_valid;
    int ctrl_cmd[4];
    int ctrl_new[4];
    int ctrl_cur[4];
};

int mc_init( motor_ctrl_data &mc, const char *config_file, int enable_motor );
void mc_fina( motor_ctrl_data &mc );
void mc_set( motor_ctrl_data &mc, float throttle_val, int throttle_state, float steering_val, int steering_state );
void mc_set_val( motor_ctrl_data &mc, float throttle_val, float steering_val );


#endif

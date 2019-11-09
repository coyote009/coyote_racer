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

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include "motor_ctrl.h"

static void *mc_thread_func( void *arg )
{
    const int wait_dir_change = 500;
    const int wait_cycle = 1000;
    const double time_to_stop = 0.5;
    
    motor_ctrl_data *mcp = (motor_ctrl_data *) arg;
    double tick_freq = cv::getTickFrequency();
    double time_begin = cv::getTickCount() / tick_freq;

    while( 1 )
    {
        int new_valid = 0;
        
        pthread_mutex_lock( &mcp->mutex );

        if( mcp->cmd_valid )
        {
            if( ( mcp->ctrl_cmd[0] && mcp->ctrl_cmd[1] ) ||
                ( mcp->ctrl_cmd[2] && mcp->ctrl_cmd[3] ) )
            {
                fprintf( stderr, "Illegal control settings:(%d,%d) (%d,%d)\n",
                         mcp->ctrl_cmd[0], mcp->ctrl_cmd[1],
                         mcp->ctrl_cmd[2], mcp->ctrl_cmd[3] );
            }
            else
            {
                for( int i = 0; i < 4; i++ )
                {
                    mcp->ctrl_new[i] = mcp->ctrl_cmd[i];
                }

                new_valid = 1;
            }

            mcp->cmd_valid = 0;
        }

        pthread_mutex_unlock( &mcp->mutex );

        int dir_change = 0;
        if( new_valid )
        {
            /*
              If motor direction should be changed
              set the currently running port to zero
             */
            if( mcp->ctrl_cur[0] && mcp->ctrl_new[1] )
            {
                gpioPWM( mcp->ports[0], 0 );
                mcp->ctrl_cur[0] = mcp->ctrl_new[0] = 0;
                dir_change = 1;
            }
            else if( mcp->ctrl_cur[1] && mcp->ctrl_new[0] )
            {
                gpioPWM( mcp->ports[1], 0 );
                mcp->ctrl_cur[1] = mcp->ctrl_new[1] = 0;
                dir_change = 1;
            }

            if( mcp->ctrl_cur[2] && mcp->ctrl_new[3] )
            {
                gpioPWM( mcp->ports[2], 0 );
                mcp->ctrl_cur[2] = mcp->ctrl_new[2] = 0;
                dir_change = 1;
            }
            else if( mcp->ctrl_cur[3] && mcp->ctrl_new[2] )
            {
                gpioPWM( mcp->ports[3], 0 );
                mcp->ctrl_cur[3] = mcp->ctrl_new[3] = 0;
                dir_change = 1;
            }

            if( dir_change )
            {
                usleep( wait_dir_change );
            }

            for( int i = 0; i < 4; i++ )
            {
                if( mcp->ctrl_new[i] != mcp->ctrl_cur[i] )
                {
                    gpioPWM( mcp->ports[i], mcp->ctrl_new[i] );
                    mcp->ctrl_cur[i] = mcp->ctrl_new[i];
                }
            }

            time_begin = cv::getTickCount() / tick_freq;
        }

        /*
        double time_cur = cv::getTickCount() / tick_freq;
        if( ( time_cur - time_begin ) > time_to_stop )
        {
            for( int i = 0; i < 4; i++ )
            {
                if( mcp->ctrl_cur[i] )
                {
                    mcp->ctrl_cur[i] = 0;
                    gpioPWM( mcp->ports[i], mcp->ctrl_cur[i] );
                }
            }
        }
        */

        if( mcp->exit_thread )
        {
            break;
        }

        if( dir_change )
        {
            usleep( wait_cycle - wait_dir_change );
        }
        else
        {
            usleep( wait_cycle );
        }
    }
}

int mc_init( motor_ctrl_data &mc, const char *config_file, int enable_motor )
{
    mc.enable_motor = enable_motor;

    if( enable_motor )
    {
        const int port_forward = 17;
        const int port_backward = 18;
        const int port_right = 27;
        const int port_left = 22;
        const int freq_thrtl = 50;
        const int freq_steer = 6;

        mc.ports[0] = port_forward;
        mc.ports[1] = port_backward;
        mc.ports[2] = port_right;
        mc.ports[3] = port_left;

        if( gpioCfgClock( 8, 0, 0 ) )
        {
            fprintf( stderr, "motor_ctrl: Failed to set sample rate\n" );
            return 1;
        }

        int ret = gpioInitialise();
        if( ret < 0 )
        {
            fprintf( stderr, "motor_ctrl: Failed to initialize\n" );
            return 1;
        }

        for( int i = 0; i < 4; i++ )
        {
            int freq = (i < 2) ? freq_thrtl : freq_steer;
            
            gpioSetPWMfrequency( mc.ports[i], freq );

            printf( "motor_ctrl: port(%d) range=%d real_range=%d freq=%d\n", i,
                    gpioGetPWMrange( mc.ports[i] ), gpioGetPWMrealRange( mc.ports[i] ),
                    gpioGetPWMfrequency( mc.ports[i] ) );

            gpioPWM( mc.ports[i], 0 );
        }

        mc.exit_thread = 0;
        mc.cmd_valid = 0;
        for( int i = 0; i < 4; i++ )
        {
            mc.ctrl_cur[i] = mc.ctrl_new[i] = 0;
        }
        
        pthread_mutex_init( &mc.mutex, NULL );
        pthread_create( &mc.thread, NULL, &mc_thread_func, &mc );
    }

    return 0;
}

void mc_fina( motor_ctrl_data &mc )
{
    if( mc.enable_motor )
    {
        mc.exit_thread = 1;
        pthread_join( mc.thread, NULL );
        gpioTerminate();
    }
}

static void get_pwm_val( int port_pos, int port_neg, float pwm_val, int state,
                         int &val_pos, int &val_neg )
{
    int val;

    val = 255 * pwm_val;
    val = (val < 0) ? 0 : val;
    val = (val > 255) ? 255 : val;

    switch( state )
    {
    case MC_CMD_IDLE:
        val_pos = val_neg = 0;
        break;

    case MC_CMD_POS:
        val_pos = val;
        val_neg = 0;
        break;
            
    case MC_CMD_NEG:
        val_pos = 0;
        val_neg = val;
        break;

    default:
        fprintf( stderr, "motor_ctrl: Invalid command\n" );
    }
}

void mc_set( motor_ctrl_data &mc, float throttle_val, int throttle_state,
             float steering_val, int steering_state )
{
    if( mc.enable_motor )
    {
        int ctrl_cmd[4];
        get_pwm_val( mc.ports[0], mc.ports[1], throttle_val, throttle_state,
                     ctrl_cmd[0], ctrl_cmd[1] );
        get_pwm_val( mc.ports[2], mc.ports[3], steering_val, steering_state,
                     ctrl_cmd[2], ctrl_cmd[3] );

        pthread_mutex_lock( &mc.mutex );

        for( int i = 0; i < 4; i++ )
        {
            mc.ctrl_cmd[i] = ctrl_cmd[i];
        }
        mc.cmd_valid = 1;

        pthread_mutex_unlock( &mc.mutex );
    }
}

void mc_set_val( motor_ctrl_data &mc, float throttle_val, float steering_val )
{
    if( mc.enable_motor )
    {
        const float eps = 0.01;

        float throttle_val_abs;
        int throttle_state;
        float steering_val_abs;
        int steering_state;
        
        if( throttle_val > eps )
        {
            throttle_val_abs = throttle_val;
            throttle_state = MC_THROT_FWD;
        }
        else if( throttle_val > -eps )
        {
            throttle_val_abs = 0.0f;
            throttle_state = MC_THROT_IDLE;
        }
        else
        {
            //throttle_val_abs = fabs( throttle_val );
            throttle_val_abs = -throttle_val;
            throttle_state = MC_THROT_BWD;
        }

        if( steering_val > eps )
        {
            steering_val_abs = steering_val;
            steering_state = MC_STEER_RGT;
        }
        else if( steering_val > -eps )
        {
            steering_val_abs = 0.0f;
            steering_state = MC_STEER_IDLE;
        }
        else
        {
            //steering_val_abs = fabs( steering_val );
            steering_val_abs = -steering_val;
            steering_state = MC_STEER_LFT;
        }

        mc_set( mc, throttle_val_abs, throttle_state, steering_val_abs, steering_state );
    }
}


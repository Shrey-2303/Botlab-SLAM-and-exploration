/**
 * This file is the main executable for the MBot firmware.
 */
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include "mbot.h"
#include "odometry.h"
#include <mbot/utils/utils.h>
#include "print_tables.h"
#include <mbot/defs/mbot_params.h>
#include <inttypes.h>

#define THETA "\u0398"
#pragma pack(1)

#ifndef MBOT_DRIVE_TYPE
#error "Please define a drive type for the bot"
#endif

// #define PRINT_MBOT_STATE

// #define CALIBRATE_GYRO

// Global
static uint64_t timestamp_offset = 0;
static uint64_t global_utime = 0;
static uint64_t global_pico_time = 0;
static bool global_comms_status = COMMS_ERROR; 
static int drive_mode = 0;
static bool running = false;
static mbot_params_t params;

//lowpass Globals
rc_filter_t gyro_lowpass;
rc_filter_t motor_vL_lowpass;
rc_filter_t motor_vR_lowpass;
rc_filter_t vx_meas_lowpass;
rc_filter_t wz_meas_lowpass;

// PID Globals
rc_filter_t pid_leftwheel;
rc_filter_t pid_rightwheel;
rc_filter_t pid_vx;
rc_filter_t pid_wz;

mbot_bhy_data_t mbot_imu_data;
mbot_bhy_config_t mbot_imu_config;

void print_mbot_params(const mbot_params_t* params) {
    printf("Robot Type: %d\n", params->robot_type);
    printf("Wheel Radius: %f\n", params->wheel_radius);
    printf("Wheel Base Radius: %f\n", params->wheel_base_radius);
    printf("Gear Ratio: %f\n", params->gear_ratio);
    printf("Encoder Resolution: %f\n", params->encoder_resolution);
    printf("Motor Left: %d\n", params->mot_left);
    printf("Motor Right: %d\n", params->mot_right);
    printf("Motor Back: %d\n", params->mot_back);
    printf("Motor Polarity: %d %d %d\n", params->motor_polarity[0], params->motor_polarity[1], params->motor_polarity[2]);
    printf("Encoder Polarity: %d %d %d\n", params->encoder_polarity[0], params->encoder_polarity[1], params->encoder_polarity[2]);
    printf("Positive Slope: %f %f %f\n", params->slope_pos[0], params->slope_pos[1], params->slope_pos[2]);
    printf("Positive Intercept: %f %f %f\n", params->itrcpt_pos[0], params->itrcpt_pos[1], params->itrcpt_pos[2]);
    printf("Negative Slope: %f %f %f\n", params->slope_neg[0], params->slope_neg[1], params->slope_neg[2]);
    printf("Negative Intercept: %f %f %f\n", params->itrcpt_neg[0], params->itrcpt_neg[1], params->itrcpt_neg[2]);
}

void register_topics()
{
    // Subscriptions
    comms_register_topic(MBOT_TIMESYNC, sizeof(serial_timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    comms_register_topic(MBOT_ODOMETRY_RESET,  sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, (MsgCb)&reset_odometry_cb);
    comms_register_topic(MBOT_ENCODERS_RESET, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, (MsgCb)&reset_encoders_cb);
    comms_register_topic(MBOT_MOTOR_PWM_CMD, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, (MsgCb)mbot_motor_vel_cmd_cb);
    comms_register_topic(MBOT_MOTOR_VEL_CMD, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, (MsgCb)mbot_motor_pwm_cmd_cb);
    comms_register_topic(MBOT_VEL_CMD, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, (MsgCb)mbot_vel_cmd_cb);

    // Published Topics
    comms_register_topic(MBOT_ODOMETRY, sizeof(serial_pose2D_t), (Deserialize)&pose2D_t_deserialize, (Serialize)&pose2D_t_serialize, NULL);
    comms_register_topic(MBOT_IMU, sizeof(serial_mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    comms_register_topic(MBOT_ENCODERS, sizeof(serial_mbot_encoders_t), (Deserialize)&mbot_encoders_t_deserialize, (Serialize)&mbot_encoders_t_serialize, NULL);
    comms_register_topic(MBOT_VEL, sizeof(serial_twist2D_t), (Deserialize)&twist2D_t_deserialize, (Serialize)&twist2D_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_VEL, sizeof(serial_mbot_motor_vel_t), (Deserialize)&mbot_motor_vel_t_deserialize, (Serialize)&mbot_motor_vel_t_serialize, NULL);
    comms_register_topic(MBOT_MOTOR_PWM, sizeof(serial_mbot_motor_pwm_t), (Deserialize)&mbot_motor_pwm_t_deserialize, (Serialize)&mbot_motor_pwm_t_serialize, NULL);

}

void timestamp_cb(serial_timestamp_t *msg)
{
    global_pico_time = to_us_since_boot(get_absolute_time());
    timestamp_offset = msg->utime - global_pico_time;
    global_comms_status = COMMS_OK;
}

void reset_encoders_cb(serial_mbot_encoders_t *msg)
{
    //memcpy(&encoders, msg, sizeof(serial_mbot_encoders_t));
    for(int i=0; i<3; i++){
        mbot_encoder_write(i, msg->ticks[i]);
    }
}

void reset_odometry_cb(serial_pose2D_t *msg)
{
    mbot_odometry.x = msg->x;
    mbot_odometry.y = msg->y;
    mbot_odometry.theta = msg->theta;
}

void mbot_vel_cmd_cb(serial_twist2D_t *msg)
{
    memcpy(&mbot_vel_cmd, msg, sizeof(serial_twist2D_t));
    drive_mode = MODE_MBOT_VEL;
}

void mbot_motor_vel_cmd_cb(serial_mbot_motor_vel_t *msg)
{
    memcpy(&mbot_motor_vel_cmd, msg, sizeof(serial_mbot_motor_vel_t));
    drive_mode = MODE_MOTOR_VEL_OL;
}

void mbot_motor_pwm_cmd_cb(serial_mbot_motor_pwm_t *msg)
{
    memcpy(&mbot_motor_pwm_cmd, msg, sizeof(serial_mbot_motor_pwm_t));
    drive_mode = MODE_MOTOR_PWM;
}

void mbot_calculate_motor_vel(serial_mbot_encoders_t encoders, serial_mbot_motor_vel_t *motor_vel){
    float conversion = (1.0 / params.gear_ratio) * (1.0 / params.encoder_resolution) * 1E6f * 2.0 * M_PI;
    motor_vel->velocity[params.mot_left] = params.encoder_polarity[params.mot_left] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_left];
    if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        motor_vel->velocity[params.mot_back] = params.encoder_polarity[params.mot_back] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_back];
    }
    motor_vel->velocity[params.mot_right] = params.encoder_polarity[params.mot_right] * (conversion / encoders.delta_time) * encoders.delta_ticks[params.mot_right];
}

void mbot_read_imu(serial_mbot_imu_t *imu){
    /**
     * filters for IMU
    */
    // double output = rc_filter_march(&gyro_lowpass, mbot_imu_data.gyro[2]);
    double output = mbot_imu_data.gyro[2];
    // printf("output: %f\n", output);
    // if (output < 0.01 && output > -0.01) output = 0;

    imu->utime = global_utime;
    imu->gyro[0] = mbot_imu_data.gyro[0];
    imu->gyro[1] = mbot_imu_data.gyro[1];
    imu->gyro[2] = output;
    imu->accel[0] = mbot_imu_data.accel[0];
    imu->accel[1] = mbot_imu_data.accel[1];
    imu->accel[2] = mbot_imu_data.accel[2];
    imu->mag[0] = mbot_imu_data.mag[0];
    imu->mag[1] = mbot_imu_data.mag[1];
    imu->mag[2] = mbot_imu_data.mag[2];
    imu->angles_rpy[0] = mbot_imu_data.rpy[0];
    imu->angles_rpy[1] = mbot_imu_data.rpy[1];
    imu->angles_rpy[2] = mbot_imu_data.rpy[2];
    imu->angles_quat[0] = mbot_imu_data.quat[0];
    imu->angles_quat[1] = mbot_imu_data.quat[1];
    imu->angles_quat[2] = mbot_imu_data.quat[2];
    imu->angles_quat[3] = mbot_imu_data.quat[3];   
}

void mbot_read_encoders(serial_mbot_encoders_t* encoders){
    int64_t delta_time = global_utime - encoders->utime;
    encoders->utime = global_utime;
    encoders->delta_time = delta_time;

    encoders->ticks[params.mot_right] = mbot_encoder_read_count(params.mot_right);
    encoders->delta_ticks[params.mot_right] = mbot_encoder_read_delta(params.mot_right);
    encoders->ticks[params.mot_left] = mbot_encoder_read_count(params.mot_left);
    encoders->delta_ticks[params.mot_left] = mbot_encoder_read_delta(params.mot_left);

    if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        encoders->ticks[params.mot_back] = mbot_encoder_read_count(params.mot_back);
        encoders->delta_ticks[params.mot_back] = mbot_encoder_read_delta(params.mot_back);
    }
}

int mbot_init_pico(void){
    bi_decl(bi_program_description("Firmware for the MBot Robot Control Board"));
    
    // set master clock to 250MHz (if unstable set SYS_CLOCK to 125Mhz)
    if(!set_sys_clock_khz(125000, true)){
        printf("ERROR mbot_init_pico: cannot set system clock\n");
        return MBOT_ERROR;
    }; 
    
    stdio_init_all(); // enable USB serial terminal
    sleep_ms(500);
    printf("\nMBot Booting Up!\n");
    return MBOT_OK;
}

int mbot_init_hardware(void){
    sleep_ms(1000);
    // Initialize Motors
    printf("initializinging motors...\n");
    mbot_motor_init(DIFF_MOTOR_LEFT_SLOT);
    if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        mbot_motor_init(1);
    }
    mbot_motor_init(DIFF_MOTOR_RIGHT_SLOT);
    printf("initializinging encoders...\n");
    mbot_encoder_init();

    // Initialize LED
    printf("Starting heartbeat LED...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    // Initialize the IMU using the Digital Motion Processor
    printf("Initializing IMU...\n");
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    mbot_init_fram();
    return MBOT_OK;
}

int mbot_init_comms(void){
    printf("Initializing LCM serial communication...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);
    return MBOT_OK;
}

void mbot_print_state(serial_mbot_imu_t imu, serial_mbot_encoders_t encoders, serial_pose2D_t odometry, serial_mbot_motor_vel_t motor_vel){
    printf("\033[2J\r");
    if(global_comms_status == COMMS_OK){
        printf("| \033[32m COMMS OK \033[0m TIME: %lld |\n", global_utime);
    }
    else{
        printf("| \033[31m SERIAL COMMUNICATION FAILURE\033[0m     |\n");
    }
    const char* imu_headings[] = {"ROLL", "PITCH", "YAW"};
    const char* enc_headings[] = {"ENC 0", "ENC 1", "ENC 2"};
    const char* odom_headings[] = {"X", "Y", "THETA"};
    const char* motor_vel_headings[] = {"MOT 0", "MOT 1", "MOT 2"};
    // we shouldn't need to do this, need to update generateTable to handle different datatypes
    int encs[3] = {(int)encoders.ticks[0], (int)encoders.ticks[1], (int)encoders.ticks[2]};
    char buf[1024] = {0};
    generateTableInt(buf, 1, 3, "ENCODERS", enc_headings, encs);
    printf("\r%s", buf);
    
    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "IMU", imu_headings, imu.angles_rpy);
    printf("\r%s", buf);
    
    buf[0] = '\0';
    generateTableFloat(buf, 1, 3, "MOTOR", motor_vel_headings, motor_vel.velocity);
    printf("\r%s", buf);
    
    buf[0] = '\0';
    float odom_array[3] = {odometry.x, odometry.y, odometry.theta};
    generateTableFloat(buf, 1, 3, "ODOMETRY", odom_headings, odom_array);
    printf("\r%s", buf);

    buf[0] = '\0';
    generateBottomLine(buf, 3);
    printf("\r%s\n", buf);
}

//Helper function to use slope + intercept from calibration to generate a PWM command.
float _calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx){
    if (vel_cmd > 0.0)
    {
        return (vel_cmd * params.slope_pos[motor_idx]) + params.itrcpt_pos[motor_idx];
    }
    else if (vel_cmd < 0.0)
    {
        return (vel_cmd * params.slope_neg[motor_idx]) + params.itrcpt_neg[motor_idx];
    }
    return 0.0;
}

// TODO: this could be tied to the IMU interrupt
bool mbot_loop(repeating_timer_t *rt)
{
    global_utime = to_us_since_boot(get_absolute_time()) + timestamp_offset;
    mbot_vel.utime = global_utime;
    mbot_read_encoders(&mbot_encoders);
    mbot_read_imu(&mbot_imu);
    mbot_calculate_motor_vel(mbot_encoders, &mbot_motor_vel);

    if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){
        // mbot_calculate_diff_body_vel(   mbot_motor_vel.velocity[params.mot_left], 
        //                                 mbot_motor_vel.velocity[params.mot_right], 
        //                                 &mbot_vel
        //                             );

        mbot_calculate_diff_body_vel_imu(   mbot_motor_vel.velocity[params.mot_left], 
                                            mbot_motor_vel.velocity[params.mot_right], 
                                            mbot_imu,
                                            &mbot_vel
                                        );
    }
    
    else if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
        mbot_calculate_omni_body_vel(   mbot_motor_vel.velocity[params.mot_left], 
                                        mbot_motor_vel.velocity[params.mot_right], 
                                        mbot_motor_vel.velocity[params.mot_back], 
                                        &mbot_vel
                                    );
    }

    mbot_calculate_odometry(mbot_vel, MAIN_LOOP_PERIOD, &mbot_odometry);
    override_heading(&mbot_odometry, &mbot_imu);
    mbot_odometry.utime = global_utime;
    // only run if we've got 2 way communication...
    if (global_comms_status == COMMS_OK)
    {
        if(drive_mode == MODE_MOTOR_VEL_OL){
            // printf("Sexy girl.\n");
            mbot_motor_pwm.utime = global_utime;
            mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_right], params.mot_right);
            mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_left], params.mot_left);
            if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
                mbot_motor_pwm_cmd.pwm[params.mot_back] = _calibrated_pwm_from_vel_cmd(mbot_motor_vel_cmd.velocity[params.mot_back], params.mot_back);
            }
        }else if(drive_mode == MODE_MBOT_VEL){
            //TODO: open loop for now - implement closed loop controller
            /**
             * TODO: 
             *   - Implement closed loop controller
             *   - Implement velocity controller
             *   - We will be running on the DIFFERENTIAL_DRIVE
            */
            if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
                mbot_motor_vel_cmd.velocity[params.mot_left] = (SQRT3 / 2.0 * mbot_vel_cmd.vx - 0.5 * mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                mbot_motor_vel_cmd.velocity[params.mot_right] = (-SQRT3 / 2.0 * mbot_vel_cmd.vx - 0.5 * mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                mbot_motor_vel_cmd.velocity[params.mot_back] = (mbot_vel_cmd.vy - params.wheel_base_radius * mbot_vel_cmd.wz) / params.wheel_radius;
                float vel_left_comp = params.motor_polarity[params.mot_left] * mbot_motor_vel_cmd.velocity[params.mot_left];
                float vel_right_comp = params.motor_polarity[params.mot_right] * mbot_motor_vel_cmd.velocity[params.mot_right];
                float vel_back_comp = params.motor_polarity[params.mot_back] * mbot_motor_vel_cmd.velocity[params.mot_back];
                
                mbot_motor_pwm.utime = global_utime;
                mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(vel_right_comp, params.mot_right);
                mbot_motor_pwm_cmd.pwm[params.mot_back] = _calibrated_pwm_from_vel_cmd(vel_back_comp, params.mot_back);
                mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(vel_left_comp, params.mot_left);
            }else if(MBOT_DRIVE_TYPE == DIFFERENTIAL_DRIVE){

                /**
                 * NOTE:
                 *  -> OUR CODE WILL RUN HERE.
                */

                //NOTE: u1 = mbot_vel_cmd.vx is the desired translational velocity
                //NOTE: u2 = mbot_vel_cmd.wz is the desired angular velocity

                //given [u1,u2] we calculate the open-loop motor commands [vLeft,vRight]
                //these vLeft and vRight will be our motor commands essentially

                //Odometry will give us (x,y,theta)

                float meas_vx = mbot_vel.vx;
                float meas_wz = mbot_vel.wz;

                meas_vx = rc_filter_march(&vx_meas_lowpass, meas_vx);
                meas_wz = rc_filter_march(&wz_meas_lowpass, meas_wz);
                // rintf("meas_vx, meas_wx: (%f, %f)\n");

                float pp_vx = mbot_vel_cmd.vx;
                float pp_wz = mbot_vel_cmd.wz;

		        pp_vx += rc_filter_march(&pid_vx, mbot_vel_cmd.vx - meas_vx);
		        pp_wz += rc_filter_march(&pid_wz, mbot_vel_cmd.wz - meas_wz);

                // feed-forward and vel pid targets
                float vL_Goal = (pp_vx - DIFF_BASE_RADIUS * pp_wz) / DIFF_WHEEL_RADIUS;
                float vR_Goal = (-pp_vx - DIFF_BASE_RADIUS * pp_wz) / DIFF_WHEEL_RADIUS;
                vL_Goal = rc_filter_march(&motor_vL_lowpass, vL_Goal);
                vR_Goal = rc_filter_march(&motor_vR_lowpass, vR_Goal);

                // run pid for vel targets
                float vL = rc_filter_march(&pid_leftwheel, vL_Goal - mbot_motor_vel.velocity[params.mot_left]);
                float vR = rc_filter_march(&pid_rightwheel, vR_Goal - mbot_motor_vel.velocity[params.mot_right]);
                // vL = 0;
                // vR = 0;

                // vel_cmd = feedforward + feedback
                mbot_motor_vel_cmd.velocity[params.mot_left] = vL + vL_Goal;
                mbot_motor_vel_cmd.velocity[params.mot_right] = vR + vR_Goal;


                // low-level cmd
                float vel_left_comp = params.motor_polarity[params.mot_left] * mbot_motor_vel_cmd.velocity[params.mot_left];
                float vel_right_comp = params.motor_polarity[params.mot_right] * mbot_motor_vel_cmd.velocity[params.mot_right];

                //might not be good to run lowpass on the motor commands (PID will drive oscillations)
                // vel_left_comp = rc_filter_march(&motor_vL_lowpass, vel_left_comp);
                // vel_right_comp = rc_filter_march(&motor_vR_lowpass, vel_right_comp);
                // printf("(vL,vR): (%f,%f)\n", vel_left_comp, vel_right_comp);


                mbot_motor_pwm.utime = global_utime;
                mbot_motor_pwm_cmd.pwm[params.mot_right] = _calibrated_pwm_from_vel_cmd(vel_right_comp, params.mot_right);
                mbot_motor_pwm_cmd.pwm[params.mot_left] = _calibrated_pwm_from_vel_cmd(vel_left_comp, params.mot_left);
            }
        }else {
            drive_mode = MODE_MOTOR_PWM;
            mbot_motor_pwm.utime = global_utime;
        }

        // Set motors
        mbot_motor_set_duty(params.mot_right, mbot_motor_pwm_cmd.pwm[params.mot_right]);
        mbot_motor_pwm.pwm[params.mot_right] = mbot_motor_pwm_cmd.pwm[params.mot_right];
        mbot_motor_set_duty(params.mot_left, mbot_motor_pwm_cmd.pwm[params.mot_left]);
        mbot_motor_pwm.pwm[params.mot_left] = mbot_motor_pwm_cmd.pwm[params.mot_left];

        if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
            mbot_motor_set_duty(params.mot_back, mbot_motor_pwm_cmd.pwm[params.mot_back]);
            mbot_motor_pwm.pwm[params.mot_back] = mbot_motor_pwm_cmd.pwm[params.mot_back];
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &mbot_encoders);
        // send odom on wire
        comms_write_topic(MBOT_ODOMETRY, &mbot_odometry);
        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &mbot_imu);
        // write the Body velocity to serial
        comms_write_topic(MBOT_VEL, &mbot_vel);
        // write the Motor velocity to serial
        comms_write_topic(MBOT_MOTOR_VEL, &mbot_motor_vel);
        // write the PWMs to serial
        comms_write_topic(MBOT_MOTOR_PWM, &mbot_motor_pwm);
        //uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }
    //check comms and kill motors if its been too long
    uint64_t timeout = to_us_since_boot(get_absolute_time()) - global_pico_time;
    if(timeout > MBOT_TIMEOUT_US){
        mbot_motor_set_duty(DIFF_MOTOR_LEFT_SLOT, 0.0);
        if(MBOT_DRIVE_TYPE == OMNI_120_DRIVE){
            mbot_motor_set_duty(1, 0.0);
        }
        mbot_motor_set_duty(DIFF_MOTOR_RIGHT_SLOT, 0.0);
        global_comms_status = COMMS_ERROR;
    }

    return true;
}

void lowpass_setup(rc_filter_t* lowpass, double alpha) 
{
    // equation: y[n] = a*x[n] + (1-a)*y[n-1]
    // a = dt/(tau + dt)
    // rework equation
    // tau = (dt - dt*a)/a 
    *lowpass = rc_filter_empty();
    const double dt = MAIN_LOOP_PERIOD * 1000;
    const double tau = (dt - dt*alpha)/alpha;
    rc_filter_first_order_lowpass(lowpass, dt, tau);
    rc_filter_prefill_inputs(lowpass, 0.0);
}

int main()
{   
    running = false;
    mbot_init_pico();
    mbot_init_hardware();
    mbot_init_comms();
    mbot_read_fram(0, sizeof(params), &params);
    
    //Check also that define drive type is same as FRAM drive type
    int validate_status = validate_FRAM_data(&params);
    if (validate_status < 0)
    {
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
        return -1;
    }

    if(params.robot_type != MBOT_DRIVE_TYPE){
        printf("#define type is not equal to calibration type!\n");
        return -1;
    }

    #ifdef CALIBRATE_GYRO
    printf("Starting Gyroscope Calibration:\n");
    printf("----------------------------------");
    size_t num_samples = 500;
    float calib_gx = 0, calib_gy = 0, calib_gz = 0;
    for (size_t i = 0; i < num_samples; ++i)
    {
        mbot_read_imu(&mbot_imu);
        calib_gx += mbot_imu_data.gyro[0];
        calib_gy += mbot_imu_data.gyro[1];
        calib_gz += mbot_imu_data.gyro[2];
        sleep_ms(20); //20ms refresh rate of imu sensor
    }
    calib_gx /= num_samples;
    calib_gy /= num_samples;
    calib_gz /= num_samples;
    printf("Average gyro_x bias: %f\n", calib_gx);
    printf("Average gyro_x bias: %f\n", calib_gy);
    printf("Average gyro_x bias: %f\n", calib_gz);
    printf("----------------------------------");
    printf("`Done!\n");
    #endif

    /**
     * setup vars before mbot_loop
    */

    // Gyro LPF
    // gyro_lowpass = rc_filter_empty();
    // const double dt = MAIN_LOOP_PERIOD * 1000;
    // rc_filter_first_order_lowpass(&gyro_lowpass, 1.0, 1.00);
    // rc_filter_prefill_inputs(&gyro_lowpass, 0.0);
    // rc_filter_prefill_inputs(&gyro_lowpass, 0.0);

    // PID vel drive
    float vxP = 0.00, vxI = 0.0, vxD = 0.0;
    float wzP = 0.00, wzI = 0.0, wzD = 0.0;
    if (rc_filter_pid(&pid_vx, vxP, vxI, vxD, MAIN_LOOP_PERIOD*1000 / 250.0, MAIN_LOOP_PERIOD*1000 / 1000.0))
    {
        printf("Failed to create PID filter\n");
        return -1;
    }
    if (rc_filter_pid(&pid_wz, wzP, wzI, wzD, MAIN_LOOP_PERIOD*1000 / 250.0, MAIN_LOOP_PERIOD*1000 / 1000.0))
    {
        printf("Failed to create PID filter\n");
        return -1;
    }

    // NOTE: we want to smooth out velocities or something to prevent
    // this sets up lowpass to run on motor commands
    lowpass_setup(&motor_vL_lowpass, 0.1);
    lowpass_setup(&motor_vR_lowpass, 0.1);
    lowpass_setup(&vx_meas_lowpass, 0.3);
    lowpass_setup(&wz_meas_lowpass, 0.3);


    float kP = 1.5, kI = 1.5, kD = 1E-9;
    // float kP=0, kI=0, kD=0;
    rc_filter_pid(&pid_leftwheel, kP, kI, kD, MAIN_LOOP_PERIOD*1000 / 250.0, MAIN_LOOP_PERIOD*1000 / 1000.0);
    rc_filter_pid(&pid_rightwheel, kP, kI, kD, MAIN_LOOP_PERIOD*1000 / 250.0, MAIN_LOOP_PERIOD*1000 / 1000.0);
    float filter_cutoff = 15.0;
    rc_filter_enable_saturation(&pid_leftwheel, -filter_cutoff, filter_cutoff);
    rc_filter_enable_saturation(&pid_rightwheel, -filter_cutoff, filter_cutoff);

    sleep_ms(3000);
    print_mbot_params(&params);
    printf("Starting MBot Loop...\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, mbot_loop, NULL, &loop_timer); // 1000x to convert to ms
    printf("Done Booting Up!\n");
    running = true;
    uint16_t counter = 0;
    
    while(running){
        // Heartbeat
        if(!(counter % 5)){
            gpio_put(LED_PIN, 1);
        }
        else if(!(counter % 7)){
            gpio_put(LED_PIN, 1);
            counter = 0;
        }
        else{
            gpio_put(LED_PIN, 0);
        }
        // Print State
        #ifdef PRINT_MBOT_STATE
        mbot_print_state(mbot_imu, mbot_encoders, mbot_odometry, mbot_motor_vel);
        #endif
        printf("ODOMETRY DATA: (%" PRId64 ", %8.4f, %8.4f, %8.4f)\n", mbot_odometry.utime, mbot_odometry.x, mbot_odometry.y, mbot_odometry.theta);
        
        // printf("Help me\n");
        sleep_ms(200);  
        counter++;
    }
}

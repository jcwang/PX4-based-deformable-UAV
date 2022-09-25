/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#include <drivers/drv_pwm_output.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/commander_state.h>


__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{


	int sensor_sub_fd = orb_subscribe(ORB_ID(commander_state));

	orb_set_interval(sensor_sub_fd, 200);


	struct commander_state_s att;
	memset(&att, 0, sizeof(att));
	orb_advert_t att_pub = orb_advertise(ORB_ID(commander_state), &att);


	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },

	};

	int error_counter = 0;
	uint8_t last_state = 8;//set stable
	char form = 'X';

	while (true) {

		int poll_ret = px4_poll(fds, 1, 1000);

		if (poll_ret == 0) {

			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {

			if (error_counter < 10 || error_counter % 50 == 0) {

				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {

				struct commander_state_s accel;

				orb_copy(ORB_ID(commander_state), sensor_sub_fd, &accel);

				// PX4_INFO("main_state :%d ,%d",
				// 	 (int)accel.main_state,
				// 	 (int)accel.main_state_changes);
				if (accel.main_state == 1) {//attitude
					if ((last_state == 8 || last_state == 1) && accel.main_state_changes > 9) {
						form = 'y';

					} else if ((last_state == 8 || last_state == 1) && accel.main_state_changes > 7) {
						form = 'T';

					} else if ((last_state == 8 || last_state == 1) && accel.main_state_changes > 5) {
						form = 'y';

					} else if ((last_state == 8 || last_state == 1) && accel.main_state_changes > 3) {
						form = 'H';

					} else {
						form = 'Y';
					}

					if (accel.main_state_changes > 11) {
						form = 'X';
					}

					last_state = 1;

				} else if (accel.main_state == 4) { //hold
					if ((last_state == 2 || last_state == 4) && accel.main_state_changes > 9) {
						form = 'X';

					} else if ((last_state == 2 || last_state == 4) && accel.main_state_changes > 7) {
						form = 'H';

					} else if ((last_state == 2 || last_state == 4) && accel.main_state_changes > 5) {
						form = 'Y';

					} else if ((last_state == 2 || last_state == 4) && accel.main_state_changes > 3) {
						form = 'H';

					} else {
						form = 'Y';
					}

					if (accel.main_state_changes > 11) {
						form = 'X';
					}

					last_state = 4;

				} else if (accel.main_state == 8) { //stable
					form = 'X';
					last_state = 8;

				} else if (accel.main_state == 2) { //position
					// if (last_state == 4 ) {
					// 	form = 'Y';

					// } else {
					// 	form = 'X';
					// }

					last_state = 2;
				}


				switch (form) {
				case 'X':
					up_pwm_servo_set(0, 1500);//right-up
					up_pwm_servo_set(1, 1435);//left-down
					up_pwm_servo_set(4, 1600);//left-up
					up_pwm_servo_set(5, 1465);//right-down
					break;

				case 'Y':
					up_pwm_servo_set(0, 1500);
					up_pwm_servo_set(1, 1935);
					up_pwm_servo_set(4, 1600);
					up_pwm_servo_set(5, 1065);
					break;

				case 'H':
					up_pwm_servo_set(0, 1900);
					up_pwm_servo_set(1, 1935);
					up_pwm_servo_set(4, 1200);
					up_pwm_servo_set(5, 1065);
					break;

				case 'T':
					up_pwm_servo_set(0, 1900);
					up_pwm_servo_set(1, 1200);
					up_pwm_servo_set(4, 1200);
					up_pwm_servo_set(5, 1800);
					break;

				case 'y':
					up_pwm_servo_set(0, 1900);
					up_pwm_servo_set(1, 1435);
					up_pwm_servo_set(4, 1200);
					up_pwm_servo_set(5, 1465);
					break;

				default:
					up_pwm_servo_set(0, 1500);
					up_pwm_servo_set(1, 1435);
					up_pwm_servo_set(4, 1600);
					up_pwm_servo_set(5, 1465);
					break;
				}

				orb_publish(ORB_ID(commander_state), att_pub, &att);
			}
		}
	}


	return 0;
}

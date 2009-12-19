/*  
 *  Copyright Droids Corporation (2009)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: arm_xy.h,v 1.4 2009-11-08 17:25:00 zer0 Exp $
 *
 * Fabrice DESCLAUX <serpilliere@droids-corp.org>
 * Olivier MATZ <zer0@droids-corp.org>
 */

/* all static configuration for an arm */
struct arm_config {
	void (*wrist_angle_deg2robot)(double wrist_edg,
				      double *wrist_out);

	void (*angle_rad2robot)(double shoulder_deg, double elbow_deg,
				double *shoulder_out, double *elbow_out);

	void (*angle_robot2rad)(double shoulder_robot, double elbow_robot,
				double *shoulder_rad, double *elbow_rad);

	/* ax12 identifiers */
	int8_t elbow_ax12;
	int8_t wrist_ax12;
	
	/* related control system */
	struct cs_block *csb;

	/* if true, don't apply to ax12 / cs */
	uint8_t simulate;
};

/* request for a final position, in mm */
struct arm_request {
	int32_t h_mm;
	int32_t d_mm;
	int32_t w_deg;
};

/* returned by arm_test_traj_end() */
#define ARM_TRAJ_END         0x01
#define ARM_TRAJ_NEAR        0x02
#define ARM_TRAJ_TIMEOUT     0x04
#define ARM_TRAJ_ERROR       0x08

#define ARM_TRAJ_ALL (ARM_TRAJ_END|ARM_TRAJ_TIMEOUT|ARM_TRAJ_ERROR)
#define ARM_TRAJ_ALL_NEAR (ARM_TRAJ_END|ARM_TRAJ_NEAR|ARM_TRAJ_TIMEOUT|ARM_TRAJ_ERROR)
#define ARM_TRAJ_END_NEAR (ARM_TRAJ_END|ARM_TRAJ_NEAR)

#define ARM_STATE_INIT       0
#define ARM_FLAG_MOVING      0x01 /* arm is currently moving */
#define ARM_FLAG_LAST_STEP   0x02 /* no more step is needed */
#define ARM_FLAG_IN_WINDOW   0x04 /* arm speed and pos are ok */
#define ARM_FLAG_TIMEOUT     0x08 /* too much time too reach pos */
#define ARM_FLAG_ERROR       0x10 /* error */
#define ARM_FLAG_FINISHED    (ARM_FLAG_LAST_STEP | ARM_FLAG_IN_WINDOW)

/* Describes the current position of the arm. Mainly filled by
 * arm_do_step(), arm_set_wrist(), arm_do_xy_cb(), ... */
struct arm_status {
	/* current position */
	int32_t h_mm;
	int32_t d_mm;

	/* wrist goal position (set once at init) */
	int16_t wrist_angle_steps;

	/* current angles in steps */
	int32_t elbow_angle_steps;
	int32_t shoulder_angle_steps;

	/* current angles in radian */
	double elbow_angle_rad;
	double shoulder_angle_rad;

	/* time before next update */
	uint32_t next_update_time;

	/* what speed to be applied, in specific speed unit */
	uint32_t shoulder_speed;
	uint32_t elbow_speed;

	volatile int8_t state; /* see list of flags above */
	int8_t event; /* scheduler event, -1 if not running */

	microseconds start_time; /* when we started that command */
	microseconds wrist_reach_time; /* when the wrist should reach dest */
	microseconds pos_reached_time; /* when last step is sent */
};

struct arm {
	struct arm_config config;
	struct arm_status status;
	struct arm_request req;
};

extern struct arm left_arm;
extern struct arm right_arm;

uint8_t arm_test_traj_end(struct arm *arm, uint8_t mask);
uint8_t arm_wait_traj_end(struct arm *arm, uint8_t mask);

/* do a specific move to distance, height. This function _must_ be
 * called from a context with a prio < ARM_PRIO to avoid any race
 * condition. */
int8_t arm_do_xy(struct arm *arm, int16_t d_mm, int16_t h_mm, int16_t w_deg);

void arm_dump(struct arm *arm);
void arm_calibrate(void);
void arm_init(void);

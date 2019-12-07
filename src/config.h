#ifndef __CONFIG_H
#define __CONFIG_H

/** Locomotion-related constants */
#define MICROMETERS_PER_COUNT 4.3457 //4.32
#define WHEELS_SEPARATION 0.085
#define SHIFT_AFTER_180_DEG_TURN 0.00

/** Time it takes for the robot to decide where to go next while searching */
#define SEARCH_REACTION_TIME 0.01 // 0.01

/** Calibration constants for sensors */
#define SENSOR_SIDE_LEFT_A  3.080 //2.806
#define SENSOR_SIDE_LEFT_B 0.262 // 0.287
#define SENSOR_SIDE_RIGHT_A 3.278 // 2.327
#define SENSOR_SIDE_RIGHT_B 0.292 // 0.231
#define SENSOR_FRONT_LEFT_A 3.113 //2.609
#define SENSOR_FRONT_LEFT_B 0.254 // 0.242
#define SENSOR_FRONT_RIGHT_A 5.067 // 2.713
#define SENSOR_FRONT_RIGHT_B 0.495 // 0.258

/** Control constants */
#define KP_LINEAR 2.0 //8.
#define KD_LINEAR 6.//16.
#define KP_ANGULAR 0.07  //0.1 // 0.05
#define KD_ANGULAR 1.
#define KI_ANGULAR .001
#define KP_ANGULAR_FRONT 0.5 //.5
#define KI_ANGULAR_FRONT 2.0//2.
#define KP_ANGULAR_SIDE 4.
#define KI_ANGULAR_SIDE 4.
#define KP_ANGULAR_DIAGONAL 2.0//2.
#define KI_ANGULAR_DIAGONAL 4.0//4


/** Control constants (resolving) */
#define KP_LINEAR_RESOLVING 4.0 //8.
#define KD_LINEAR_RESOLVING 8.//16.
#define KP_ANGULAR_RESOLVING 0.09  //0.1 // 0.05
#define KD_ANGULAR_RESOLVING 2.5
#define KI_ANGULAR_RESOLVING .0 //.0001
#define KP_ANGULAR_FRONT_RESOLVING .25//.5
#define KI_ANGULAR_FRONT_RESOLVING 2.0//2.
#define KP_ANGULAR_SIDE_RESOLVING 5.5
#define KI_ANGULAR_SIDE_RESOLVING 4.
#define KP_ANGULAR_DIAGONAL_RESOLVING 5//2.
#define KI_ANGULAR_DIAGONAL_RESOLVING 4.0//4



struct control_constants {
	float kp_linear;
	float kd_linear;
	float kp_angular;
	float kd_angular;
  float ki_angular; 
	float kp_angular_front;
	float ki_angular_front;
	float kp_angular_side;
	float ki_angular_side;
	float kp_angular_diagonal;
	float ki_angular_diagonal;
};

/** Speed constants */
#define LINEAR_SPEED_LIMIT 2.


float get_micrometers_per_count(void);
void set_micrometers_per_count(float value);
float get_wheels_separation(void);
void set_wheels_separation(float value);
struct control_constants get_control_constants(void);
void set_control_constants(struct control_constants value);
float get_linear_speed_limit(void);
void set_linear_speed_limit(float value);
void modify_control_for_resolving(void);

#endif /* _CONFIG_H */

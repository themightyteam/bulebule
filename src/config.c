#include "config.h"

static volatile float micrometers_per_count = MICROMETERS_PER_COUNT;
static volatile float wheels_separation = WHEELS_SEPARATION;
static volatile struct control_constants control = {
    .kp_linear = KP_LINEAR,
    .kd_linear = KD_LINEAR,
    .kp_angular = KP_ANGULAR,
    .kd_angular = KD_ANGULAR,
    .ki_angular = KI_ANGULAR,
    .kp_angular_front = KP_ANGULAR_FRONT,
    .ki_angular_front = KI_ANGULAR_FRONT,
    .kp_angular_side = KP_ANGULAR_SIDE,
    .ki_angular_side = KI_ANGULAR_SIDE,
    .kp_angular_diagonal = KP_ANGULAR_DIAGONAL,
    .ki_angular_diagonal = KI_ANGULAR_DIAGONAL};

static volatile float linear_speed_limit = LINEAR_SPEED_LIMIT;


void modify_control_for_resolving(void)
{
  control.kp_linear = KP_LINEAR_RESOLVING;
  control.kd_linear = KD_LINEAR_RESOLVING;
  control.kp_angular = KP_ANGULAR_RESOLVING;
  control.kd_angular = KD_ANGULAR_RESOLVING;
  control.ki_angular = KI_ANGULAR_RESOLVING;
  control.kp_angular_front = KP_ANGULAR_FRONT_RESOLVING;
  control.ki_angular_front = KI_ANGULAR_FRONT_RESOLVING;
  control.kp_angular_side = KP_ANGULAR_SIDE_RESOLVING;
  control.ki_angular_side = KI_ANGULAR_SIDE_RESOLVING;
  control.kp_angular_diagonal = KP_ANGULAR_DIAGONAL_RESOLVING;
  control.ki_angular_diagonal = KI_ANGULAR_DIAGONAL_RESOLVING;

}


float get_micrometers_per_count(void)
{
	return micrometers_per_count;
}

void set_micrometers_per_count(float value)
{
	micrometers_per_count = value;
}

float get_wheels_separation(void)
{
	return wheels_separation;
}

void set_wheels_separation(float value)
{
	wheels_separation = value;
}

struct control_constants get_control_constants(void)
{
	return control;
}

void set_control_constants(struct control_constants value)
{
	control = value;
}

float get_linear_speed_limit(void)
{
	return linear_speed_limit;
}

void set_linear_speed_limit(float value)
{
	linear_speed_limit = value;
}

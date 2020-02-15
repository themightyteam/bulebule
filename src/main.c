#include "mmlib/calibration.h"
#include "mmlib/clock.h"
#include "mmlib/command.h"
#include "mmlib/control.h"
#include "mmlib/encoder.h"
#include "mmlib/hmi.h"
#include "mmlib/logging.h"
#include "mmlib/move.h"
#include "mmlib/search.h"
#include "mmlib/solve.h"
#include "mmlib/speed.h"
#include "mmlib/walls.h"

#include "eeprom.h"
#include "motor.h"
#include "setup.h"
#include "voltage.h"

#define MAX_TICKS_TO_LOG 50
int ticks_to_log = MAX_TICKS_TO_LOG;

enum path_language current_path_selected = PATH_DIAGONALS;


/**
 * @brief Handle the SysTick interruptions.
 */
void sys_tick_handler(void)
{
	clock_tick();
	update_distance_readings();
	update_gyro_readings();
	update_encoder_readings();
	motor_control();

	ticks_to_log = ticks_to_log - 1;
	if (ticks_to_log < 0) {
		log_data();
		ticks_to_log = MAX_TICKS_TO_LOG;
	}
}

/**
 * @brief Check battery voltage and warn if the voltage is getting too low.
 */
static void check_battery_voltage(void)
{
	float voltage;

	voltage = get_battery_voltage();
	if (voltage < 3.6)
		speaker_warn_low_battery();
	if (voltage < 3.5)
		speaker_warn_low_battery();
	if (voltage < 3.4)
		speaker_warn_low_battery();
	if (voltage < 3.3)
		speaker_play_error();
}

/**
 * @brief Includes the functions to be executed before robot starts to move.
 */
static void before_moving(void)
{
	reset_motion();
	disable_walls_control();
	repeat_blink(10, 100);
	sleep_us(5000000);
	check_battery_voltage();
	led_left_on();
	led_right_on();
	wait_front_sensor_close_signal(0.16);
	speaker_play_beeps(4);
	srand(read_cycle_counter());
	led_left_off();
	led_right_off();
	sleep_us(2000000);
	calibrate();
	enable_motor_control();

//target_straight(0, CELL_DIMENSION * 4, 0.0);
//led_right_on();
	//run_movement_sequence("FFFFr");
	//while(1){}

	set_starting_position();
}

/**
 * @brief Functions to be executed after moving (running or exploring).
 */
static void after_moving(void)
{
	reset_motion();
	if (collision_detected())
		blink_collision();
	else
		speaker_play_success();
	check_battery_voltage();
}

/**
 * @brief Let the user configure the mouse forces before exploring or running.
 *
 * The user selects the force to apply to the tires while exploring or running.
 *
 * Data logging is always active during movement phase.
 *
 * @param[in] run Whether the robot should be running.
 */
static void movement_phase(bool do_run)
{
	float force;

	force = hmi_configure_force(0.10 /*0.1*/, 0.025/*0.05*/);
	kinematic_configuration(force, do_run);

	start_data_logging(log_data_control);
	before_moving();
	if (!do_run) {
		explore(force);
	} else {
	  modify_control_for_resolving();
	  run(force, current_path_selected);
	  run_back(force);
	}
	after_moving();
	stop_data_logging();
}

/**
 * @brief Configure the goal for the search phase.
 */
static void configure_goal(void)
{

  while(1)
    {
	switch (button_user_wait_action()) {
	  
	case BUTTON_SHORT:
		add_goal(10, 2);
		speaker_play_success();
		//add_goal(0, 1);
		return;
		//break;
	
	case BUTTON_LONG:
		set_goal_classic();
		speaker_play_competition();
		//add_goal(10, 2);
		return;
		//break;

	default:
	  break;
	   
	}
    }
}

/**
 * @brief Execute the exploration phase.
 */
static void exploration_phase(void)
{

  indy_1();
  indy_2();

  
  set_search_initial_direction(NORTH);
  //configure_goal();
  set_target_goal();

  movement_phase(false);

  set_run_sequence();
  save_maze();
}

/**
 * @brief Configure the robot and execute the search/run.
 */
static void configure_and_move(void)
{
	if (!maze_is_saved())
		exploration_phase();
	led_bluepill_on();


	bool should_start = false;
	
	while(1)
	  {
	
	switch (button_user_wait_action()) {
	case BUTTON_SHORT:
		load_maze();
		should_start = true;
		break;
	case BUTTON_LONG:
		reset_maze();
		led_bluepill_off();
		exploration_phase();
		led_bluepill_on();
		should_start = true;
		break;

	case BUTTON_SHORT_LEFT:
	  speaker_play_beeps(2);
	  current_path_selected = PATH_SAFE;
	  break;
	  
	case BUTTON_LONG_LEFT:
	  speaker_play_beeps(12);
	  current_path_selected = PATH_DIAGONALS;
	  break;
	}

	if (should_start)
	  break;

	  }
	
	while (1)
		movement_phase(true);
}


/**
 * @brief Configure the robot and execute the search/run.
 */
static void configure_and_move_buri(void)
{
	if (!maze_is_saved())
		exploration_phase();
	led_bluepill_on();

	// Load stored maze
	load_maze();

	under_the_sea_1();
	under_the_sea_2();

	bool should_start = false;
	
	while(1) {
	  
	  // Configure diagonals
	  switch(button_user_wait_action()) {
	  case BUTTON_SHORT_LEFT:
	    speaker_play_beeps(2);
	    current_path_selected = PATH_SAFE;
	    break;
	    
	  case BUTTON_LONG_LEFT:
	    speaker_play_beeps(12);
	    current_path_selected = PATH_DIAGONALS;
	    break;
	  
	  default:
	    should_start = true;
	    break;
	  }
	  if (should_start)
	    break;
	}

	speaker_play_competition();
	
	while (1)
		movement_phase(true);

}


void process_config_command()
{
  

  cantina_1();
  enum button_response action = button_user_wait_action();


  
  if (action == BUTTON_LONG) {
    cantina_1();
    action = button_user_wait_action();

    if (action == BUTTON_LONG) {
      cantina_1();
      speaker_play_beeps(40);
      speaker_play_success();
      led_bluepill_off();
      reset_maze();
    }
    else {
      speaker_play_error();
    }

  }
  else if (action == BUTTON_LONG_LEFT) {
    cantina_2();
    action = button_user_wait_action();
    if (action == BUTTON_LONG_LEFT) {
      cantina_2();
      speaker_play_success();
      // Set a custom goal
      add_goal(10, 2);
    }    
  }
  else {
    speaker_play_error();
  }
}

/**
 * @brief Initial setup and infinite wait.
 */
int main(void)
{
	setup();
	kinematic_configuration(0.25, false);
	systick_interrupt_enable();
	speaker_play_beeps(3);

	// set goal classic by default
	//set_goal_classic();
	add_goal(10, 2);
	
	if (!maze_is_saved())
	  {
	    indy_1();
	    indy_2();

	  }
	else
	  {
	    speaker_play_competition();
	  }
	

	while(1) {

	  switch (button_user_response()) {
	  case BUTTON_NONE:
	    switch (button_user_left_response())
	      {
	      case BUTTON_LONG_LEFT:
		// going to introduce a complex command
		process_config_command();
	    
		break;
	      default:
		break;

	      }
	    break;

	  case BUTTON_SHORT:
	  case BUTTON_LONG:
	    configure_and_move_buri();
	    
	  default:
	    
	    break;
	  }


	  execute_command();
	}

	return 0;

       

	
/* 
	int i;
	for (i = 0; i < 400; i++){
		power_left(i);
		power_right(i);
		sleep_ticks(10);
	}
		for (i = 400; i > 0; i--){
		power_left(i);
		power_right(i);
		sleep_ticks(10);
	}
*/

	/*
	while (1) {
		switch (button_user_response()) {
		case BUTTON_NONE:
			break;
		default:
			configure_and_move();
			break;
		}
		execute_command();
	}
	

	return 0;

	*/
}

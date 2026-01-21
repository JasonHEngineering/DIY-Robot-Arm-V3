#include "driver/mcpwm_prelude.h"

#define SERVO_GPIO 20 // this is actually using USB D+ pin

mcpwm_cmpr_handle_t comparator;
mcpwm_oper_handle_t oper;
mcpwm_timer_handle_t timer;
mcpwm_gen_handle_t generator;

void setServoUs(uint32_t us)
{
  // MCPWM timer resolution = 1 MHz → 1 tick = 1 µs
  mcpwm_comparator_set_compare_value(comparator, us);
}

void setup() {
  // 1 MHz timer resolution
  mcpwm_timer_config_t timer_cfg = {};
  timer_cfg.group_id = 0;
  timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_cfg.resolution_hz = 1000000;
  timer_cfg.period_ticks = 20000;   // 20 ms → 50 Hz
  timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

  mcpwm_new_timer(&timer_cfg, &timer);

  mcpwm_operator_config_t oper_cfg = {};
  oper_cfg.group_id = 0;
  mcpwm_new_operator(&oper_cfg, &oper);
  mcpwm_operator_connect_timer(oper, timer);

  mcpwm_comparator_config_t cmp_cfg = {};
  mcpwm_new_comparator(oper, &cmp_cfg, &comparator);

  mcpwm_generator_config_t gen_cfg = {};
  gen_cfg.gen_gpio_num = SERVO_GPIO;
  mcpwm_new_generator(oper, &gen_cfg, &generator);

  // HIGH at timer start
  mcpwm_generator_set_action_on_timer_event(
    generator,
    MCPWM_GEN_TIMER_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      MCPWM_TIMER_EVENT_EMPTY,
      MCPWM_GEN_ACTION_HIGH));

  // LOW when compare matches
  mcpwm_generator_set_action_on_compare_event(
    generator,
    MCPWM_GEN_COMPARE_EVENT_ACTION(
      MCPWM_TIMER_DIRECTION_UP,
      comparator,
      MCPWM_GEN_ACTION_LOW));

  mcpwm_timer_enable(timer);
  mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

  // Center servo
  setServoUs(1500);
}

void loop() {
  setServoUs(1000);  // 0°
  delay(2000);

  setServoUs(1500);  // 90°
  delay(2000);

  setServoUs(2000);  // 180°
  delay(2000);
}


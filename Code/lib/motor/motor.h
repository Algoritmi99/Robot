/**
 * @file motor.h
 * @ingroup motor
 *
 * Motor control functions
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>


/**
 * Initialize the motor library.
 *
 * Timer 3 (for motor A) and timer 4 (for motor B) are used to generate PWM signals
 * with which the motor's speed is controlled through the motor driver TI DRV8833.
 * Phase correct PWM mode is used with a frequency of 488.759Hz.
 */
void Motor_init(void);


/**
 * Set velocity of motor A (determines the duty cycle of the PWM signal).
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * Note that due to the phase correct PWM mode, an updated velocity value is
 * taken over by the PWM generator only once during a cycle which has a duration
 * of 1/488.759Hz = 2.046ms. This constitutes an upper limit on how often new
 * velocities should be set.
 *
 * @param   velocity   a value in [-8191...+8191] determining direction (sign)
 *                     and speed (absolute value)
 */
void Motor_setVelocityA(const int16_t velocity);


/**
 * Set velocity of motor B (determines the duty cycle of the PWM signal).
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * Note that due to the phase correct PWM mode, an updated velocity value is
 * taken over by the PWM generator only once during a cycle which has a duration
 * of 1/488.759Hz = 2.046ms. This constitutes an upper limit on how often new
 * velocities should be set.
 *
 * @param   velocity   a value in [-8191...+8191] determining direction (sign)
 *                     and speed (absolute value)
 */
void Motor_setVelocityB(const int16_t velocity);


/**
 * Set velocities for motors A and B (determines the duty cycle of the PWM signal).
 *
 * This function should be used in preference to Motor_setVelocityA() and
 * Motor_setVelocityB() when setting the velocities of both motors.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 *
 * Note that due to the phase correct PWM mode, an updated velocity value is
 * taken over by the PWM generator only once during a cycle which has a duration
 * of 1/488.759Hz = 2.046ms. This constitutes an upper limit on how often new
 * velocities should be set.
 *
 * @param   velocityA   a value in [-8191...+8191] determining direction (sign)
 *                      and speed (absolute value) of motor A
 * @param   velocityB   a value in [-8191...+8191] determining direction (sign)
 *                      and speed (absolute value) of motor B
 */
void Motor_setVelocities(const int16_t velocityA, const int16_t velocityB);


/**
 * Stop motor A
 *
 * This function should be used in preference to Motor_setVelocityA(0).
 *
 * Note that Motor_setVelocityA(0) is not equivalent to Motor_stopA().
 * The former uses the slow-decay mode (breaking), leaving the outputs of the
 * motor driver low. The latter uses the fast-decay-mode (coasting) in which
 * the outputs of the motor driver become high-impedant.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 */
void Motor_stopA(void);


/**
 * Stop motor B
 *
 * This function should be used in preference to Motor_setVelocityB(0).
 *
 * Note that Motor_setVelocityB(0) is not equivalent to Motor_stopB().
 * The former uses the slow-decay mode (breaking), leaving the outputs of the
 * motor driver low. The latter uses the fast-decay-mode (coasting) in which
 * the outputs of the motor driver become high-impedant.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards.
 */
void Motor_stopB(void);


/**
 * Stop motors A and B.
 *
 * This function should be used in preference to Motor_stopA() and
 * Motor_stopB() when stopping both motors simultaneously.
 *
 * Note that Motor_setVelocities(0,0) is not equivalent to Motor_stopAll().
 * The former uses the slow-decay mode (breaking), leaving the outputs of the
 * motor driver low. The latter uses the fast-decay-mode (coasting) in which
 * the outputs of the motor driver become high-impedant.
 *
 * If enabled, interrupts are disabled during this function and their state
 * restored afterwards. This ensures that both motors are stopped simultaneously.
 */
void Motor_stopAll(void);


#endif /* MOTOR_H_ */

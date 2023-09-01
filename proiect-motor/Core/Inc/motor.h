#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

#define MOT_FORWARD 1
#define MOT_STOP 0
#define MOT_BACKWARD -1
#define MOT_FWD MOT_FORWARD
#define MOT_BKD MOT_BACKWARD

void mot_init(void);
void mot_set_dir(int dir);
void mot_set_pwm(uint16_t pwm);
void mot_set_vel(float vel); // TODO
void mot_set(uint16_t pwm, int dir);
void mot_toggle_dir();
uint32_t mot_get_pos();

int mot_get_dir(void);
uint32_t mot_get_pos(void); // read motor position from starting point
uint32_t mot_get_tah(void); // TODO read motor tachometer (forward + backward movement)
uint16_t mot_get_pwm(void);
float mot_get_vel(void); // TODO read motor velocity (m/s)
float mot_get_avg_vel(void); // TODO read average motor vel (m/s)
float mot_get_acc(void); // TODO read motor acceleration (m/s2)

float mot_get_odometer(void);

#endif /* INC_MOTOR_H_ */

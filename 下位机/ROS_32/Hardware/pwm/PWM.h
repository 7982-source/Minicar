#ifndef __PWM_H
#define __PWM_H

void Left_PWM_Init(void);
void Right_PWM_Init(void);
void Leftspeed_set(uint16_t Compare);
void Rightspeed_set(uint16_t Compare);

#endif

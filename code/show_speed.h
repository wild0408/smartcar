#ifdef __SHOW_SPEED_H__
#define __SHOW_SPEED_H__

#include "seekfree_assistant.h"
extern seekfree_assistant_oscilloscope_struct oscilloscope_data;
void show_speed_init(void);
void show_speed_by_uart(void);

void controller_init(void);
void controller_by_uart(void);

#endif // __SHOW_SPEED_H__

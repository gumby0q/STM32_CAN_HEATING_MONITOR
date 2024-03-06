#ifndef DISPLAY_PRINTS_H_
#define DISPLAY_PRINTS_H_


/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "u8g2.h"

#define ERROR_FLAG_ON	1
#define ERROR_FLAG_OFF	0


struct display_screen1_data
{
	char str_boiler_value[10];

	char str_tempreture_value_1[10];
	char str_humidity_value_1[10];

	char str_tempreture_value_2[10];
	char str_humidity_value_2[10];

	char str_pump_status_1[10];
	char str_pump_status_2[10];
};

struct display_screen1_error_message
{
	char str_error[24];
	uint8_t error_flag;
};
//ERROR_FLAG_OFF

void display_update2(u8g2_t *p_u8g2, struct display_screen1_data *screen_data, struct display_screen1_error_message *screen_error);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* DISPLAY_PRINTS_H_ */
/** @}*/

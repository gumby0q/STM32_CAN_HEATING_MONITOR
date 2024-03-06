#include "display_prints.h"
#include <stdio.h>


void display_update2(u8g2_t *p_u8g2, struct display_screen1_data *screen_data, struct display_screen1_error_message *screen_error) {
	char tmp_string[24];
	u8g2_FirstPage(p_u8g2);

	do {
		/* boiler temperature */
		u8g2_SetFont(p_u8g2, u8g2_font_courB10_tf);
		sprintf(tmp_string, "Boiler:%s%sC", screen_data->str_boiler_value, "\xb0");
		u8g2_DrawStr(p_u8g2, 0, 10, tmp_string);

		u8g2_SetFont(p_u8g2, u8g2_font_courR08_tf);
		/* house 1 temperature */
		sprintf(tmp_string, "t:%s %sC", screen_data->str_tempreture_value_1, "\xb0");
		u8g2_DrawStr(p_u8g2, 0, 25, tmp_string);

		/* house 1 humidity */
		sprintf(tmp_string, "h:%s %%", screen_data->str_humidity_value_2);
		u8g2_DrawStr(p_u8g2, 0, 35, tmp_string);


		/* house 2 temperature */
		sprintf(tmp_string, "t:%s %sC", screen_data->str_tempreture_value_1, "\xb0");
		u8g2_DrawStr(p_u8g2, 64, 25, tmp_string);

		/* house 2 humidity */
		sprintf(tmp_string, "h:%s %%", screen_data->str_humidity_value_2);
		u8g2_DrawStr(p_u8g2, 64, 35, tmp_string);

		/* pump status 1 */
		sprintf(tmp_string, "pump1:%s", screen_data->str_pump_status_1);
		u8g2_DrawStr(p_u8g2, 0, 45, tmp_string);

		/* pump status 1 */
		sprintf(tmp_string, "pump2:%s", screen_data->str_pump_status_2);
		u8g2_DrawStr(p_u8g2, 64, 45, tmp_string);

		if (screen_error->error_flag != ERROR_FLAG_OFF) {
			/* pump status 1 */
			sprintf(tmp_string, "Error:%s", screen_error->str_error);
			u8g2_DrawStr(p_u8g2, 0, 64, tmp_string);
		}

	 } while (u8g2_NextPage(p_u8g2));
}

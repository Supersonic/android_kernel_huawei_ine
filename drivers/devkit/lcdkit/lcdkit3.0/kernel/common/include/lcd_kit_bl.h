#ifndef _LCD_KIT_BL_H_
#define _LCD_KIT_BL_H_

struct lcd_kit_bl_ops {
	int (*set_backlight)(unsigned int level);
};

/*function declare*/
struct lcd_kit_bl_ops* lcd_kit_get_bl_ops(void);
int lcd_kit_bl_register(struct lcd_kit_bl_ops* ops);
int lcd_kit_bl_unregister(struct lcd_kit_bl_ops* ops);
#endif

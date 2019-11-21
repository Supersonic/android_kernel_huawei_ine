/*
*panel adapter
*product:alps
*panel:jdi-nt36860c
*/
static int alps_jdi_nt36860c_set_backlight(struct platform_device* pdev, uint32_t bl_level)
{
	LCD_KIT_INFO("enter!\n");
	LCD_KIT_INFO("bl_level = %d\n", bl_level);
	LCD_KIT_INFO("end!\n");
}

static struct lcd_kit_panel_ops alps_jdi_nt36860c_ops = {
	.lcd_kit_read_project_id = NULL,
};

static int alps_jdi_nt36860c_proble(void)
{
	int ret = LCD_KIT_OK;

	ret = lcd_kit_panel_ops_register(&alps_jdi_nt36860c_ops);
	if (ret) {
		LCD_KIT_ERR("failed\n");
		return LCD_KIT_FAIL;
	}
	return LCD_KIT_OK;
}

/*******************************************************************************
 * COPYRIGHT (C) VITESCO TECHNOLOGIES
 * ALL RIGHTS RESERVED.
 *
 * The reproduction, transmission or use of this document or its
 * contents is not permitted without express written authority.
 * Offenders will be liable for damages. All rights, including rights
 * created by patent grant or registration of a utility model or design,
 * are reserved.
 ******************************************************************************/

#include "SRVL/SCHEDULER/scheduler.h"
#include "ASW/Climate_Control_System/climate_control_system.h"
#include "BSW/MCAL/ADC/adc.h"
#include "BSW/MCAL/GPIO/gpio.h"
#include "BSW/MCAL/PWM/pwm.h"
#include "BSW/MCAL/WIFI/wifi.h"
#include "BSW/HAL/Servo_Motor/servo_motor.h"
#include "BSW/HAL/Buzzer/buzzer.h"
#include "BSW/HAL/Shift_Register/shift_register.h"

#include "BSW/HAL/Com/com.h"
#include "BSW/HAL/Photo_Resistor/photo_resistor.h"
#include "BSW/HAL/Temp_Sensor/temp_sensor.h"

#include "RTE/rte.h"

#include "ASW/Ambient_Light/ambient_light.h"
#include "ASW/Climate_Control_System/climate_control_system.h"
#include "ASW/Headlights/headlights.h"
#include "ASW/Horn/horn.h"
#include "ASW/Locking_System/locking_system.h"
#include "ASW/Security/security.h"
#include "ASW/Trunk/trunk.h"
#include "BSW/HAL/Proximity_Sensor/proximity_sensor.h"
#include "BSW/HAL/DC_Motor/dc_motor.h"
#include "nvs_flash.h"
#include "ASW/Trunk/trunk.h"

static const char *TAG = "SRVL SCHEDULER";
uint16_t regMask = 1;
static httpd_handle_t server = NULL;
#define counter 0

void SYSTEM_vInit(void)
{
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ADC_vInit();

	GPIO_vInit();

	PWM_vInit();

	WIFI_vInit(&server);
}

void vTask100ms(void)
{

}

void vTask200ms(void)
{
	if (PROX_u16Read() < 10) {
		DCMOT_vChangeSpeed(15000);
	} else {
		DCMOT_vChangeSpeed(0);
	}/*
	if (PHRES_u16Read() < 3000 || PHRES_u16Read() > 7500) {
		DCMOT_vChangeSpeed(15000);
	} else {
		DCMOT_vChangeSpeed(0);
	}*/
	DHT11_struct dht11DataSch = DHT11_dht11Read();
	/*if (dht11DataSch.u8IntegralHum > 40 && dht11DataSch.u8IntegralHum!=255) {
		DCMOT_vChangeSpeed(15000);
	} else if(dht11DataSch.u8IntegralHum <= 40) {
		DCMOT_vChangeSpeed(0);
	}*/
	SHIFTREG_vOutput8Bits(regMask);
	regMask *=2;
	if(regMask == 256)
		regMask = 1;
}

void vTask500ms(void)
{
	SERVO_vChangeAngle(0);

}

void vTask800ms(void)
{

}

void vTask1000ms(void)
{
	/*
	ESP_LOGI(TAG, "%d", PHRES_u16Read());
	uint16_t u16PhotoValue;
	u16PhotoValue = PHRES_u16Read();
	if (u16PhotoValue < 3000) {
		GPIO_vSetLevel(13, HIGH_LEVEL);
	}
	else
	{
		GPIO_vSetLevel(13, LOW_LEVEL);
	}
	*/
}

void vTask2000ms(void) {
PHRES_vTaskCalculate();

}

void vTask5000ms(void)
{
}

void SYSTEM_vTaskScheduler(void)
{
	uint16_t u16TickCount = 0;

	while (1)
	{	
		if (u16TickCount % TASK_100MS == 0)
		{
			vTask100ms();
		}

		if (u16TickCount % TASK_200MS == 0)
		{
			vTask200ms();
		}

		if (u16TickCount % TASK_500MS == 0)
		{
			vTask500ms();
		}

		if (u16TickCount % TASK_800MS == 0)
		{
			vTask800ms();
		}

		if (u16TickCount % TASK_1000MS == 0)
		{
			vTask1000ms();
		}

		if (u16TickCount % TASK_2000MS == 0)
		{
			vTask2000ms();
		}

		if (u16TickCount % TASK_5000MS == 0)
		{
			vTask5000ms();
		}

		u16TickCount++;
		if (u16TickCount >= TASK_5000MS)
		{
			u16TickCount = 0;
		}

		vTaskDelay(100);
	}
}

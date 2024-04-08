/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rgbled_WS2812.cpp
 *
 * Driver for the onboard RGB LED controller (NCP5623B or WS2812)
 * connected via I2C.
 *
 * @author CUAVcaijie <caijie@cuav.net>
 */

#include <string.h>

#include <drivers/device/spi.h>
#include <lib/drivers/device/spi.h>
#include <lib/led/led.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/manual_control_switches.h>

using namespace time_literals;

#define WS2812_ADDR       0x39  /**< I2C address of WS2812 */

#define NCP5623_LED_CURRENT 0x20  /**< Current register */
#define NCP5623_LED_PWM0    0x40  /**< pwm0 register */
#define NCP5623_LED_PWM1    0x60  /**< pwm1 register */
#define NCP5623_LED_PWM2    0x80  /**< pwm2 register */

#define WS2812_LED_BRIGHT  255  /**< full brightness */
#define NCP5623_LED_OFF     0x00  /**< off */

#define WS2812_H 0xf8
#define WS2812_L 0xc0
#define WS2812_RED 255<<8
#define WS2812_BULE 50<<16 
#define WS2812_OFF 0
#define WS2812_LENGTH 3

class RGBLED_WS2812 : public device::SPI, public I2CSPIDriver<RGBLED_WS2812>
{
public:
	RGBLED_WS2812(const I2CSPIDriverConfig &config);
	virtual ~RGBLED_WS2812() = default;

	static void print_usage();

	int		init() override;
	int		probe() override;

	void			RunImpl();
//	virtual int8_t  get_i2c_address() {return get_device_address();}

private:
	int			send_led_rgb();

//        uORB::Subscription	_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
        uORB::Subscription	_setpoint_sub{ORB_ID(manual_control_switches)};

	float			_brightness{1.0f};

	uint8_t		_r{0};
	uint8_t		_g{0};
	uint8_t		_b{0};
	volatile bool		_running{false};
	volatile bool		_should_run{true};
	bool			_leds_enabled{true};

	LedController		_led_controller;

        uint8_t         buf[24*(WS2812_LENGTH+3)]={0};
};

RGBLED_WS2812::RGBLED_WS2812(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config)
{
//	int ordering = config.custom1;
	// ordering is RGB: Hundreds is Red, Tens is green and ones is Blue
	// 123 would drive the
	//      R LED from = NCP5623_LED_PWM0
	//      G LED from = NCP5623_LED_PWM1
	//      B LED from = NCP5623_LED_PWM2
	// 321 would drive the
	//      R LED from = NCP5623_LED_PWM2
	//      G LED from = NCP5623_LED_PWM1
	//      B LED from = NCP5623_LED_PWM0

}

int
RGBLED_WS2812::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		return ret;
	}

	_running = true;

	ScheduleNow();

	return PX4_OK;
}

int
RGBLED_WS2812::probe()
{

    if (1) {
      return OK;
    }
    return -EIO;
}

void
RGBLED_WS2812::RunImpl()
{
	LedControlData led_control_data;

	if (_led_controller.update(led_control_data) == 1) {
		switch (led_control_data.leds[0].color) {
		case led_control_s::COLOR_RED:
			_r = WS2812_LED_BRIGHT; _g = 0; _b = 0;
			break;

		case led_control_s::COLOR_GREEN:
			_r = 0; _g = WS2812_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_BLUE:
			_r = 0; _g = 0; _b = WS2812_LED_BRIGHT;
			break;

		case led_control_s::COLOR_AMBER: //make it the same as yellow
		case led_control_s::COLOR_YELLOW:
			_r = WS2812_LED_BRIGHT; _g = WS2812_LED_BRIGHT; _b = 0;
			break;

		case led_control_s::COLOR_PURPLE:
			_r = WS2812_LED_BRIGHT; _g = 0; _b = WS2812_LED_BRIGHT;
			break;

		case led_control_s::COLOR_CYAN:
			_r = 0; _g = WS2812_LED_BRIGHT; _b = WS2812_LED_BRIGHT;
			break;

		case led_control_s::COLOR_WHITE:
			_r = WS2812_LED_BRIGHT; _g = WS2812_LED_BRIGHT; _b = WS2812_LED_BRIGHT;
			break;

		default: // led_control_s::COLOR_OFF
			_r = 0; _g = 0; _b = 0;
			break;
		}

		_brightness = (float)led_control_data.leds[0].brightness / 255.f;
		send_led_rgb();

	}

	/* re-queue ourselves to run again later */
	ScheduleDelayed(_led_controller.maximum_update_interval());
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_WS2812::send_led_rgb()
{
//        uint8_t msg[7] = {0x20, 0x70, 0x40, 0x70, 0x60, 0x70, 0x80};
//	uint8_t brightness = UINT8_MAX;
//
//	msg[0] = NCP5623_LED_CURRENT | (brightness & 0x1f);
//	msg[2] = _red | (uint8_t(_r * _brightness) & 0x1f);
//	msg[4] = _green | (uint8_t(_g * _brightness) & 0x1f);
//	msg[6] = _blue | (uint8_t(_b * _brightness) & 0x1f);
//        if (_setpoint_sub.updated()){
//          vehicle_local_position_setpoint_s setpoint;
//          if (_setpoint_sub.copy(&setpoint)) {
//            if (setpoint.acceleration[0] > 5) {
//              for(int i = 0; i < 24*WS2812_LENGTH; i++) {
//                buf[i] = WS2812_H;
//              }
//            }
//          }
//        }
//if (_setpoint_sub.updated()){
//            manual_control_switches_s setpoint;
//            if (_setpoint_sub.copy(&setpoint)) {
//              if (setpoint.kill_switch == 1) {
//                for(int i = 0; i < 24*WS2812_LENGTH; i++) {
//                  buf[i] = WS2812_H;
//                }
//              }
//              else {
//                for (int i = 0; i < 24 * WS2812_LENGTH; i++) {
//                  buf[i] = WS2812_L;
//                }
//              }
//            }
//          }
        for (int j = 0 ; j < WS2812_LENGTH ; j++){
        if (_g != 0){
          for (int i = 0+24*j ; i < 8+24*j ; i++)
            {
                buf[i] = WS2812_H;
            }
        }
        else {
          for (int i = 0+24*j ; i < 8+24*j ; i++)
          {
            buf[i] = WS2812_L;
          }
        }
        if (_r != 0){
          for (int i = 8+24*j ; i < 16+24*j ; i++)
          {
            buf[i] = WS2812_H;
          }
        }
        else {
          for (int i = 8+24*j ; i < 16+24*j ; i++)
          {
            buf[i] = WS2812_L;
          }
        }
        if (_b != 0){
          for (int i = 16+24*j ; i < 24+24*j ; i++)
          {
            buf[i] = WS2812_H;
          }
        }
        else {
          for (int i = 16+24*j ; i < 24+24*j ; i++)
          {
            buf[i] = WS2812_L;
          }
        }
      }
	return transfer(&buf[0], nullptr, 24*(WS2812_LENGTH+3));
}

void
RGBLED_WS2812::print_usage()
{
	PRINT_MODULE_USAGE_NAME("rgbled", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
//	PRINT_MODULE_USAGE_PARAM_INT('o', 123, 123, 321, "RGB PWM Assignment", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int rgbled_ws2812_main(int argc, char *argv[])
{
	using ThisDriver = RGBLED_WS2812;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 8000000;
	int ch;

        while ((ch = cli.getOpt(argc, argv, "o:")) != EOF) {
          switch (ch) {
          case 'o':
            cli.custom1 = atoi(cli.optArg());
            break;
          }
        }


	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli,
				     DRV_LED_DEVTYPE_RGBLED_WS2812);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

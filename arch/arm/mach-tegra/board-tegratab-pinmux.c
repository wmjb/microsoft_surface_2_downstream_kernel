/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/gpio-tegra.h>
#include "board.h"
#include "board-tegratab.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra-board-id.h"

#include <mach/pinmux-t11.h>

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive,	\
		_pullup_drive, _pulldn_slew, _pullup_slew)	\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_##_hsm,			\
		.schmitt = TEGRA_SCHMITT_##_schmitt,		\
		.drive = TEGRA_DRIVE_##_drive,			\
		.pull_down = TEGRA_PULL_##_pulldn_drive,	\
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,	\
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * drive_type - Drive type to be used depending on the resistors.
 */

#define SET_DRIVE_WITH_TYPE(_name, _hsm, _schmitt, _drive, _pulldn_drive,\
		_pullup_drive, _pulldn_slew, _pullup_slew, _drive_type)	\
	{								\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,		\
		.hsm = TEGRA_HSM_##_hsm,				\
		.schmitt = TEGRA_SCHMITT_##_schmitt,			\
		.drive = TEGRA_DRIVE_##_drive,				\
		.pull_down = TEGRA_PULL_##_pulldn_drive,		\
		.pull_up = TEGRA_PULL_##_pullup_drive,			\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,		\
		.slew_falling = TEGRA_SLEW_##_pullup_slew,		\
		.drive_type = TEGRA_DRIVE_TYPE_##_drive_type,		\
	}

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define DDC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _rcv_sel) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.rcv_sel	= TEGRA_PIN_RCV_SEL_##_rcv_sel,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

#define CEC_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od)   \
	{                                                               \
		.pingroup   = TEGRA_PINGROUP_##_pingroup,                   \
		.func       = TEGRA_MUX_##_mux,                             \
		.pupd       = TEGRA_PUPD_##_pupd,                           \
		.tristate   = TEGRA_TRI_##_tri,                             \
		.io         = TEGRA_PIN_##_io,                              \
		.lock       = TEGRA_PIN_LOCK_##_lock,                       \
		.od         = TEGRA_PIN_OD_##_od,                           \
		.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,                   \
	}

#define GPIO_PINMUX(_pingroup, _pupd, _tri, _io, _od)   \
	{                                                   \
		.pingroup   = TEGRA_PINGROUP_##_pingroup,       \
		.func       = TEGRA_MUX_SAFE,                   \
		.pupd       = TEGRA_PUPD_##_pupd,               \
		.tristate   = TEGRA_TRI_##_tri,                 \
		.io         = TEGRA_PIN_##_io,                  \
		.lock       = TEGRA_PIN_LOCK_DEFAULT,           \
		.od         = TEGRA_PIN_OD_##_od,               \
		.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,       \
	}

#define UNUSED_PINMUX(_pingroup)                    \
	{                                               \
		.pingroup   = TEGRA_PINGROUP_##_pingroup,   \
		.func       = TEGRA_MUX_SAFE,               \
		.pupd       = TEGRA_PUPD_PULL_DOWN,         \
		.tristate   = TEGRA_TRI_TRISTATE,           \
		.io         = TEGRA_PIN_OUTPUT,             \
		.lock       = TEGRA_PIN_LOCK_DEFAULT,       \
		.od         = TEGRA_PIN_OD_DEFAULT,         \
		.ioreset    = TEGRA_PIN_IO_RESET_DEFAULT,   \
	}

#define USB_PINMUX CEC_PINMUX

#define GPIO_INIT_PIN_MODE(_gpio, _is_input, _value)	\
	{					\
		.gpio_nr	= _gpio,	\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

static __initdata struct tegra_drive_pingroup_config tegratab_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 36, 20, SLOW, SLOW),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 22, 36, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 2, 2, FASTEST,
								FASTEST, 1),
};

#include "board-tegratab-pinmux-t11x.h"

/* THIS IS FOR TESTING OR WORKAROUND PURPOSES. ANYTHING INSIDE THIS TABLE
 * SHOULD BE PUSHED TO PINMUX SPREADSHEET FOR AUTOGEN OR FIXED
 * */
static __initdata struct tegra_pingroup_config manual_config_pinmux[] = {

	/* ULPI SFIOs are not supposed to be supported.
	 * This setting is only for Tegratab. */
	DEFAULT_PINMUX(ULPI_DATA0,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA1,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA5,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA6,    ULPI,        NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(ULPI_DATA7,    ULPI,        NORMAL,    NORMAL,   INPUT),
};

static __initdata struct tegra_pingroup_config dvt_a00_manual_config_pinmux[] = {
	GPIO_PINMUX(KB_COL1, PULL_UP, NORMAL, INPUT, DISABLE), /* hall sensor input */
};

static struct gpio_init_pin_info dvt_a00_manual_gpio_mode[] = {
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PQ1, true, 0), /* hall sensor input */
};

static void __init tegratab_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_P1640) {
		len = ARRAY_SIZE(init_gpio_mode_tegratab_ffd_common);
		pins_info = init_gpio_mode_tegratab_ffd_common;
	} else { /* ERS */
		len = ARRAY_SIZE(init_gpio_mode_tegratab_common);
		pins_info = init_gpio_mode_tegratab_common;
	}

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}

	if (board_info.board_id == BOARD_P1640) {
		len = ARRAY_SIZE(dvt_a00_manual_gpio_mode);
		pins_info = dvt_a00_manual_gpio_mode;
		for (i = 0; i < len; ++i) {
			tegra_gpio_init_configure(pins_info->gpio_nr,
				pins_info->is_input, pins_info->value);
			pins_info++;
		}
	}

}

int __init tegratab_pinmux_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	tegratab_gpio_init_configure();

	tegra_drive_pinmux_config_table(tegratab_drive_pinmux,
					ARRAY_SIZE(tegratab_drive_pinmux));

	if (board_info.board_id == BOARD_P1640) {
		tegra_pinmux_config_table(tegratab_ffd_pinmux_common,
					ARRAY_SIZE(tegratab_ffd_pinmux_common));
		tegra_pinmux_config_table(ffd_unused_pins_lowpower,
					ARRAY_SIZE(ffd_unused_pins_lowpower));
		tegra_pinmux_config_table(dvt_a00_manual_config_pinmux,
					ARRAY_SIZE(dvt_a00_manual_config_pinmux));
	} else { /* ERS */
		tegra_pinmux_config_table(tegratab_pinmux_common,
					ARRAY_SIZE(tegratab_pinmux_common));
		tegra_pinmux_config_table(unused_pins_lowpower,
					ARRAY_SIZE(unused_pins_lowpower));
	}

	tegra_pinmux_config_table(manual_config_pinmux,
		ARRAY_SIZE(manual_config_pinmux));

	return 0;
}

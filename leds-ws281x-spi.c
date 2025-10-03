// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Chris Morgan <macromorgan@hotmail.com>
 *
 * Driver for controlling one or more WS2812B LEDs over SPI.
 * This driver works by sending SPI packets with precise timing
 * that are able to effectively emulate the signals required by
 * the LED controller.
 *
 * Datasheet: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
 *
 */

#include <linux/led-class-multicolor.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

/**
 * struct ws281x_info - Chip specific information. This information may
 *	vary depending upon different ws281x controllers.
 *
 * @zero_val: SPI byte interpreted as 0 by ws281x hardware.
 * @one_val: SPI byte interpreted as 1 by ws281x hardware.
 * @write_freq: SPI write frequency required for ws281x hardware.
 * Should be 8x the frequency of the specific chip (typically 400Khz
 * or 800Khz).
 * @subpixel_sz: Length in bits of subpixel data.
 * @ch_per_led: Number of subpixels. Should be 3 (RGB) or 4 (RGBW).
 * @pixel_sz: Total size of pixel information. Should be
 * (subpixel_sz * ch_per_led).
 */
struct ws281x_chipinfo {
	u8				zero_val;
	u8				one_val;
	u32				write_freq;
	u8				subpixel_sz;
	u8				ch_per_led;
	u8				pixel_sz;
};

/**
 * struct ws281x_led - Per LED data structure.
 *
 * @parent: Pointer to ws281x_array struct.
 * @led: led_classdev_mc struct containing LED specific info.
 */
struct ws281x_led {
	struct ws281x_array		*parent;
	struct led_classdev_mc		led;
};

/**
 * struct ws281x_array - ws281x private driver information containing
 * info required by driver at runtime.
 *
 * @dev: Pointer to device for this hardware.
 * @spi: Pointer to SPI device used for control signals.
 * @mutex: Mutex used to keep writes ordered.
 * @info: Pointer to hardware specific information.
 * @pixelstream: Pointer to buffer which stores the stream of specially
 * formatted data written directly to the SPI hardware.
 * @num_leds: Number of controllable LEDs.
 * @leds: Array of individual LED structs.
 */
struct ws281x_array {
	struct device			*dev;
	struct spi_device		*spi;
	struct mutex			mutex;
	const struct ws281x_chipinfo	*info;
	unsigned char			*pixelstream;
	u32				num_leds;
	struct ws281x_led		leds[] __counted_by(num_leds);
};

/**
 * ws281x_format_subpixel() - format the subpixel data
 * @ws281x: Driver data.
 * @subpixel_buf: A pre-allocated buffer to contain formatted subpixel
 * data
 * @pixel: An 8-bit subpixel value.
 *
 * Convert the 8 bit subpixel value into a packet that can be
 * understood by the ws821x starting with the MSB.
 */
static void ws281x_format_subpixel(struct ws281x_array *ws281x,
				   char *subpixel_buf, unsigned char pixel)
{
	int i = 0;

	for (i = 0; i < ws281x->info->subpixel_sz; i++) {
		subpixel_buf[i] = (pixel & 0x80) ? \
			ws281x->info->one_val : ws281x->info->zero_val;
		pixel <<= 1;
	}
}

/**
 * ws2812_format_pixel_grb() - combine r, g, and b values into a single
 * packet
 * @ws281x: Driver data.
 * @pixel_buf: A pre-allocated buffer to contain formatted pixel
 * data
 * @r: An 8-bit subpixel value for red.
 * @g: An 8-bit subpixel value for green.
 * @b: An 8-bit subpixel value for blue.
 *
 * Convert the 3 subpixel values into a full pixel packet that can be
 * understood by the ws821x. The ws2812b requires data in green, red,
 * blue order.
 */
static void ws2812_format_pixel_grb(struct ws281x_array *ws281x,
				    unsigned char *pixel_buf,
				    unsigned char g, unsigned char r,
				    unsigned char b)
{
	ws281x_format_subpixel(ws281x, pixel_buf, g);
	pixel_buf += ws281x->info->subpixel_sz;
	ws281x_format_subpixel(ws281x, pixel_buf, r);
	pixel_buf += ws281x->info->subpixel_sz;
	ws281x_format_subpixel(ws281x, pixel_buf, b);
}

/**
 * ws281x_write() - Write the active pixel buffer via SPI to the LEDs
 * @ws281x: Driver data.
 *
 * Write the active pixelstream from the driver data to the LEDs via
 * the SPI bus to update the LEDs.
 *
 * Return: 0 on success or error on failure.
 */
static int ws281x_write(struct ws281x_array *ws281x)
{
	struct spi_device *spi = ws281x->spi;
	struct spi_message spi_msg;
	struct spi_transfer xfers;
	int ret;

	spi_message_init(&spi_msg);
	memset(&xfers, 0, sizeof(xfers));

	xfers.tx_buf = ws281x->pixelstream;
	xfers.len = (ws281x->info->pixel_sz * ws281x->num_leds);
	spi_message_add_tail(&xfers, &spi_msg);

	ret = spi_sync(spi, &spi_msg);
	if (ret) {
		dev_err(ws281x->dev, "spi transfer error: %d", ret);
		return ret;
	}

	return 0;
}

/**
 * ws281x_update_pixelstream() - Update the pixelstream data to write
 * for the LEDs.
 * @ws281x: Driver data.
 *
 * Iterate through every LED to write the appropriate values to the
 * pixelstream buffer inside the driver data.
 *
 * Return: 0 on success or error on failure.
 */
static void ws281x_update_pixelstream(struct ws281x_array *ws281x)
{
	unsigned char *pixelstream = ws281x->pixelstream;
	int i;

	for (i = 0; i < ws281x->num_leds; i++) {
		ws2812_format_pixel_grb(ws281x, pixelstream,
				 ws281x->leds[i].led.subled_info[1].brightness,
				 ws281x->leds[i].led.subled_info[0].brightness,
				 ws281x->leds[i].led.subled_info[2].brightness);
		pixelstream += ws281x->info->pixel_sz;
	}
}

/**
 * ws281x_brightness_set_blocking() - Update the subpixel data for an
 * LED
 * @dev: Pointer to led_classdev of LED being updated.
 * @brightness: Brightness value to write to LED.
 *
 * Convert the brightness information into the individual color
 * components for the updated LED and update the LED with the required
 * values. Then, update the pixelstream with the data to format all LEDs.
 * Lastly, write the entire pixelstream to update the individual pixel that
 * has changed.
 *
 * Return: 0 for success or error for failure.
 */
static int ws281x_brightness_set_blocking(struct led_classdev *dev, enum led_brightness brightness)
{
	struct led_classdev_mc *mc_cdev = lcdev_to_mccdev(dev);
	struct ws281x_led *ws281x_led = container_of(mc_cdev, struct ws281x_led, led);
	struct ws281x_array *ws281x = ws281x_led->parent;
	int ret = 0;

	led_mc_calc_color_components(mc_cdev, brightness);
	mutex_lock(&ws281x->mutex);
	ws281x_update_pixelstream(ws281x);
	ret = ws281x_write(ws281x);
	mutex_unlock(&ws281x->mutex);

	return ret;
}

/**
 * ws281x_register_leds() - Register each individual LED
 * @dev: Pointer to parent device.
 * @ws281x: Driver data.
 *
 * Iterate through each defined LED and register it as a multicolor LED.
 *
 * Return: 0 for success or error for failure.
 */
static int ws281x_register_leds(struct device *dev,
				struct ws281x_array *ws281x)
{
	struct fwnode_handle *parent_node = dev_fwnode(ws281x->dev);
	struct fwnode_handle *child_node;
	int ret;
	int num = 0;

	fwnode_for_each_child_node(parent_node, child_node) {
		struct mc_subled *mc_led_info;
		struct led_init_data init_data = {};

		mc_led_info = devm_kmalloc_array(dev, ws281x->info->ch_per_led,
						 sizeof(*mc_led_info),
						 GFP_KERNEL);
		init_data.fwnode = child_node;

		mc_led_info[0].color_index = LED_COLOR_ID_RED;
		mc_led_info[1].color_index = LED_COLOR_ID_GREEN;
		mc_led_info[2].color_index = LED_COLOR_ID_BLUE;

		ws281x->leds[num].parent = ws281x;
		ws281x->leds[num].led.subled_info = mc_led_info;
		ws281x->leds[num].led.num_colors = ws281x->info->ch_per_led;
		ws281x->leds[num].led.led_cdev.brightness = LED_OFF;
		ws281x->leds[num].led.led_cdev.max_brightness = LED_FULL;
		ws281x->leds[num].led.led_cdev.brightness_set_blocking = \
			ws281x_brightness_set_blocking;

		ret = devm_led_classdev_multicolor_register_ext(ws281x->dev,
								&ws281x->leds[num].led,
								&init_data);
		if (ret)
			return ret;
		num++;
	}

	return 0;
}

static int ws281x_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ws281x_array *ws281x;
	size_t count;
	int ret;

	count = device_get_child_node_count(&spi->dev);
	if (!count)
		return dev_err_probe(dev, -EINVAL,
				     "No LEDs defined for control\n");

	ws281x = devm_kzalloc(&spi->dev,
			      struct_size(ws281x, leds, count), GFP_KERNEL);
	if (!ws281x)
		return -ENOMEM;

	ws281x->num_leds = count;
	ws281x->dev = dev;
	ws281x->info = device_get_match_data(dev);
	spi_set_drvdata(spi, ws281x);

	ret = devm_mutex_init(dev, &ws281x->mutex);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Could not get mutex\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = ws281x->info->write_freq;

	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Unable to set up SPI for ws281x\n");

	ws281x->pixelstream = devm_kcalloc(&spi->dev,
					   (ws281x->info->pixel_sz * count),
					   sizeof(uint8_t), GFP_KERNEL);
	if (!ws281x->pixelstream)
		return -ENOMEM;

	ws281x->spi = spi;

	ret = ws281x_register_leds(&spi->dev, ws281x);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Cannot register LEDs\n");

	return 0;
}

/*
 * The datasheet for the ws2812b defines a 0 as high for 0.4us and low
 * for 0.85us, and a 1 as high for 0.8us and low for 0.4us. By setting
 * our transfer rate to 6.4Mhz (8 times the 800KHz refresh rate) this
 * allows an SPI write of 0xc0 (11000000) to be interpreted as a 0 and
 * 0xfc (11111100) to be interpreted as a 1.
 */
static const struct ws281x_chipinfo ws2812b_info = {
	.zero_val = 0xc0,
	.one_val = 0xfc,
	.write_freq = 6400000,
	.subpixel_sz = BITS_PER_BYTE,
	.ch_per_led = 3,
	.pixel_sz = (BITS_PER_BYTE * 3),
};

static const struct of_device_id ws281x_spi_dt_ids[] = {
	{ .compatible = "worldsemi,ws2812b-spi", .data = &ws2812b_info },
	{},
};
MODULE_DEVICE_TABLE(of, ws281x_spi_dt_ids);

static const struct spi_device_id ws281x_spi_ids[] = {
	{ "ws2812b-spi", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, ws281x_spi_ids);

static struct spi_driver ws281x_spi_driver = {
	.probe			= ws281x_spi_probe,
	.driver			= {
		.name		= KBUILD_MODNAME,
		.of_match_table	= ws281x_spi_dt_ids,
	},
	.id_table		= ws281x_spi_ids,
};
module_spi_driver(ws281x_spi_driver);

MODULE_AUTHOR("Chris Morgan <macromorgan@hotmail.com>");
MODULE_DESCRIPTION("WS281x Over SPI LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:ws281x-spi");

/*
 * libtinyiiod - Tiny IIO Daemon Library
 *
 * Copyright (C) 2016 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */
#include <inttypes.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "tinyiiod.h"
#include "tinyiiod_user.h"
#include "axi_adc_core.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "ad9361_parameters.h"
#include "util.h"
#include "xil_io.h"
#include "ad9361_api.h"

#ifdef UART_INTERFACE
#include "serial.h"
#endif // UART_INTERFACE
#ifdef TCPIP_INTERFACE
#include "network.h"
#endif // TCPIP_INTERFACE
#include "xil_cache.h"
#include "platform_drivers.h"
static uint32_t request_mask;
// mask for cf-ad9361-lpc 0x0F, it has 4 channels
static uint32_t input_channel_mask = 0x0F;
extern struct ad9361_rf_phy *ad9361_phy;

static const char * const ad9361_agc_modes[] =
{"manual", "fast_attack", "slow_attack", "hybrid"};
static const char * const ad9361_rf_rx_port[] = {
	"A_BALANCED", "B_BALANCED", "C_BALANCED",
	"A_N", "A_P", "B_N", "B_P", "C_N", "C_P", "TX_MONITOR1",
	"TX_MONITOR2", "TX_MONITOR1_2"
};
static const char * const ad9361_rf_tx_port[] =
{"A", "B"};
static const char * const ad9361_calib_mode[] =
{"auto", "manual", "tx_quad", "rf_dc_offs", "rssi_gain_step"};

/**
 * Compare two strings
 * @param *str1 pointer to string 1
 * @param *str2 pointer to string 2
 * @return TRUE if strings are equal, 0 otherwise
 */
static bool strequal(const char *str1, const char *str2)
{
	return !strcmp(str1, str2);
}

/**
 * Check device
 * @param *device
 * @return TRUE if valid device is found
 */
static bool dev_is_ad9361_module(const char *device)
{
	return strequal(device, "ad9361-phy")
	       || strequal(device, "cf-ad9361-lpc")
	       || strequal(device, "cf-ad9361-dds-core-lpc");
}

/**
 * Converts from string to long value
 * @param *str
 * @return long value
 */
static long read_value(const char *str)
{
	char *end;
	int32_t value = strtol(str, &end, 0);

	if (end == str)
		return -EINVAL;
	else
		return value;
}

/**
 * Converts from string to unsigned long value
 * @param *str
 * @return long value
 */
static unsigned long read_ul_value(const char *str)
{
	char *end;
	uint32_t value = strtoul(str, &end, 0);

	if (end == str)
		return -EINVAL;
	else
		return value;
}

/**
 * Get channel number
 * @param *ch channel name + channel number
 * @param *ch_name channel name
 * @return channel number
 */
int32_t get_channel(char *ch, char *ch_name)
{
	char *p = strstr(ch, ch_name);
	if(p == NULL) {
		return -ENODEV;
	} else {
		p += strlen(ch_name);
		return read_value(p);
	}
}

extern int32_t ad9361_spi_read(struct spi_desc *spi, uint32_t reg);
extern int32_t ad9361_spi_write(struct spi_desc *spi, uint32_t reg,
				uint32_t val);

/**
 * Write register
 * @param addr
 * @param value
 * @return 0 on success, negative value on failure
 */
static int32_t write_reg(uint32_t addr, uint32_t value)
{
	return ad9361_spi_write(ad9361_phy->spi, addr, value);
}

/**
 * Read register
 * @param addr
 * @param *value
 * @return 0 on success, negative value on failure
 */
static int32_t read_reg(uint32_t addr, int32_t *value)
{
	*value = ad9361_spi_read(ad9361_phy->spi, addr);
	if(*value < 0)
		return *value;

	return 0;
}

extern const char *ad9361_ensm_states[12];
extern int32_t ad9361_parse_fir(struct ad9361_rf_phy *phy,
				char *data, uint32_t size);

struct channel_info {
	int32_t ch_num;
	bool ch_out;
};

struct attrtibute_map {
	char *attr_name;
	ssize_t (*exec)(char *buf, size_t len, const struct channel_info *channel);
};

/**
 * Get attribute ID from map based on attribute name
 * @param attr* attribute name
 * @param *map pointer to map
 * @param map_size map_size element numbers of the map
 * @return attribute ID, or negative value if attribute is not found
 */
int16_t get_attribute_id(const char *attr, const struct attrtibute_map* map,
			 int32_t map_size)
{
	int16_t i;
	for(i = 0; i < map_size; i++) {
		if (strequal(attr, map[i].attr_name )) {
			break;
		}
	}
	if(i >= map_size) {
		i = -ENODEV;
	}

	return i;
}

/**
 * get_dcxo_tune_coarse
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dcxo_tune_coarse(char *buf, size_t len,
			     const struct channel_info *channel)
{

	if (ad9361_phy->pdata->use_extclk)
		return -ENODEV;
	else
		return sprintf(buf, "%d", (int)ad9361_phy->pdata->dcxo_coarse);
}

/**
 * get_rx_path_rates
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rx_path_rates(char *buf, size_t len,
			  const struct channel_info *channel)
{
	unsigned long clk[6];
	ad9361_get_trx_clock_chain(ad9361_phy, clk, NULL);
	return sprintf(buf,
		       "BBPLL:%"PRIu32" ADC:%"PRIu32" R2:%"PRIu32" R1:%"PRIu32" RF:%"PRIu32" RXSAMP:%"PRIu32"",
		       clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
}

/**
 * get_trx_rate_governor
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_trx_rate_governor(char *buf, size_t len,
			      const struct channel_info *channel)
{
	uint32_t rate_governor;
	ad9361_get_trx_rate_gov (ad9361_phy, &rate_governor);

	return sprintf(buf, "%s", rate_governor ? "nominal" : "highest_osr");
}

/**
 * get_calib_mode_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_calib_mode_available(char *buf, size_t len,
				 const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%s %s %s %s %s", ad9361_calib_mode[0],
				 ad9361_calib_mode[1], ad9361_calib_mode[2],
				 ad9361_calib_mode[3], ad9361_calib_mode[4]);
}

/**
 * get_xo_correction_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_xo_correction_available(char *buf, size_t len,
				    const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%d", 0); //dummy
}

/**
 * get_gain_table_config
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_gain_table_config(char *buf, size_t len,
			      const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%d", 0); //dummy
}

/**
 * get_dcxo_tune_fine
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dcxo_tune_fine(char *buf, size_t len,
			   const struct channel_info *channel)
{
	if (ad9361_phy->pdata->use_extclk)
		return -ENODEV;
	else
		return sprintf(buf, "%d", (int)ad9361_phy->pdata->dcxo_fine);
}

/**
 * get_dcxo_tune_fine_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dcxo_tune_fine_available(char *buf, size_t len,
				     const struct channel_info *channel)
{
	return sprintf(buf, "%s",
		       ad9361_phy->pdata->use_extclk ? "[0 0 0]" : "[0 1 8191]");
}

/**
 * get_ensm_mode_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_ensm_mode_available(char *buf, size_t len,
				const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%s", ad9361_phy->pdata->fdd ?
				 "sleep wait alert fdd pinctrl pinctrl_fdd_indep" :
				 "sleep wait alert rx tx pinctrl");
}

/**
 * get_multichip_sync
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_multichip_sync(char *buf, size_t len,
			   const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%d", 0); //dummy
}

/**
 * get_rssi_gain_step_error
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rssi_gain_step_error(char *buf, size_t len,
				 const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%d", 0); //dummy
}

/**
 * get_dcxo_tune_coarse_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dcxo_tune_coarse_available(char *buf, size_t len,
				       const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%s",
				 ad9361_phy->pdata->use_extclk ? "[0 0 0]" : "[0 1 63]");
}

/**
 * get_tx_path_rates
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_tx_path_rates(char *buf, size_t len,
			  const struct channel_info *channel)
{
	unsigned long clk[6];
	ad9361_get_trx_clock_chain(ad9361_phy, NULL, clk);

	return sprintf(buf, "BBPLL:%lu DAC:%lu T2:%lu T1:%lu TF:%lu TXSAMP:%lu",
		       clk[0], clk[1], clk[2], clk[3], clk[4], clk[5]);
}

/**
 * get_trx_rate_governor_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_trx_rate_governor_available(char *buf, size_t len,
					const struct channel_info *channel)
{
	return sprintf(buf, "%s", "nominal highest_osr");
}

/**
 * get_xo_correction
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_xo_correction(char *buf, size_t len,
			  const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%d", 0); //dummy
}

/**
 * get_ensm_mode
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_ensm_mode(char *buf, size_t len,
		      const struct channel_info *channel)
{
	ssize_t ret = 0;
	ret = ad9361_ensm_get_state(ad9361_phy);
	if (ret < 0)
		return ret;
	if (ret >= ARRAY_SIZE(ad9361_ensm_states) ||
	    ad9361_ensm_states[ret] == NULL) {
		return -EIO;
	}

	return sprintf(buf, "%s", ad9361_ensm_states[ret]);
}

/**
 * get_filter_fir_config
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_filter_fir_config(char *buf, size_t len,
			      const struct channel_info *channel)
{
	return sprintf(buf, "FIR Rx: %d,%d Tx: %d,%d",
		       ad9361_phy->rx_fir_ntaps, ad9361_phy->rx_fir_dec,
		       ad9361_phy->tx_fir_ntaps, ad9361_phy->tx_fir_int);
}

/**
 * get_calib_mode
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_calib_mode(char *buf, size_t len,
		       const struct channel_info *channel)
{
	uint8_t en_dis;
	ad9361_get_tx_auto_cal_en_dis(ad9361_phy, &en_dis);

	return (ssize_t) snprintf(buf, len, "%s", en_dis ? "auto" : "manual");
}

/**
 * read all attributes from map
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @param *map map with coresponding attributes
 * @param map_size
 * @return length of chars written in buf
 */
static ssize_t read_all_attr(char *buf, size_t len,
			     const struct channel_info *channel, const struct attrtibute_map* map,
			     int32_t map_size)
{
	int16_t i, j = 0;
	char local_buf[0x1000];
	for(i = 0; i < map_size; i++) {

		int16_t attr_length = map[i].exec((local_buf), len, channel);
		int32_t *len = (int32_t *)(buf + j);
		*len = Xil_EndianSwap32(attr_length);

		j += 4;
		if(attr_length >= 0) {
			sprintf(buf + j, "%s", local_buf);
			if (attr_length & 0x3) //multiple of 4
				attr_length = ((attr_length >> 2) + 1) << 2;
			j += attr_length;
		}
	}

	return j;
}

/**
 * write all attributes from map
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @param *map map with coresponding attributes
 * @param map_size
 * @return length of chars written in buf
 */
static ssize_t write_all_attr(char *buf, size_t len,
			      const struct channel_info *channel, const struct attrtibute_map* map,
			      int32_t map_size)
{
	return 0;
}

static struct attrtibute_map global_read_attrtibute_map[] = {
	{"dcxo_tune_coarse", get_dcxo_tune_coarse},
	{"rx_path_rates", get_rx_path_rates},
	{"trx_rate_governor", get_trx_rate_governor},
	{"calib_mode_available", get_calib_mode_available},
	{"xo_correction_available", get_xo_correction_available},
	{"gain_table_config", get_gain_table_config},
	{"dcxo_tune_fine", get_dcxo_tune_fine},
	{"dcxo_tune_fine_available", get_dcxo_tune_fine_available},
	{"ensm_mode_available", get_ensm_mode_available},
	{"multichip_sync", get_multichip_sync},
	{"rssi_gain_step_error", get_rssi_gain_step_error},
	{"dcxo_tune_coarse_available", get_dcxo_tune_coarse_available},
	{"tx_path_rates", get_tx_path_rates},
	{"trx_rate_governor_available", get_trx_rate_governor_available},
	{"xo_correction", get_xo_correction},
	{"ensm_mode", get_ensm_mode},
	{"filter_fir_config", get_filter_fir_config},
	{"calib_mode", get_calib_mode},
};

/**
 * read global attribute
 * @param *device name
 * @param *attr name
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param debug
 * @return length of chars written in buf
 */
static ssize_t read_attr(const char *device, const char *attr,
			 char *buf, size_t len, bool debug)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;
	if(strequal(device, "ad9361-phy")) {
		int16_t attribute_id = get_attribute_id(attr, global_read_attrtibute_map,
							ARRAY_SIZE(global_read_attrtibute_map));
		if(attribute_id >= 0) {
			return global_read_attrtibute_map[attribute_id].exec(buf, len, NULL);
		}
		if(strequal(attr, "")) {
			return read_all_attr(buf, len, NULL, global_read_attrtibute_map,
					     ARRAY_SIZE(global_read_attrtibute_map));
		}
		return -ENOENT;
	} else if(strequal(device, "cf-ad9361-dds-core-lpc")) {

	} else if(strequal(device, "cf-ad9361-lpc")) {

	}

	return -ENODEV;
}

/**
 * set_trx_rate_governor
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_trx_rate_governor(char *buf, size_t len,
			      const struct channel_info *channel)
{
	ssize_t ret = 0;
	if(strequal(buf, "nominal")) {
		ad9361_set_trx_rate_gov (ad9361_phy, 1);
	} else if(strequal(buf, "highest_osr")) {
		ad9361_set_trx_rate_gov (ad9361_phy, 0);
	} else
		ret =  -ENOENT;

	return ret;
}

/**
 * set_dcxo_tune_coarse
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dcxo_tune_coarse(char *buf, size_t len,
			     const struct channel_info *channel)
{
	ssize_t ret = 0;
	uint32_t dcxo_coarse = read_ul_value(buf);
	dcxo_coarse = clamp_t(uint32_t, dcxo_coarse, 0, 63U);
	ad9361_phy->pdata->dcxo_coarse = dcxo_coarse;
	ret = ad9361_set_dcxo_tune(ad9361_phy, ad9361_phy->pdata->dcxo_coarse,
				   ad9361_phy->pdata->dcxo_fine);

	return ret;
}

/**
 * set_dcxo_tune_fine
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dcxo_tune_fine(char *buf, size_t len,
			   const struct channel_info *channel)
{
	ssize_t ret = 0;
	uint32_t dcxo_fine = read_ul_value(buf);
	dcxo_fine = clamp_t(uint32_t, dcxo_fine, 0, 8191U);
	ad9361_phy->pdata->dcxo_fine = dcxo_fine;
	ret = ad9361_set_dcxo_tune(ad9361_phy, ad9361_phy->pdata->dcxo_coarse,
				   ad9361_phy->pdata->dcxo_fine);

	return ret;
}

/**
 * set_calib_mode
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_calib_mode(char *buf, size_t len,
		       const struct channel_info *channel)
{
	int32_t arg = -1;
	ssize_t ret = 0;
	u32 val = 0;
	val = 0;
	if (strequal(buf, "auto")) {
		ad9361_set_tx_auto_cal_en_dis (ad9361_phy, 1);
	} else if (strequal(buf, "manual")) {
		ad9361_set_tx_auto_cal_en_dis (ad9361_phy, 0);
	} else if (!strncmp(buf, "tx_quad", 7)) {
		ret = sscanf(buf, "tx_quad %"PRIi32, &arg);
		if (ret != 1)
			arg = -1;
		val = TX_QUAD_CAL;
	} else if (strequal(buf, "rf_dc_offs"))
		val = RFDC_CAL;
	else if (strequal(buf, "rssi_gain_step"))
		ret = ad9361_rssi_gain_step_calib(ad9361_phy);
	else
		return -ENOENT;

	if (val)
		ret = ad9361_do_calib(ad9361_phy, val, arg);

	return ret ? ret : len;
}

/**
 * set_ensm_mode
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_ensm_mode(char *buf, size_t len,
		      const struct channel_info *channel)
{
	ssize_t ret = 0;
	u32 val = 0;
	bool res = false;
	ad9361_phy->pdata->fdd_independent_mode = false;

	if (strequal(buf, "tx"))
		val = ENSM_STATE_TX;
	else if (strequal(buf, "rx"))
		val = ENSM_STATE_RX;
	else if (strequal(buf, "alert"))
		val = ENSM_STATE_ALERT;
	else if (strequal(buf, "fdd"))
		val = ENSM_STATE_FDD;
	else if (strequal(buf, "wait"))
		val = ENSM_STATE_SLEEP_WAIT;
	else if (strequal(buf, "sleep"))
		val = ENSM_STATE_SLEEP;
	else if (strequal(buf, "pinctrl")) {
		res = true;
		val = ENSM_STATE_SLEEP_WAIT;
	} else if (strequal(buf, "pinctrl_fdd_indep")) {
		val = ENSM_STATE_FDD;
		ad9361_phy->pdata->fdd_independent_mode = true;
	} else
		return -ENOENT;

	ad9361_set_ensm_mode(ad9361_phy, ad9361_phy->pdata->fdd, res);
	ret = ad9361_ensm_set_state(ad9361_phy, val, res);

	return ret;
}

/**
 * set_multichip_sync
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_multichip_sync(char *buf, size_t len,
			   const struct channel_info *channel)
{
	ssize_t ret = 0;
	uint32_t readin = read_ul_value(buf);
	ret = ad9361_mcs(ad9361_phy, readin);

	return ret;
}

/**
 * set_filter_fir_config
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_filter_fir_config(char *buf, size_t len,
			      const struct channel_info *channel)
{
	return ad9361_parse_fir(ad9361_phy, (char *)buf, len);
}

static struct attrtibute_map global_write_attrtibute_map[] = {
	{"trx_rate_governor", set_trx_rate_governor},
	{"dcxo_tune_coarse", set_dcxo_tune_coarse},
	{"dcxo_tune_fine", set_dcxo_tune_fine},
	{"calib_mode", set_calib_mode},
	{"ensm_mode", set_ensm_mode},
	{"multichip_sync", set_multichip_sync},
	{"filter_fir_config", set_filter_fir_config},
};

/**
 * write global attribute
 * @param *device name
 * @param *attr name
 * @param *buff where value is stored
 * @param len length of the value
 * @debug
 * @return length of chars written to attribute, negative value in case of failure
 */
static ssize_t write_attr(const char *device, const char *attr,
			  const char *buf, size_t len, bool debug)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;
	if(strequal(device, "ad9361-phy")) {
		int16_t attribute_id = get_attribute_id(attr, global_write_attrtibute_map,
							ARRAY_SIZE(global_write_attrtibute_map));
		if(attribute_id >= 0) {
			return global_write_attrtibute_map[attribute_id].exec((char*)buf, len, NULL);
		}
		if(strequal(attr, "")) {
			return write_all_attr((char*)buf, len, NULL, global_write_attrtibute_map,
					      ARRAY_SIZE(global_write_attrtibute_map));
		}
		return -ENOENT;
	} else if(strequal(device, "cf-ad9361-dds-core-lpc")) {

	} else if(strequal(device, "cf-ad9361-lpc")) {

	}

	return -ENODEV;
}

/**
 * get_sampling_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_sampling_frequency(char *buf, size_t len,
			       const struct channel_info *channel)
{
	uint32_t sampling_freq_hz;
	ad9361_get_rx_sampling_freq (ad9361_phy, &sampling_freq_hz);
	return (ssize_t) snprintf(buf, len, "%d", (int)sampling_freq_hz);
}

/**
 * get_sampling_frequency_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_sampling_frequency_available(char *buf, size_t len,
		const struct channel_info *channel)
{
	int32_t int_dec;
	uint32_t max;
	ssize_t ret;
	if (ad9361_phy->pdata->port_ctrl.pp_conf[2] & LVDS_MODE) {
		max = 61440000U;
	} else {
		max = 61440000U / (ad9361_phy->pdata->rx2tx2 ? 2 : 1);
	}

	if (channel->ch_out) {
		if (ad9361_phy->bypass_tx_fir) {
			int_dec = 1;
		} else {
			int_dec = ad9361_phy->tx_fir_int;
		}

	} else {
		if (ad9361_phy->bypass_rx_fir) {
			int_dec = 1;
		} else {
			int_dec = ad9361_phy->rx_fir_dec;
		}
	}
	ret = (ssize_t) snprintf(buf, len, "[%"PRIu32" %d %"PRIu32"]",
				 MIN_ADC_CLK / (12 * int_dec), 1, max);

	return ret;
}

/**
 * get_filter_fir_en
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_filter_fir_en(char *buf, size_t len,
			  const struct channel_info *channel)
{
	uint8_t en_dis;
	if(channel->ch_out) {
		ad9361_get_tx_fir_en_dis (ad9361_phy, &en_dis);
	} else {
		ad9361_get_rx_fir_en_dis (ad9361_phy, &en_dis);
	}

	return (ssize_t) snprintf(buf, len, "%d", en_dis);
}

/**
 * get_bb_dc_offset_tracking_en
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_bb_dc_offset_tracking_en(char *buf, size_t len,
				     const struct channel_info *channel)
{
	if(!channel->ch_out) {
		buf[1] = 0;
		return (ssize_t) sprintf(buf, "%d", ad9361_phy->bbdc_track_en) + 1;
	}

	return -ENOENT;
}

/**
 * get_rf_dc_offset_tracking_en
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rf_dc_offset_tracking_en(char *buf, size_t len,
				     const struct channel_info *channel)
{
	if(!channel->ch_out) {
		buf[1] = 0;
		return (ssize_t) sprintf(buf, "%d", ad9361_phy->rfdc_track_en) + 1;
	}

	return -ENOENT;
}

/**
 * get_quadrature_tracking_en
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_quadrature_tracking_en(char *buf, size_t len,
				   const struct channel_info *channel)
{
	if(!channel->ch_out) {
		buf[1] = 0;
		return (ssize_t) sprintf(buf, "%d", ad9361_phy->quad_track_en) + 1;
	}

	return -ENOENT;
}

/**
 * get_rssi
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rssi(char *buf, size_t len, const struct channel_info *channel)
{
	ssize_t ret = 0;
	if(channel->ch_out) {
		uint32_t rssi_db_x_1000;
		ret = ad9361_get_tx_rssi(ad9361_phy, channel->ch_num, &rssi_db_x_1000);
		if (ret < 0) {
			return -EINVAL;
		}
		return ret < 0 ? ret : sprintf(buf, "%"PRIu32".%02"PRIu32" dB",
					       rssi_db_x_1000 / 1000, rssi_db_x_1000 % 1000);
	} else {
		struct rf_rssi rssi = {0};
		ret = ad9361_get_rx_rssi (ad9361_phy, channel->ch_num, &rssi);
		return ret < 0 ? ret : sprintf(buf, "%"PRIu32".%02"PRIu32" dB",
					       rssi.symbol / rssi.multiplier, rssi.symbol % rssi.multiplier);
	}
}

/**
 * get_rf_bandwidth
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rf_bandwidth(char *buf, size_t len,
			 const struct channel_info *channel)
{
	if(channel->ch_out) {
		return sprintf(buf, "%"PRIu32"", ad9361_phy->current_tx_bw_Hz);
	} else {
		return sprintf(buf, "%"PRIu32"", ad9361_phy->current_rx_bw_Hz);
	}
}

/**
 * get_rf_bandwidth_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rf_bandwidth_available(char *buf, size_t len,
				   const struct channel_info *channel)
{
	if(channel->ch_out) {
		return sprintf(buf, "[200000 1 40000000]");
	} else {
		return sprintf(buf, "[200000 1 56000000]");
	}
}

/**
 * get_rf_port_select_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rf_port_select_available(char *buf, size_t len,
				     const struct channel_info *channel)
{
	if(channel->ch_out) {
		return (ssize_t) sprintf(buf, "%s %s",
					 ad9361_rf_tx_port[0],
					 ad9361_rf_tx_port[1]);
	} else {
		ssize_t bytes_no = 0;
		for(int16_t i = 0; i < sizeof(ad9361_rf_rx_port) / sizeof(ad9361_rf_rx_port[0]);
		    i++) {
			if(i > 0 ) {
				bytes_no += sprintf(buf + bytes_no, " ");
			}
			bytes_no += sprintf(buf + bytes_no, "%s", ad9361_rf_rx_port[i]);
			if(bytes_no < 0) {
				break;
			}
		}
		return bytes_no;
	}
}

/**
 * get_rf_port_select
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_rf_port_select(char *buf, size_t len,
			   const struct channel_info *channel)
{
	ssize_t ret = 0;
	if(channel->ch_out) {
		uint32_t mode;
		ret = ad9361_get_tx_rf_port_output(ad9361_phy, &mode);
		return ret < 0 ? ret : sprintf(buf, "%s", ad9361_rf_tx_port[mode]);
	} else {
		uint32_t mode;
		ret = ad9361_get_rx_rf_port_input(ad9361_phy, &mode);
		return ret < 0 ? ret : sprintf(buf, "%s", ad9361_rf_rx_port[mode]);
	}
}

/**
 * get_gain_control_mode_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_gain_control_mode_available(char *buf, size_t len,
					const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%s %s %s %s",
				 ad9361_agc_modes[0],
				 ad9361_agc_modes[1],
				 ad9361_agc_modes[2],
				 ad9361_agc_modes[3]);
}

/**
 * get_gain_control_mode
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_gain_control_mode(char *buf, size_t len,
			      const struct channel_info *channel)
{
	return (ssize_t) sprintf(buf, "%s",
				 ad9361_agc_modes[ad9361_phy->agc_mode[channel->ch_num]]);
}

/**
 * get_hardwaregain_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_hardwaregain_available(char *buf, size_t len,
				   const struct channel_info *channel)
{
	if (channel->ch_out) {
		return (ssize_t) snprintf(buf, len, "[%d, %d, %d]", 0, 250, 89750);
	} else {
		return (ssize_t) snprintf(buf, len, "[%"PRIi32", %d, %"PRIi32"]",
					  ad9361_phy->rx_gain[ad9361_phy->current_table].starting_gain_db,
					  1,
					  ad9361_phy->rx_gain[ad9361_phy->current_table].max_gain_db);
	}
}

/**
 * get_hardwaregain
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_hardwaregain(char *buf, size_t len,
			 const struct channel_info *channel)
{
	if(channel->ch_out) {
		int32_t ret = ad9361_get_tx_atten(ad9361_phy, channel->ch_num + 1);
		if (ret < 0) {
			return -EINVAL;
		}
		int32_t val1 = -1 * (ret / 1000);
		int32_t val2 = (ret % 1000) * 1000;
		if (!val1) {
			val2 *= -1;

		}
		int32_t i = 0;
		if(val2 < 0 && val1 >= 0) {
			ret = (ssize_t) snprintf(buf, len, "-");
			i++;
		}
		ret = i + (ssize_t) snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32" dB", val1,
					     labs(val2));
		return ret;
	} else {
		struct rf_rx_gain rx_gain = {0};
		int32_t ret = ad9361_get_rx_gain(ad9361_phy,
						 ad9361_1rx1tx_channel_map(ad9361_phy,
								 false, channel->ch_num + 1), &rx_gain);
		if (ret < 0) {
			return -EINVAL;
		}
		return (ssize_t) snprintf(buf, len, "%d.000000 dB", (int)rx_gain.gain_db);
	}
}

static struct attrtibute_map voltage_input_read_map[] = {
	{"hardwaregain_available", get_hardwaregain_available},
	{"hardwaregain", get_hardwaregain},
	{"rssi", get_rssi},
	{"rf_port_select", get_rf_port_select},
	{"gain_control_mode", get_gain_control_mode},
	{"rf_port_select_available", get_rf_port_select_available},
	{"rf_bandwidth", get_rf_bandwidth},
	{"rf_dc_offset_tracking_en", get_rf_dc_offset_tracking_en},
	{"sampling_frequency_available", get_sampling_frequency_available},
	{"quadrature_tracking_en", get_quadrature_tracking_en},
	{"sampling_frequency", get_sampling_frequency},
	{"gain_control_mode_available", get_gain_control_mode_available},
	{"filter_fir_en", get_filter_fir_en},
	{"rf_bandwidth_available", get_rf_bandwidth_available},
	{"bb_dc_offset_tracking_en", get_bb_dc_offset_tracking_en},
};

static struct attrtibute_map voltage_output_map[] = {
	{"rf_port_select", get_rf_port_select},
	{"hardwaregain", get_hardwaregain},
	{"rssi", get_rssi},
	{"hardwaregain_available", get_hardwaregain_available},
	{"sampling_frequency_available", get_sampling_frequency_available},
	{"rf_port_select_available", get_rf_port_select_available},
	{"filter_fir_en", get_filter_fir_en},
	{"sampling_frequency", get_sampling_frequency},
	{"rf_bandwidth_available", get_rf_bandwidth_available},
	{"rf_bandwidth", get_rf_bandwidth},
};

/**
 * get_frequency_available
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_frequency_available(char *buf, size_t len,
				const struct channel_info *channel)
{
	return sprintf(buf, "[%llu 1 %llu]", AD9363A_MIN_CARRIER_FREQ_HZ,
		       AD9363A_MAX_CARRIER_FREQ_HZ);
}

/**
 * get_fastlock_save
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_fastlock_save(char *buf, size_t len,
			  const struct channel_info *channel)
{
	u8 faslock_vals[16];
	size_t length;
	int32_t ret = 0;
	int32_t i;
	ret = ad9361_fastlock_save(ad9361_phy, channel->ch_num == 1,
				   ad9361_phy->fastlock.save_profile, faslock_vals);
	length = sprintf(buf, "%u ", ad9361_phy->fastlock.save_profile);

	for (i = 0; i < RX_FAST_LOCK_CONFIG_WORD_NUM; i++)
		length += sprintf(buf + length, "%u%c", faslock_vals[i],
				  i == 15 ? '\n' : ',');
	if(ret < 0)
		return ret;

	return length;
}

/**
 * get_powerdown
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_powerdown(char *buf, size_t len,
		      const struct channel_info *channel)
{
	u64 val = 0;
	val = !!(ad9361_phy->cached_synth_pd[channel->ch_num ? 0 : 1] &
		 RX_LO_POWER_DOWN);
	return sprintf(buf, "%llu", val);
}

/**
 * get_fastlock_load
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_fastlock_load(char *buf, size_t len,
			  const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * get_fastlock_store
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_fastlock_store(char *buf, size_t len,
			   const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * get_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_frequency(char *buf, size_t len,
		      const struct channel_info *channel)
{
	u64 val = 0;
	val = ad9361_from_clk(clk_get_rate(ad9361_phy,
					   ad9361_phy->ref_clk_scale[channel->ch_num ?
									   TX_RFPLL : RX_RFPLL]));
	return sprintf(buf, "%llu", (val));
}

/**
 * get_external
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_external(char *buf, size_t len, const struct channel_info *channel)
{
	if(channel->ch_num == 0)
		return (ssize_t) sprintf(buf, "%d", ad9361_phy->pdata->use_ext_rx_lo);
	else
		return (ssize_t) sprintf(buf, "%d", ad9361_phy->pdata->use_ext_tx_lo);
}

/**
 * get_fastlock_recall
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_fastlock_recall(char *buf, size_t len,
			    const struct channel_info *channel)
{
	return sprintf(buf, "%d",
		       ad9361_phy->fastlock.current_profile[channel->ch_num]);
}

static struct attrtibute_map altvoltage_read_attrtibute_map[] = {
	{"frequency_available", get_frequency_available},
	{"fastlock_save", get_fastlock_save},
	{"powerdown", get_powerdown},
	{"fastlock_load", get_fastlock_load},
	{"fastlock_store", get_fastlock_store},
	{"frequency", get_frequency},
	{"external", get_external},
	{"fastlock_recall", get_fastlock_recall},
};

/**
 * get_dds_calibscale
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_calibscale(char *buf, size_t len,
			   const struct channel_info *channel)
{
	int32_t val, val2;
	ssize_t ret = axi_dac_dds_get_calib_scale(ad9361_phy->tx_dac, channel->ch_num,
			&val, &val2);
	int32_t i = 0;
	if(ret < 0)
		return ret;
	if(val2 < 0 && val >= 0) {
		ret = (ssize_t) snprintf(buf, len, "-");
		i++;
	}
	ret = i + (ssize_t) snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32"", val,
				     labs(val2));

	return ret;
}

/**
 * get_dds_calibphase
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_calibphase(char *buf, size_t len,
			   const struct channel_info *channel)
{
	int32_t val, val2;
	int32_t i = 0;
	ssize_t ret = axi_dac_dds_get_calib_phase(ad9361_phy->tx_dac, channel->ch_num,
			&val, &val2);
	if(ret < 0)
		return ret;
	if(val2 < 0 && val >= 0) {
		i++;
	}
	return i + snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32"", val, labs(val2));
}

/**
 * get_dds_sampling_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_sampling_frequency(char *buf, size_t len,
				   const struct channel_info *channel)
{
	return -ENODEV;
}

static struct attrtibute_map dds_voltage_read_attrtibute_map[] = {
	{"calibphase", get_dds_calibphase},
	{"calibscale", get_dds_calibscale},
	{"sampling_frequency", get_dds_sampling_frequency},
};

/**
 * get_dds_altvoltage_phase
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_altvoltage_phase(char *buf, size_t len,
				 const struct channel_info *channel)
{
	uint32_t phase;
	axi_dac_dds_get_phase(ad9361_phy->tx_dac, channel->ch_num, &phase);
	return snprintf(buf, len, "%"PRIu32"", phase);
}

/**
 * get_dds_altvoltage_scale
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_altvoltage_scale(char *buf, size_t len,
				 const struct channel_info *channel)
{
	int32_t scale;
	axi_dac_dds_get_scale(ad9361_phy->tx_dac, channel->ch_num, &scale);

	return snprintf(buf, len, "%"PRIi32".%.6"PRIi32"", (scale / 1000000),
			(scale % 1000000));
}

/**
 * get_dds_altvoltage_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_altvoltage_frequency(char *buf, size_t len,
				     const struct channel_info *channel)
{
	uint32_t freq;
	axi_dac_dds_get_frequency(ad9361_phy->tx_dac, channel->ch_num, &freq);

	return snprintf(buf, len, "%"PRIi32"", freq);
}

extern int32_t ad9361_auxdac_get(struct ad9361_rf_phy *phy, int32_t dac);
/**
 * get_dds_altvoltage_raw
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_altvoltage_raw(char *buf, size_t len,
			       const struct channel_info *channel)
{
	return snprintf(buf, len, "%"PRIi32"", ad9361_auxdac_get(ad9361_phy,
			channel->ch_num));
}

/**
 * get_dds_altvoltage_sampling_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_dds_altvoltage_sampling_frequency(char *buf, size_t len,
		const struct channel_info *channel)
{
	return -ENODEV;
}

static struct attrtibute_map dds_altvoltage_read_attrtibute_map[] = {
	{"phase", get_dds_altvoltage_phase},
	{"scale", get_dds_altvoltage_scale},
	{"frequency", get_dds_altvoltage_frequency},
	{"raw", get_dds_altvoltage_raw},
	{"sampling_frequency", get_dds_altvoltage_sampling_frequency},
};

/**
 * get_cf_calibphase
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_cf_calibphase(char *buf, size_t len,
			  const struct channel_info *channel)
{
	int32_t val, val2;
	int32_t i = 0;
	ssize_t ret = axi_adc_get_calib_phase(ad9361_phy->rx_adc, channel->ch_num, &val,
					      &val2);
	if(ret < 0)
		return ret;
	if(val2 < 0 && val >= 0) {
		snprintf(buf, len, "-");
		i++;
	}

	return i + snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32"", val, labs(val2));
}

/**
 * get_cf_calibbias
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_cf_calibbias(char *buf, size_t len,
			 const struct channel_info *channel)
{
	int32_t val;
	axi_adc_get_calib_bias(ad9361_phy->rx_adc,
			       channel->ch_num,
			       &val,
			       NULL);

	return snprintf(buf, len, "%"PRIi32"", val);
}

/**
 * get_cf_calibscale
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_cf_calibscale(char *buf, size_t len,
			  const struct channel_info *channel)
{
	int32_t val, val2;
	ssize_t ret = axi_adc_get_calib_scale(ad9361_phy->rx_adc, channel->ch_num, &val,
					      &val2);
	int32_t i = 0;
	if(ret < 0)
		return ret;
	if(val2 < 0 && val >= 0) {
		ret = (ssize_t) snprintf(buf, len, "-");
		i++;
	}
	ret = i + (ssize_t) snprintf(&buf[i], len, "%"PRIi32".%.6"PRIi32"", val,
				     labs(val2));

	return ret;
}

/**
 * get_cf_samples_pps
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_cf_samples_pps(char *buf, size_t len,
			   const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * get_cf_sampling_frequency
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @param *channel channel properties
 * @return length of chars written in buf, or negative value on failure
 */
ssize_t get_cf_sampling_frequency(char *buf, size_t len,
				  const struct channel_info *channel)
{
	uint32_t sampling_freq_hz;
	ad9361_get_rx_sampling_freq (ad9361_phy, &sampling_freq_hz);

	return (ssize_t) snprintf(buf, len, "%d", (int)sampling_freq_hz);
}

static struct attrtibute_map cf_voltage_read_attrtibute_map[] = {
	{"calibphase", get_cf_calibphase},
	{"calibbias", get_cf_calibbias},
	{"calibscale", get_cf_calibscale},
	{"samples_pps", get_cf_samples_pps},
	{"sampling_frequency", get_cf_sampling_frequency},
};

/**
 * read channel attribute
 * @param *device name
 * @param *channel name
 * @param *ch_out type: input/output
 * @param *attr name
 * @param *buff where value is stored
 * @param len maximum length of value to be stored in buf
 * @return length of chars written in buf
 */
static ssize_t ch_read_attr(const char *device, const char *channel,
			    bool ch_out, const char *attr, char *buf, size_t len)
{
	int32_t temp;
	int16_t attribute_id;
	if (!dev_is_ad9361_module(device))
		return -ENODEV;
	if(strequal(device, "ad9361-phy")) { // global attributes
		if(channel == strstr(channel, "voltage")) {
			const struct channel_info channel_info = {
				get_channel((char*)channel, "voltage"),
				ch_out
			};
			attribute_id = get_attribute_id(attr, voltage_input_read_map,
							ARRAY_SIZE(voltage_input_read_map));
			if(attribute_id >= 0) {
				return voltage_input_read_map[attribute_id].exec(buf, len, &channel_info);
			}
			if(strequal(attr, "")) {
				if(ch_out)
					return read_all_attr(buf, len, &channel_info, voltage_output_map,
							     ARRAY_SIZE(voltage_output_map));

				else
					return read_all_attr(buf, len, &channel_info, voltage_input_read_map,
							     ARRAY_SIZE(voltage_input_read_map));
			}
		} else if(NULL != strstr(channel, "altvoltage")) {
			const struct channel_info channel_info = {
				get_channel((char*)channel, "altvoltage"),
				ch_out
			};
			attribute_id = get_attribute_id(attr, altvoltage_read_attrtibute_map,
							ARRAY_SIZE(altvoltage_read_attrtibute_map));
			if(attribute_id >= 0) {
				return altvoltage_read_attrtibute_map[attribute_id].exec(buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return read_all_attr(buf, len, &channel_info, altvoltage_read_attrtibute_map,
						     ARRAY_SIZE(altvoltage_read_attrtibute_map));
			}
		}

		else if(strequal(channel, "temp0")) {
			if(strequal(attr, "input")) {
				ad9361_get_temperature(ad9361_phy, &temp);
				return (ssize_t) snprintf(buf, len, "%d", (int)temp);
			}
		} else if(strequal(channel, "out")) {
			if(strequal(attr, "voltage_filter_fir_en")) {
				uint8_t en_dis_tx, en_dis_rx;
				ad9361_get_tx_fir_en_dis (ad9361_phy, &en_dis_tx);
				ad9361_get_rx_fir_en_dis (ad9361_phy, &en_dis_rx);
				return (ssize_t) snprintf(buf, len, "%d", en_dis_rx && en_dis_tx);
			}
		}
	} else if(strequal(device, "cf-ad9361-dds-core-lpc")) {
		if(channel == strstr(channel, "voltage")) {
			attribute_id = get_attribute_id(attr, dds_voltage_read_attrtibute_map,
							ARRAY_SIZE(dds_voltage_read_attrtibute_map));
			const struct channel_info channel_info = {
				get_channel((char*)channel, "voltage"),
				ch_out
			};
			if(attribute_id >= 0) {
				return dds_voltage_read_attrtibute_map[attribute_id].exec(buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return read_all_attr(buf, len, &channel_info, dds_voltage_read_attrtibute_map,
						     ARRAY_SIZE(dds_voltage_read_attrtibute_map));
			}
		} else if(NULL != strstr(channel, "altvoltage")) {
			attribute_id = get_attribute_id(attr, dds_altvoltage_read_attrtibute_map,
							ARRAY_SIZE(dds_altvoltage_read_attrtibute_map));
			const struct channel_info channel_info = {
				get_channel((char*)channel, "altvoltage"),
				ch_out
			};
			if(attribute_id >= 0) {
				return dds_altvoltage_read_attrtibute_map[attribute_id].exec(buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return read_all_attr(buf, len, &channel_info,
						     dds_altvoltage_read_attrtibute_map,
						     ARRAY_SIZE(dds_altvoltage_read_attrtibute_map));
			}
		}
		return -ENOENT;
	} else if(strequal(device, "cf-ad9361-lpc")) {
		if(channel == strstr(channel, "voltage")) {
			attribute_id = get_attribute_id(attr, cf_voltage_read_attrtibute_map,
							ARRAY_SIZE(cf_voltage_read_attrtibute_map));
			const struct channel_info channel_info = {
				get_channel((char*)channel, "voltage"),
				ch_out
			};
			if(attribute_id >= 0) {
				return cf_voltage_read_attrtibute_map[attribute_id].exec((char*)buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return read_all_attr((char*)buf, len, &channel_info,
						     cf_voltage_read_attrtibute_map, ARRAY_SIZE(cf_voltage_read_attrtibute_map));
			}
		}
		return -ENOENT;
	}

	return -ENOENT;
}

/**
 * set_hardwaregain_available
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_hardwaregain_available(char *buf, size_t len,
				   const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_hardwaregain
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_hardwaregain(char *buf, size_t len,
			 const struct channel_info *channel)
{
	ssize_t ret = 0;
	float gain = strtof(buf, NULL);
	int32_t val1 = (int32_t)gain;
	int32_t val2 = (int32_t)(gain * 1000) % 1000;
	if (channel->ch_out) {
		int32_t ch;
		if (val1 > 0 || (val1 == 0 && val2 > 0)) {
			return -EINVAL;
		}
		uint32_t code = ((abs(val1) * 1000) + (abs(val2)/* / 1000*/));
		ch = ad9361_1rx1tx_channel_map(ad9361_phy, true, channel->ch_num);
		ret = ad9361_set_tx_atten(ad9361_phy, code, ch == 0, ch == 1,
					  !ad9361_phy->pdata->update_tx_gain_via_alert);
		if (ret < 0) {
			return -EINVAL;
		}
	} else {
		struct rf_rx_gain rx_gain = {0};
		rx_gain.gain_db = val1;
		ret = ad9361_set_rx_gain(ad9361_phy,
					 ad9361_1rx1tx_channel_map(ad9361_phy, false, channel->ch_num + 1), &rx_gain);
		if (ret < 0) {
			return -EINVAL;
		}
	}

	return len;
}

/**
 * set_rssi
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_rssi(char *buf, size_t len, const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_rf_port_select
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_rf_port_select(char *buf, size_t len,
			   const struct channel_info *channel)
{
	ssize_t ret = 0;
	uint32_t i = 0;
	if(channel->ch_out) {
		for(i = 0; i < sizeof(ad9361_rf_tx_port) / sizeof(ad9361_rf_tx_port[0]); i++) {
			if(strequal(ad9361_rf_tx_port[i], buf)) {
				break;
			}
		}
		if(i >= sizeof(ad9361_rf_tx_port) / sizeof(ad9361_rf_tx_port[0])) {
			return -EINVAL;
		}
		ret = ad9361_set_tx_rf_port_output(ad9361_phy, i);
		return ret < 0 ? ret : len;
	} else {
		for(i = 0; i < sizeof(ad9361_rf_rx_port) / sizeof(ad9361_rf_rx_port[0]); i++) {
			if(strequal(ad9361_rf_rx_port[i], buf)) {
				break;
			}
		}
		if(i >= sizeof(ad9361_rf_tx_port) / sizeof(ad9361_rf_tx_port[0])) {
			return -EINVAL;
		}
		ret = ad9361_set_rx_rf_port_input(ad9361_phy, i);
		return ret < 0 ? ret : len;
	}
}

/**
 * set_gain_control_mode
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_gain_control_mode(char *buf, size_t len,
			      const struct channel_info *channel)
{
	struct rf_gain_ctrl gc = {0};
	int32_t i;
	for(i = 0; i < sizeof(ad9361_agc_modes) / sizeof(ad9361_agc_modes[0]); i++) {
		if(strequal(ad9361_agc_modes[i], buf)) {
			break;
		}
	}
	if(i >= sizeof(ad9361_agc_modes) / sizeof(ad9361_agc_modes[0])) {
		return -EINVAL;
	}
	uint32_t mode = i;
	if (ad9361_phy->agc_mode[channel->ch_num] == mode)
		return len;
	gc.ant = ad9361_1rx1tx_channel_map(ad9361_phy, false, channel->ch_num + 1);
	gc.mode = ad9361_phy->agc_mode[channel->ch_num] = mode;
	ad9361_set_gain_ctrl_mode(ad9361_phy, &gc);

	return len;
}

/**
 * set_rf_port_select_available
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_rf_port_select_available(char *buf, size_t len,
				     const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_rf_bandwidth
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_rf_bandwidth(char *buf, size_t len,
			 const struct channel_info *channel)
{
	ssize_t ret = 0;
	uint32_t rf_bandwidth = read_ul_value(buf);
	rf_bandwidth = ad9361_validate_rf_bw(ad9361_phy, rf_bandwidth);
	if(channel->ch_out) {
		if(ad9361_phy->current_tx_bw_Hz != rf_bandwidth) {
			ret = ad9361_update_rf_bandwidth(ad9361_phy, ad9361_phy->current_rx_bw_Hz,
							 rf_bandwidth);
		}
	} else {
		if(ad9361_phy->current_rx_bw_Hz != rf_bandwidth) {
			ret = ad9361_update_rf_bandwidth(ad9361_phy, rf_bandwidth,
							 ad9361_phy->current_tx_bw_Hz);
		}
	}
	if(ret < 0) {
		return ret;
	}

	return len;
}

/**
 * set_rf_dc_offset_tracking_en
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_rf_dc_offset_tracking_en(char *buf, size_t len,
				     const struct channel_info *channel)
{
	int8_t en_dis = read_value(buf);
	if(en_dis < 0) {
		return en_dis;
	}
	ad9361_phy->rfdc_track_en = en_dis ? 1 : 0;
	if(!channel->ch_out) {
		return ad9361_tracking_control(ad9361_phy, ad9361_phy->bbdc_track_en,
					       ad9361_phy->rfdc_track_en, ad9361_phy->quad_track_en);
	}

	return -ENOENT;
}

/**
 * set_sampling_frequency_available
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_sampling_frequency_available(char *buf, size_t len,
		const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_quadrature_tracking_en
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_quadrature_tracking_en(char *buf, size_t len,
				   const struct channel_info *channel)
{
	int8_t en_dis = read_value(buf);
	if(en_dis < 0) {
		return en_dis;
	}
	ad9361_phy->quad_track_en = en_dis ? 1 : 0;
	if(!channel->ch_out) {
		return ad9361_tracking_control(ad9361_phy, ad9361_phy->bbdc_track_en,
					       ad9361_phy->rfdc_track_en, ad9361_phy->quad_track_en);
	}

	return -ENOENT;
}

/**
 * set_sampling_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_sampling_frequency(char *buf, size_t len,
			       const struct channel_info *channel)
{
	uint32_t sampling_freq_hz = read_ul_value(buf);
	ad9361_set_rx_sampling_freq (ad9361_phy, sampling_freq_hz);

	return len;
}

/**
 * set_gain_control_mode_available
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_gain_control_mode_available(char *buf, size_t len,
					const struct channel_info *channel)
{
	struct rf_gain_ctrl gc = {0};
	int32_t i;
	for(i = 0; i < sizeof(ad9361_agc_modes) / sizeof(ad9361_agc_modes[0]); i++) {
		if(strequal(ad9361_agc_modes[i], buf)) {
			break;
		}
	}
	if(i >= sizeof(ad9361_agc_modes) / sizeof(ad9361_agc_modes[0])) {
		return -EINVAL;
	}
	uint32_t mode = i;
	if (ad9361_phy->agc_mode[channel->ch_num] == mode)
		return len;
	gc.ant = ad9361_1rx1tx_channel_map(ad9361_phy, false, channel->ch_num + 1);
	gc.mode = ad9361_phy->agc_mode[channel->ch_num] = mode;
	ad9361_set_gain_ctrl_mode(ad9361_phy, &gc);

	return len;
}

/**
 * set_filter_fir_en
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_filter_fir_en(char *buf, size_t len,
			  const struct channel_info *channel)
{
	int8_t en_dis = read_value(buf);
	if(en_dis < 0) {
		return en_dis;
	}
	en_dis = en_dis ? 1 : 0;
	if(channel->ch_out) {
		ad9361_set_tx_fir_en_dis (ad9361_phy, en_dis);
	} else {
		ad9361_set_rx_fir_en_dis (ad9361_phy, en_dis);
	}

	return len;
}

/**
 * set_rf_bandwidth_available
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_rf_bandwidth_available(char *buf, size_t len,
				   const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_bb_dc_offset_tracking_en
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_bb_dc_offset_tracking_en(char *buf, size_t len,
				     const struct channel_info *channel)
{
	int8_t en_dis = read_value(buf);
	if(en_dis < 0) {
		return en_dis;
	}
	ad9361_phy->bbdc_track_en = en_dis ? 1 : 0;
	if(!channel->ch_out) {
		return ad9361_tracking_control(ad9361_phy, ad9361_phy->bbdc_track_en,
					       ad9361_phy->rfdc_track_en, ad9361_phy->quad_track_en);
	}

	return -ENOENT;
}

static struct attrtibute_map ch_in_write_attrtibute_map[] = {
	{"hardwaregain_available", set_hardwaregain_available},
	{"hardwaregain", set_hardwaregain},
	{"rssi", set_rssi},
	{"rf_port_select", set_rf_port_select},
	{"gain_control_mode", set_gain_control_mode},
	{"rf_port_select_available", set_rf_port_select_available},
	{"rf_bandwidth", set_rf_bandwidth},
	{"rf_dc_offset_tracking_en", set_rf_dc_offset_tracking_en},
	{"sampling_frequency_available", set_sampling_frequency_available},
	{"quadrature_tracking_en", set_quadrature_tracking_en},
	{"sampling_frequency", set_sampling_frequency},
	{"gain_control_mode_available", set_gain_control_mode_available},
	{"filter_fir_en", set_filter_fir_en},
	{"rf_bandwidth_available", set_rf_bandwidth_available},
	{"bb_dc_offset_tracking_en", set_bb_dc_offset_tracking_en},
};

static struct attrtibute_map ch_out_write_attrtibute_map[] = {
	{"rf_port_select", set_rf_port_select},
	{"hardwaregain", set_hardwaregain},
	{"rssi", set_rssi},
	{"hardwaregain_available", set_hardwaregain_available},
	{"sampling_frequency_available", set_sampling_frequency_available},
	{"rf_port_select_available", set_rf_port_select_available},
	{"filter_fir_en", set_filter_fir_en},
	{"sampling_frequency", set_sampling_frequency},
	{"rf_bandwidth_available", set_rf_bandwidth_available},
	{"rf_bandwidth", set_rf_bandwidth},
};

/**
 * set_frequency_available
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_frequency_available(char *buf, size_t len,
				const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_fastlock_save
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_fastlock_save(char *buf, size_t len,
			  const struct channel_info *channel)
{
	u32 readin = read_ul_value(buf);
	ad9361_phy->fastlock.save_profile = readin;

	return len;
}

/**
 * set_powerdown
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_powerdown(char *buf, size_t len,
		      const struct channel_info *channel)
{
	ssize_t ret = 0;
	bool res = read_value(buf) ? 1 : 0;
	switch (channel->ch_num) {
	case 0:
		ret = ad9361_synth_lo_powerdown(ad9361_phy, res ? LO_OFF : LO_ON, LO_DONTCARE);
		break;
	case 1:
		ret = ad9361_synth_lo_powerdown(ad9361_phy, LO_DONTCARE, res ? LO_OFF : LO_ON);
		break;
	}
	if(ret < 0)
		return ret;

	return len;
}

/**
 * set_fastlock_load
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_fastlock_load(char *buf, size_t len,
			  const struct channel_info *channel)
{
	ssize_t ret = 0;
	char *line, *ptr = (char*) buf;
	u8 faslock_vals[16];
	uint32_t profile = 0, val, val2, i = 0;

	while ((line = strsep(&ptr, ","))) {
		if (line >= buf + len)
			break;

		ret = sscanf(line, "%"PRIu32" %"PRIu32"", &val, &val2);
		if (ret == 1) {
			faslock_vals[i++] = val;
			continue;
		} else if (ret == 2) {
			profile = val;
			faslock_vals[i++] = val2;
			continue;
		}
	}
	if (i == 16)
		ret = ad9361_fastlock_load(ad9361_phy, channel->ch_num == 1,
					   profile, faslock_vals);
	else
		ret = -EINVAL;
	if(ret < 0)
		return ret;

	return len;
}

/**
 * set_fastlock_store
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_fastlock_store(char *buf, size_t len,
			   const struct channel_info *channel)
{
	uint32_t profile = read_ul_value(buf);

	return ad9361_fastlock_store(ad9361_phy, channel->ch_num == 1, profile);
}

/**
 * set_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_frequency(char *buf, size_t len,
		      const struct channel_info *channel)
{
	uint64_t lo_freq_hz = read_ul_value(buf);
	ssize_t ret = 0;
	switch (channel->ch_num) {
	case 0:
		ret = clk_set_rate(ad9361_phy, ad9361_phy->ref_clk_scale[RX_RFPLL],
				   ad9361_to_clk(lo_freq_hz));
		break;
	case 1:
		ret = clk_set_rate(ad9361_phy, ad9361_phy->ref_clk_scale[TX_RFPLL],
				   ad9361_to_clk(lo_freq_hz));
		break;
	default:
		ret = -EINVAL;
	}
	if(ret < 0)
		return ret;

	return len;
}

/**
 * set_external
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_external(char *buf, size_t len, const struct channel_info *channel)
{
	bool select = read_value(buf) ? 1 : 0;
	ssize_t ret = 0;
	if(channel->ch_num == 0)
		ret = ad9361_set_rx_lo_int_ext(ad9361_phy, select);
	else
		ret = ad9361_set_tx_lo_int_ext(ad9361_phy, select);
	if(ret < 0)
		return ret;

	return len;
}

/**
 * set_fastlock_recall
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_fastlock_recall(char *buf, size_t len,
			    const struct channel_info *channel)
{
	ssize_t ret = 0;
	uint32_t profile = read_ul_value(buf);
	ret = ad9361_fastlock_recall(ad9361_phy, channel->ch_num == 1, profile);
	if(ret < 0)
		return ret;
	return len;
}

static struct attrtibute_map altvoltage_write_attrtibute_map[] = {
	{"frequency_available", set_frequency_available},
	{"fastlock_save", set_fastlock_save},
	{"powerdown", set_powerdown},
	{"fastlock_load", set_fastlock_load},
	{"fastlock_store", set_fastlock_store},
	{"frequency", set_frequency},
	{"external", set_external},
	{"fastlock_recall", set_fastlock_recall},
};

/**
 * set_dds_calibscale
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_calibscale(char *buf, size_t len,
			   const struct channel_info *channel)
{
	float calib= strtof(buf, NULL);
	int32_t val = (int32_t)calib;
	int32_t val2 = (int32_t)(calib* 1000000) % 1000000;
	axi_dac_dds_set_calib_scale(ad9361_phy->tx_dac, channel->ch_num, val, val2);

	return len;
}

/**
 * set_dds_calibphase
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_calibphase(char *buf, size_t len,
			   const struct channel_info *channel)
{
	float calib = strtof(buf, NULL);
	int32_t val = (int32_t)calib;
	int32_t val2 = (int32_t)(calib* 1000000) % 1000000;
	axi_dac_dds_set_calib_phase(ad9361_phy->tx_dac, channel->ch_num, val, val2);

	return len;
}

/**
 * set_dds_sampling_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_sampling_frequency(char *buf, size_t len,
				   const struct channel_info *channel)
{
	return -ENODEV;
}

static struct attrtibute_map dds_voltage_write_attrtibute_map[] = {
	{"calibphase", set_dds_calibphase},
	{"calibscale", set_dds_calibscale},
	{"sampling_frequency", set_dds_sampling_frequency},
};

/**
 * set_dds_altvoltage_phase
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_altvoltage_phase(char *buf, size_t len,
				 const struct channel_info *channel)
{
	uint32_t phase = read_ul_value(buf);
	axi_dac_dds_set_phase(ad9361_phy->tx_dac, channel->ch_num, phase);

	return len;
}

/**
 * set_dds_altvoltage_scale
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_altvoltage_scale(char *buf, size_t len,
				 const struct channel_info *channel)
{
	float fscale = strtof(buf, NULL);
	int32_t scale = fscale * 1000000;
	axi_dac_dds_set_scale(ad9361_phy->tx_dac, channel->ch_num, scale);

	return len;
}

/**
 * set_dds_altvoltage_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_altvoltage_frequency(char *buf, size_t len,
				     const struct channel_info *channel)
{
	uint32_t freq = read_ul_value(buf);
	axi_dac_dds_set_frequency(ad9361_phy->tx_dac, channel->ch_num, freq);

	return len;
}

extern int32_t ad9361_auxdac_set(struct ad9361_rf_phy *phy, int32_t dac,
				 int32_t val_mV);
/**
 * set_dds_altvoltage_raw
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_altvoltage_raw(char *buf, size_t len,
			       const struct channel_info *channel)
{
	uint32_t dds_mode = read_ul_value(buf);
	if(dds_mode) { 		//DDS mode selected
		axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DDS);
	} else {				//DMA mode selected
		axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DMA);
	}

	return len;
}

/**
 * set_dds_altvoltage_sampling_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_dds_altvoltage_sampling_frequency(char *buf, size_t len,
		const struct channel_info *channel)
{
	return -ENODEV;
}

static struct attrtibute_map dds_altvoltage_write_attrtibute_map[] = {
	{"phase", set_dds_altvoltage_phase},
	{"scale", set_dds_altvoltage_scale},
	{"frequency", set_dds_altvoltage_frequency},
	{"raw", set_dds_altvoltage_raw},
	{"sampling_frequency", set_dds_altvoltage_sampling_frequency},
};

/**
 * set_cf_calibphase
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_cf_calibphase(char *buf, size_t len,
			  const struct channel_info *channel)
{
	float calib = strtof(buf, NULL);
	int32_t val = (int32_t)calib;
	int32_t val2 = (int32_t)(calib* 1000000) % 1000000;
	axi_adc_set_calib_phase(ad9361_phy->rx_adc, channel->ch_num, val, val2);

	return len;
}

/**
 * set_cf_calibbias
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_cf_calibbias(char *buf, size_t len,
			 const struct channel_info *channel)
{
	int32_t val = read_value(buf);
	axi_adc_set_calib_bias(ad9361_phy->rx_adc,
			       channel->ch_num,
			       val,
			       0);

	return len;
}

/**
 * set_cf_calibscale
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_cf_calibscale(char *buf, size_t len,
			  const struct channel_info *channel)
{
	float calib= strtof(buf, NULL);
	int32_t val = (int32_t)calib;
	int32_t val2 = (int32_t)(calib* 1000000) % 1000000;
	axi_adc_set_calib_scale(ad9361_phy->rx_adc, channel->ch_num, val, val2);

	return len;
}

/**
 * set_cf_samples_pps
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_cf_samples_pps(char *buf, size_t len,
			   const struct channel_info *channel)
{
	return -ENODEV;
}

/**
 * set_cf_sampling_frequency
 * @param *buff value to be written to attribute
 * @param len of the value
 * @param *channel channel properties
 * @return length of chars written to attribute, or negative value on failure
 */
ssize_t set_cf_sampling_frequency(char *buf, size_t len,
				  const struct channel_info *channel)
{
	return -ENODEV;
}

static struct attrtibute_map cf_voltage_write_attrtibute_map[] = {
	{"calibphase", set_cf_calibphase},
	{"calibbias", set_cf_calibbias},
	{"calibscale", set_cf_calibscale},
	{"samples_pps", set_cf_samples_pps},
	{"sampling_frequency", set_cf_sampling_frequency},
};

/**
 * write channel attribute
 * @param *device name
 * @param *channel name
 * @param *ch_out type: input/output
 * @param *attr name
 * @param *buff where value is stored
 * @param len length of the value
 * @return length of chars written to attribute, negative value in case of failure
 */
static ssize_t ch_write_attr(const char *device, const char *channel,
			     bool ch_out, const char *attr, const char *buf, size_t len)
{
	int16_t attribute_id;
	if (!dev_is_ad9361_module(device))
		return -ENODEV;
	if(strequal(device, "ad9361-phy")) {
		if(channel == strstr(channel, "voltage")) {
			const struct channel_info channel_info = {
				get_channel((char*)channel, "voltage"),
				ch_out
			};
			attribute_id = get_attribute_id(attr, ch_in_write_attrtibute_map,
							ARRAY_SIZE(ch_in_write_attrtibute_map));
			if(attribute_id >= 0) {
				return ch_in_write_attrtibute_map[attribute_id].exec((char*)buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {

				if(ch_out)
					return write_all_attr((char*)buf, len, &channel_info,
							      ch_out_write_attrtibute_map, ARRAY_SIZE(ch_out_write_attrtibute_map));
				else
					return write_all_attr((char*)buf, len, &channel_info,
							      ch_in_write_attrtibute_map, ARRAY_SIZE(ch_in_write_attrtibute_map));
			}
			return -ENOENT;
		} else if(NULL != strstr(channel, "altvoltage")) {
			const struct channel_info channel_info = {
				get_channel((char*)channel, "altvoltage"),
				ch_out
			};
			int16_t attribute_id = get_attribute_id(attr, altvoltage_write_attrtibute_map,
								ARRAY_SIZE(altvoltage_write_attrtibute_map));
			if(attribute_id >= 0) {
				return altvoltage_write_attrtibute_map[attribute_id].exec((char*)buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return write_all_attr((char*)buf, len, &channel_info,
						      altvoltage_write_attrtibute_map, ARRAY_SIZE(altvoltage_write_attrtibute_map));
			}
			return -ENOENT;
		} else if(strequal(channel, "out")) {
			if(strequal(attr, "voltage_filter_fir_en")) {
				int8_t en_dis = read_value(buf) ? 1 : 0;
				ad9361_set_tx_fir_en_dis (ad9361_phy, en_dis);
				ad9361_set_rx_fir_en_dis (ad9361_phy, en_dis);
				return len;
			}
		}
	} else if(strequal(device, "cf-ad9361-dds-core-lpc")) {
		if(channel == strstr(channel, "voltage")) {
			attribute_id = get_attribute_id(attr, dds_voltage_write_attrtibute_map,
							ARRAY_SIZE(dds_voltage_write_attrtibute_map));
			const struct channel_info channel_info = {
				get_channel((char*)channel, "voltage"),
				ch_out
			};
			if(attribute_id >= 0) {
				return dds_voltage_write_attrtibute_map[attribute_id].exec((char*)buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return write_all_attr((char*)buf, len, &channel_info,
						      dds_voltage_write_attrtibute_map, ARRAY_SIZE(dds_voltage_write_attrtibute_map));
			}
		} else if(NULL != strstr(channel, "altvoltage")) {
			attribute_id = get_attribute_id(attr, dds_altvoltage_write_attrtibute_map,
							ARRAY_SIZE(dds_altvoltage_write_attrtibute_map));
			const struct channel_info channel_info = {
				get_channel((char*)channel, "altvoltage"),
				ch_out
			};
			if(attribute_id >= 0) {
				return dds_altvoltage_write_attrtibute_map[attribute_id].exec((char*)buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return write_all_attr((char*)buf, len, &channel_info,
						      dds_altvoltage_write_attrtibute_map,
						      ARRAY_SIZE(dds_altvoltage_write_attrtibute_map));
			}
		}
		return -ENOENT;
	} else if(strequal(device, "cf-ad9361-lpc")) {
		if(channel == strstr(channel, "voltage")) {
			attribute_id = get_attribute_id(attr, cf_voltage_write_attrtibute_map,
							ARRAY_SIZE(cf_voltage_write_attrtibute_map));
			const struct channel_info channel_info = {
				get_channel((char*)channel, "voltage"),
				ch_out
			};
			if(attribute_id >= 0) {
				return cf_voltage_write_attrtibute_map[attribute_id].exec((char*)buf, len,
						&channel_info);
			}
			if(strequal(attr, "")) {
				return write_all_attr((char*)buf, len, &channel_info,
						      cf_voltage_write_attrtibute_map, ARRAY_SIZE(cf_voltage_write_attrtibute_map));
			}
		}

		return -ENOENT;
	}

	return -ENOENT;
}

/**
 * open device
 * @param *device name
 * @param sample_size
 * @param mask
 * @return 0 on success, otherwise negative value
 */
static int32_t open_dev(const char *device, size_t sample_size, uint32_t mask)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;

	if (mask & ~input_channel_mask)
		return -ENOENT;

	request_mask = mask;

	return 0;
}

/**
 * close device
 * @param *device name
 * @return 0 on success, otherwise negative value
 */
static int32_t close_dev(const char *device)
{
	return dev_is_ad9361_module(device) ? 0 : -ENODEV;
}

/**
 * get device mask, this specifies the channels that are used
 * @param *device name
 * @param *device mask
 * @return 0 on success, otherwise negative value
 */
static int32_t get_mask(const char *device, uint32_t *mask)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;
	*mask = input_channel_mask; // this way client has to do demux of data

	return 0;
}

/**
 * write data to DAC
 * @param *device name
 * @param *buff
 * @param bytes_count
 * @return bytes_count
 */
static ssize_t write_dev(const char *device, const char *buf,
			 size_t bytes_count)
{
	ad9361_phy->tx_dmac->flags = DMA_CYCLIC;
	axi_dac_set_buff(ad9361_phy->tx_dac, DAC_DDR_BASEADDR, (uint16_t *)buf,
			 bytes_count);
	axi_dmac_transfer(ad9361_phy->tx_dmac, DAC_DDR_BASEADDR, bytes_count);
	axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DMA);

	return bytes_count;
}

/**
 * read data from DAC
 * @param *device name
 * @param **buff where data's are stored
 * @param bytes_count
 * @return bytes_count
 */
static ssize_t read_dev(const char *device, char **pbuf, size_t bytes_count)
{
	if (!dev_is_ad9361_module(device))
		return -ENODEV;
	ad9361_phy->rx_dmac->flags = 0;
	axi_dmac_transfer(ad9361_phy->rx_dmac,
			  ADC_DDR_BASEADDR, bytes_count);
	Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR,	bytes_count);
	*pbuf = (char *)ADC_DDR_BASEADDR;

	return bytes_count;
}

const struct tinyiiod_ops ops = {
	//communication
#ifdef UART_INTERFACE
	.read = serial_read,
	.read_line = serial_read_line,
	.read_nonbloking = serial_read_nonblocking,
	.read_wait = serial_read_wait,
	.write = serial_write_data,
#endif // UART_INTERFACE

#ifdef TCPIP_INTERFACE
	.read = network_read,
	.read_line = network_read_line,
	.read_nonbloking = network_read_nonblocking,
	.read_wait = network_read_wait,
	.write = network_write_data,
	.exit = network_exit,
#endif // TCPIP_INTERFACE
	//device operations
	.read_attr = read_attr,
	.write_attr = write_attr,
	.ch_read_attr = ch_read_attr,
	.ch_write_attr = ch_write_attr,
	.read_device = read_dev,
	.write_device = write_dev,
	.read_reg = read_reg,
	.write_reg = write_reg,

	//
	.open = open_dev,
	.close = close_dev,
	.get_mask = get_mask,
};

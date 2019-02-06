/***************************************************************************//**
 *   @file   axi_dac_core.h
 *   @brief  Driver for the Analog Devices AXI-DAC-CORE module.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef AXI_DAC_CORE_H_
#define AXI_DAC_CORE_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
enum dds_data_select {
	DATA_SEL_DDS,
	DATA_SEL_SED,
	DATA_SEL_DMA,
	DATA_SEL_ZERO,	/* OUTPUT 0 */
	DATA_SEL_PN7,
	DATA_SEL_PN15,
	DATA_SEL_PN23,
	DATA_SEL_PN31,
	DATA_SEL_LB,	/* loopback data (ADC) */
	DATA_SEL_PNXX,	/* (Device specific) */
};

struct axi_dac {
	const char *name;
	uint32_t base;
	uint8_t	num_channels;
	uint64_t clock_hz;
};

struct axi_dac_init {
	const char *name;
	uint32_t base;
	uint8_t	num_channels;
};

#define DAC_REG_CHAN_CNTRL_6(c)		(0x0414 + (c) * 0x40)
#define DAC_IQCOR_ENB				(1 << 2) /* v8.0 */

#define DAC_REG_CHAN_CNTRL_7(c)		(0x0418 + (c) * 0x40) /* v8.0 */
#define DAC_DAC_DDS_SEL(x)			(((x) & 0xF) << 0)
#define DAC_TO_DAC_DDS_SEL(x)		(((x) >> 0) & 0xF)

#define DAC_REG_CHAN_CNTRL_8(c)		(0x041C + (c) * 0x40) /* v8.0 */
#define DAC_IQCOR_COEFF_1(x)		(((x) & 0xFFFF) << 16)
#define DAC_TO_IQCOR_COEFF_1(x)		(((x) >> 16) & 0xFFFF)
#define DAC_IQCOR_COEFF_2(x)		(((x) & 0xFFFF) << 0)
#define DAC_TO_IQCOR_COEFF_2(x)		(((x) >> 0) & 0xFFFF)

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
int32_t axi_dac_init(struct axi_dac **dac_core,
		     const struct axi_dac_init *init);
int32_t axi_dac_remove(struct axi_dac *dac);
int32_t axi_dac_write(struct axi_dac *dac,
		      uint32_t reg_addr,
		      uint32_t reg_data);
int32_t axi_dac_read(struct axi_dac *dac,
		     uint32_t reg_addr,
		     uint32_t *reg_data);
int32_t axi_dac_dds_set_frequency(struct axi_dac *dac,
				  uint32_t chan, uint32_t freq_hz);
void axi_dac_dds_get_frequency(struct axi_dac *dac, uint32_t chan,
			       uint32_t *freq);
int32_t axi_dac_dds_set_phase(struct axi_dac *dac, uint32_t chan,
			      uint32_t phase);
void axi_dac_dds_get_phase(struct axi_dac *dac, uint32_t chan, uint32_t *phase);
int32_t axi_dac_dds_set_scale(struct axi_dac *dac, uint32_t chan,
			      int32_t scale_micro_units);
void axi_dac_dds_get_scale(struct axi_dac *dac, uint32_t chan,
			   int32_t *scale_micro_units);
int32_t axi_dac_dds_set_calib_scale(struct axi_dac *dac, uint32_t chan,
				    int32_t val, int32_t val2);
int32_t axi_dac_dds_get_calib_scale(struct axi_dac *dac, uint32_t chan,
				    int32_t *val, int32_t *val2);
int32_t axi_dac_dds_set_calib_phase(struct axi_dac *dac, uint32_t chan,
				    int32_t val, int32_t val2);
int32_t axi_dac_dds_get_calib_phase(struct axi_dac *dac, uint32_t chan,
				    int32_t *val, int32_t *val2);
int32_t axi_dac_datasel(struct axi_dac *dac, int32_t chan,
			enum dds_data_select sel);
int32_t axi_dmac_set_buff(struct axi_dac *dac, uint32_t address, uint16_t *buf,
			  uint32_t buff_size);
uint32_t axi_dmac_set_sine_lut(struct axi_dac *dac, uint32_t address);
#endif

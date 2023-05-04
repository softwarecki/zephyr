/*
 * Copyright (c) 2022 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <zephyr/spinlock.h>

#define LOG_DOMAIN dai_intel_dmic_nhlt
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_DOMAIN);

#include <zephyr/drivers/dai.h>
#include <zephyr/sys/util.h>
#include <adsp_clk.h>
#include "dmic.h"
#include "dmic_nhlt.h"
#include "dmic_regs.h"

extern struct dai_dmic_global_shared dai_dmic_global;

/* Base addresses (in PDM scope) of 2ch PDM controllers and coefficient RAM. */
static const uint32_t base[4] = {PDM0, PDM1, PDM2, PDM3};
static const uint32_t coef_base_a[4] = {PDM0_COEFFICIENT_A, PDM1_COEFFICIENT_A,
					PDM2_COEFFICIENT_A, PDM3_COEFFICIENT_A};
static const uint32_t coef_base_b[4] = {PDM0_COEFFICIENT_B, PDM1_COEFFICIENT_B,
					PDM2_COEFFICIENT_B, PDM3_COEFFICIENT_B};

static inline void dai_dmic_write(const struct dai_intel_dmic *dmic,
				  uint32_t reg, uint32_t val)
{
	sys_write32(val, dmic->reg_base + reg);
}

static inline uint32_t dai_dmic_read(const struct dai_intel_dmic *dmic, uint32_t reg)
{
	return sys_read32(dmic->reg_base + reg);
}

#ifdef CONFIG_SOC_SERIES_INTEL_ACE
static int dai_ipm_source_to_enable(struct dai_intel_dmic *dmic,
				    struct nhlt_pdm_ctrl_cfg **pdm_cfg,
				    int *count, int pdm_count, int stereo,
				    int source_pdm)
{
	int mic_swap;

	if (source_pdm >= CONFIG_DAI_DMIC_HW_CONTROLLERS)
		return -EINVAL;

	if (*count < pdm_count) {
		(*count)++;
		mic_swap = FIELD_GET(MIC_CONTROL_CLK_EDGE, pdm_cfg[source_pdm]->mic_control);
		if (stereo)
			dmic->enable[source_pdm] = 0x3; /* PDMi MIC A and B */
		else
			dmic->enable[source_pdm] = mic_swap ? 0x2 : 0x1; /* PDMi MIC B or MIC A */
	}

	return 0;
}

static int dai_nhlt_dmic_dai_params_get(struct dai_intel_dmic *dmic,
					int32_t *outcontrol,
					struct nhlt_pdm_ctrl_cfg **pdm_cfg,
					struct nhlt_pdm_ctrl_fir_cfg **fir_cfg)
{
	uint32_t outcontrol_val = outcontrol[dmic->dai_config_params.dai_index];
	int num_pdm;
	int source_pdm;
	int ret;
	int n;
	bool stereo_pdm;

	switch (FIELD_GET(OUTCONTROL_OF, outcontrol_val)) {
	case 0:
	case 1:
		dmic->dai_config_params.format = DAI_DMIC_FRAME_S16_LE;
		dmic->dai_config_params.word_size = 16;
		break;
	case 2:
		dmic->dai_config_params.format = DAI_DMIC_FRAME_S32_LE;
		dmic->dai_config_params.word_size = 32;
		break;
	default:
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal OF bit field");
		return -EINVAL;
	}

	num_pdm = FIELD_GET(OUTCONTROL_IPM, outcontrol_val);
	if (num_pdm > CONFIG_DAI_DMIC_HW_CONTROLLERS) {
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal IPM PDM controllers count %d",
			num_pdm);
		return -EINVAL;
	}

	stereo_pdm = FIELD_GET(OUTCONTROL_IPM_SOURCE_MODE, outcontrol_val);

	dmic->dai_config_params.channels = (stereo_pdm + 1) * num_pdm;
	for (n = 0; n < CONFIG_DAI_DMIC_HW_CONTROLLERS; n++)
		dmic->enable[n] = 0;

	n = 0;
	source_pdm = FIELD_GET(OUTCONTROL_IPM_SOURCE_1, outcontrol_val);
	ret = dai_ipm_source_to_enable(dmic, pdm_cfg, &n, num_pdm, stereo_pdm, source_pdm);
	if (ret) {
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal IPM_SOURCE_1");
		return -EINVAL;
	}

	source_pdm = FIELD_GET(OUTCONTROL_IPM_SOURCE_2, outcontrol_val);
	ret = dai_ipm_source_to_enable(dmic, pdm_cfg, &n, num_pdm, stereo_pdm, source_pdm);
	if (ret) {
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal IPM_SOURCE_2");
		return -EINVAL;
	}

	source_pdm = FIELD_GET(OUTCONTROL_IPM_SOURCE_3, outcontrol_val);
	ret = dai_ipm_source_to_enable(dmic, pdm_cfg, &n, num_pdm, stereo_pdm, source_pdm);
	if (ret) {
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal IPM_SOURCE_3");
		return -EINVAL;
	}

	source_pdm = FIELD_GET(OUTCONTROL_IPM_SOURCE_4, outcontrol_val);
	ret = dai_ipm_source_to_enable(dmic, pdm_cfg, &n, num_pdm, stereo_pdm, source_pdm);
	if (ret) {
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal IPM_SOURCE_4");
		return -EINVAL;
	}

	return 0;
}


/*
 * @brief Set clock source used by device
 *
 * @param source Clock source index
 */
static inline void dai_dmic_clock_select_set(const struct dai_intel_dmic *dmic, uint32_t source)
{
	uint32_t val;
#ifdef CONFIG_SOC_INTEL_ACE20_LNL /* Ace 2.0 */
	val = sys_read32(dmic->vshim_base + DMICLVSCTL_OFFSET);
	val &= ~DMICLVSCTL_MLCS;
	val |= FIELD_PREP(DMICLVSCTL_MLCS, source);
	sys_write32(val, dmic->vshim_base + DMICLVSCTL_OFFSET);
#else
	val = sys_read32(dmic->shim_base + DMICLCTL_OFFSET);
	val &= ~DMICLCTL_MLCS;
	val |= FIELD_PREP(DMICLCTL_MLCS, source);
	sys_write32(val, dmic->shim_base + DMICLCTL_OFFSET);
#endif
}

/*
 * @brief Get clock source used by device
 *
 * @return Clock source index
 */
static inline uint32_t dai_dmic_clock_select_get(const struct dai_intel_dmic *dmic)
{
	uint32_t val;
#ifdef CONFIG_SOC_INTEL_ACE20_LNL /* Ace 2.0 */
	val = sys_read32(dmic->vshim_base + DMICLVSCTL_OFFSET);
	val &= ~DMICLCTL_MLCS;
	return FIELD_GET(DMICLVSCTL_MLCS, val);
#else
	val = sys_read32(dmic->shim_base + DMICLCTL_OFFSET);
	return FIELD_GET(DMICLCTL_MLCS, val);
#endif
}

/*
 * @brief Set clock source used by device
 *
 * @param source Clock source index
 */
static int dai_dmic_set_clock(const struct dai_intel_dmic *dmic, const uint8_t clock_source)
{
	LOG_DBG("dai_dmic_set_clock(): clock_source = %u", clock_source);

	if (!adsp_clock_souce_is_supported(clock_source)) {
		return -ENOTSUP;
	}

#ifndef CONFIG_SOC_INTEL_ACE20_LNL /* Ace 2.0 */
	if (clock_source && !(sys_read32(dmic->shim_base + DMICLCAP_OFFSET) & DMICLCAP_MLCS)) {
		return -ENOTSUP;
	}
#endif

	dai_dmic_clock_select_set(dmic, clock_source);
	return 0;
}

static inline void dai_dmic_pdm_config_clear(const struct dai_intel_dmic *dmic,
					     const uint32_t pdm_base)
{
	const uint32_t regs_to_clear[] = { CIC_CONFIG, MIC_CONTROL,
		FIR_CONTROL_A, FIR_CONFIG_A, DC_OFFSET_LEFT_A, DC_OFFSET_RIGHT_A, OUT_GAIN_LEFT_A,
		OUT_GAIN_RIGHT_A,
		FIR_CONTROL_B, FIR_CONFIG_B, DC_OFFSET_LEFT_B, DC_OFFSET_RIGHT_B, OUT_GAIN_LEFT_B,
		OUT_GAIN_RIGHT_B
	};

	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_clear); i++) {
		dai_dmic_write(dmic, pdm_base + regs_to_clear[i], 0);
	}

	/* clear soft_reset, mute mic by default */
	dai_dmic_write(dmic, pdm_base + CIC_CONTROL, CIC_CONTROL_MIC_MUTE);
}

static inline void dai_dmic_pdm_config_copy(const struct dai_intel_dmic *dmic,
					    const uint32_t pdm_base, const uint32_t *config)
{
	const uint32_t regs_to_copy[] = { CIC_CONFIG,
		FIR_CONTROL_A, FIR_CONFIG_A, DC_OFFSET_LEFT_A, DC_OFFSET_RIGHT_A, OUT_GAIN_LEFT_A,
		OUT_GAIN_RIGHT_A,
		FIR_CONTROL_B, FIR_CONFIG_B, DC_OFFSET_LEFT_B, DC_OFFSET_RIGHT_B, OUT_GAIN_LEFT_B,
		OUT_GAIN_RIGHT_B
	};

	int i;

	for (i = 0; i < ARRAY_SIZE(regs_to_copy); i++) {
		dai_dmic_write(dmic, pdm_base + regs_to_copy[i], config[i]);
	}
}

/*
* Return decimator's mask where:
1st decimator is ipm_source_1, 2nd ipm_source_2, etc.
* And set appriopriate bit for each decimator:
* bit 0x1 - decimator 0
* bit 0x2 - decimator 1
* bit 0x4 - decimator 2
* bit 0x8 - decimator 3
*/
static uint32_t get_decimator_source_mask(uint32_t outcontrol)
{
	uint32_t ipm;
	uint32_t decimator_mask = 0;

	ipm = FIELD_GET(OUTCONTROL_IPM, outcontrol);

	/* only 1 - 4 number of decimators is allowed */
	if (ipm > 0 && ipm <= 4) {
		switch (ipm) {
		case 4:
			decimator_mask |= 1 << FIELD_GET(OUTCONTROL_IPM_SOURCE_4, outcontrol);
			/* fallback */
		case 3:
			decimator_mask |= 1 << FIELD_GET(OUTCONTROL_IPM_SOURCE_3, outcontrol);
			/* fallback */
		case 2:
			decimator_mask |= 1 << FIELD_GET(OUTCONTROL_IPM_SOURCE_2, outcontrol);
			/* fallback */
		case 1:
			decimator_mask |= 1 << FIELD_GET(OUTCONTROL_IPM_SOURCE_1, outcontrol);
		}
	}

	return decimator_mask;
}

/* Returns pointer right after coefficients data */
static const uint32_t *dai_dmic_configure_coeff(const struct dai_intel_dmic *dmic,
						const uint32_t pdm_base,
						const uint32_t *coeffs_in_dw)
{
	const uint8_t *coeffs_in_bytes;
	int i, length_0, length_1;
	uint32_t coeff_out;
	uint32_t coeff_val;

	/* Read fir_lengths */
	length_0 = FIELD_GET(FIR_CONFIG_FIR_LENGTH, dai_dmic_read(dmic, pdm_base + FIR_CONFIG_A));
	if (length_0) {
		/* fir length in reg is n-1, hence ++ */
		length_0++;
	}

	length_1 = FIELD_GET(FIR_CONFIG_FIR_LENGTH, dai_dmic_read(dmic, pdm_base + FIR_CONFIG_B));
	if (length_1) {
		/* fir length in reg is n-1, hence ++ */
		length_1++;
	}

	if (length_0 > 256 || length_1 > 256) {
		LOG_ERR("dai_dmic_configure_coeff(): invalid coeff length! %d %d", length_0,
			length_1);
		return NULL;
	}

	if (*coeffs_in_dw != FIR_COEFFS_PACKED_TO_24_BITS) {
		coeff_out = pdm_base + PDM_COEFFICIENT_A;
		for (i = 0; i < length_0; i++, coeff_out += 4) {
			dai_dmic_write(dmic, coeff_out, *coeffs_in_dw++);
		}

		coeff_out = pdm_base + PDM_COEFFICIENT_B;
		for (i = 0; i < length_1; i++, coeff_out += 4) {
			dai_dmic_write(dmic, coeff_out, *coeffs_in_dw++);
		}

		return coeffs_in_dw;
	} else {
		/* First dword is not included into length_0 and length_1 - skip it. */
		coeffs_in_dw++;
		
		const uint8_t *coeffs_in_bytes = (const uint8_t *)coeffs_in_dw;

		coeff_out = pdm_base + PDM_COEFFICIENT_A;
		for (i = 0; i < length_0; i++, coeff_out += 4) {
			coeff_val = coeffs_in_bytes[0] +
				(coeffs_in_bytes[1] << 8) +
				(coeffs_in_bytes[2] << 16);
			coeffs_in_bytes += 3;

			dai_dmic_write(dmic, coeff_out, coeff_val);
		}

		coeff_out = pdm_base + PDM_COEFFICIENT_B;
		for (i = 0; i < length_1; i++, coeff_out += 4) {
			coeff_val = coeffs_in_bytes[0] +
				(coeffs_in_bytes[1] << 8) +
				(coeffs_in_bytes[2] << 16);
			coeffs_in_bytes += 3;

			dai_dmic_write(dmic, coeff_out, coeff_val);
		}

		return UINT_TO_POINTER(ROUND_UP(POINTER_TO_UINT(coeffs_in_bytes),
						sizeof(uint32_t)));
	}
}


#else
static int dai_nhlt_dmic_dai_params_get(struct dai_intel_dmic *dmic,
					int32_t *outcontrol,
					struct nhlt_pdm_ctrl_cfg **pdm_cfg,
					struct nhlt_pdm_ctrl_fir_cfg **fir_cfg)
{
	int fir_stereo[2];
	int mic_swap;

	switch (FIELD_GET(OUTCONTROL_OF, outcontrol[dmic->dai_config_params.dai_index])) {
	case 0:
	case 1:
		dmic->dai_config_params.format = DAI_DMIC_FRAME_S16_LE;
		break;
	case 2:
		dmic->dai_config_params.format = DAI_DMIC_FRAME_S32_LE;
		break;
	default:
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal OF bit field");
		return -EINVAL;
	}

	switch (FIELD_GET(OUTCONTROL_IPM, outcontrol[dmic->dai_config_params.dai_index])) {
	case 0:
		if (!fir_cfg[0])
			return -EINVAL;

		fir_stereo[0] = FIELD_GET(FIR_CONTROL_STEREO, fir_cfg[0]->fir_control);
		if (fir_stereo[0]) {
			dmic->dai_config_params.channels = 2;
			dmic->enable[0] = 0x3; /* PDM0 MIC A and B */
			dmic->enable[1] = 0x0;	/* PDM1 none */

		} else {
			dmic->dai_config_params.channels = 1;
			mic_swap = FIELD_GET(MIC_CONTROL_CLK_EDGE, pdm_cfg[0]->mic_control);
			dmic->enable[0] = mic_swap ? 0x2 : 0x1; /* PDM0 MIC B or MIC A */
			dmic->enable[1] = 0x0;	/* PDM1 */
		}
		break;
	case 1:
		if (!fir_cfg[1])
			return -EINVAL;

		fir_stereo[1] = FIELD_GET(FIR_CONTROL_STEREO_GET, fir_cfg[1]->fir_control);
		if (fir_stereo[1]) {
			dmic->dai_config_params.channels = 2;
			dmic->enable[0] = 0x0; /* PDM0 none */
			dmic->enable[1] = 0x3;	/* PDM1 MIC A and B */
		} else {
			dmic->dai_config_params.channels = 1;
			dmic->enable[0] = 0x0; /* PDM0 none */
			mic_swap = FIELD_GET(MIC_CONTROL_CLK_EDGE, pdm_cfg[1]->mic_control);
			dmic->enable[1] = mic_swap ? 0x2 : 0x1; /* PDM1 MIC B or MIC A */
		}
		break;
	case 2:
		if (!fir_cfg[0] || !fir_cfg[0])
			return -EINVAL;

		fir_stereo[0] = FIELD_GET(FIR_CONTROL_STEREO, fir_cfg[0]->fir_control);
		fir_stereo[1] = FIELD_GET(FIR_CONTROL_STEREO, fir_cfg[1]->fir_control);
		if (fir_stereo[0] == fir_stereo[1]) {
			dmic->dai_config_params.channels = 4;
			dmic->enable[0] = 0x3; /* PDM0 MIC A and B */
			dmic->enable[1] = 0x3;	/* PDM1 MIC A and B */
			LOG_INF("nhlt_dmic_dai_params_get(): set 4ch pdm0 and pdm1");
		} else {
			LOG_ERR("nhlt_dmic_dai_params_get(): Illegal 4ch configuration");
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("nhlt_dmic_dai_params_get(): Illegal OF bit field");
		return -EINVAL;
	}

	return 0;
}

static inline int dai_dmic_set_clock(uint8_t clock_source)
{
	return 0;
}
#endif




int dai_dmic_set_config_nhlt_new(struct dai_intel_dmic *dmic, const void *bespoke_cfg)
{
	const struct nhlt_dmic_config_blob *cfg = bespoke_cfg;
	const struct nhlt_dmic_channel_config *channel_cfg = cfg->channel_config;
	const struct nhlt_pdm_ctrl_mask *pdm_ctrl;
	const struct nhlt_pdm_ctrl_cfg *pdm_cfg;

	int channel_idx, pdm_idx;
	uint8_t channel_ctrl_mask;
	uint32_t pdm_ctrl_mask;
	uint32_t base, val;
	uint32_t pdm_a_mask;
	uint32_t pdm_b_mask;
	int ret;

	const uint32_t *fir_coeffs;

	/* Array of pointers to pdm coefficient data. Used to reuse coefficient from another pdm. */
	const uint32_t *pdm_coeff_ptr[CONFIG_DAI_DMIC_HW_CONTROLLERS] = { 0 };



	if (dmic->dai_config_params.dai_index >= DMIC_HW_FIFOS_MAX) {
		LOG_ERR("dmic_set_config_nhlt(): illegal DAI index %d",
			dmic->dai_config_params.dai_index);
		return -EINVAL;
	}

	/* Channel_ctlr_mask bits indicate the FIFOs enabled */
	channel_ctrl_mask = cfg->ctrl_mask.channel_ctrl_mask;

	if (!channel_ctrl_mask || channel_ctrl_mask & ~BIT_MASK(DMIC_HW_FIFOS_MAX)) {
		LOG_ERR("dmic_set_config_nhlt(): invalid channel_ctrl_mask %u", channel_ctrl_mask);
		return -EINVAL;
	}

	LOG_DBG("dmic_set_config_nhlt(): channel_ctrl_mask = %u, number of FIFOs %d",
		channel_ctrl_mask, POPCOUNT(channel_ctrl_mask));

	// TODO: Check capabilities

	/* Configure clock source */
	ret = dai_dmic_set_clock(dmic, cfg->ctrl_mask.clock_source);
	if (ret)
		return ret;

	/* Get OUTCONTROLx configuration */
	for (channel_idx = 0; channel_idx < DMIC_HW_FIFOS_MAX; channel_idx++) {
		dmic->dai_conf.ignore_ctrls_enable[channel_idx] = false;

		// TODO: TO chyba powinno byc przed continue?
		/* clear value pdm_ctrl_routed_to_ch_cfg for channel_id. This field will be updated
		 * in this loop (pdm_controller_routed) and later during updating
		 * PdmController registers */
		dmic->dai_conf.pdm_ctrl_routed_to_ch_cfg[channel_idx] = 0;

		if ((channel_ctrl_mask & BIT(channel_idx)) == 0) {
			continue;
		}

		/* store outctrl register without any ingerention */
		dai_dmic_write(dmic, PDM_CHANNEL_REGS_SIZE * channel_idx + OUTCONTROL,
			       channel_cfg->out_control);

		/* ipm means number of decimators used for each channel and ipm_source_index'es
		 * specify which exact pdm's are used */
		dmic->dai_conf.pdm_ctrl_routed_to_ch_cfg[channel_idx] =
			get_decimator_source_mask(channel_cfg->out_control);




		// TODO: To było w pętli pdm, ale to chyba bez sensu?
		// SOF ma to fajnie wydzielone do funkcji
		// Jeśli to jest faktycznie uzywane, wydziel to do funckji
		val = channel_cfg->out_control;

		if (FIELD_GET(OUTCONTROL_IPM, val) == 1)
		{
			if ((val & OUTCONTROL_IPM_SOURCE_MODE) == 0) {
				/* 1 decimator and mono = mono */
				// TODO: SOF tutaj jeszcze wybieral który mikrofon jest uzywany w trybie mono
				dmic->dai_conf.mics_mask[channel_idx] = 0x1 << (channel_idx * 2);
			} else {
				/* 1 decimator and stereo = stereo */
				dmic->dai_conf.mics_mask[channel_idx] = 0x3 << (channel_idx * 2);
			}
		}
		else if (FIELD_GET(OUTCONTROL_IPM, val) == 2)
		{
			/* 2 decimators and stereo = quadro */
			if (val & OUTCONTROL_IPM_SOURCE_MODE) {
				dmic->dai_conf.mics_mask[channel_idx] = 0xFF;
			}
		}




		channel_cfg++;
	}

	/* pdm_ctrl_mask is placed right after channels config. */
	pdm_ctrl = (const struct nhlt_pdm_ctrl_mask *)channel_cfg;
	pdm_ctrl_mask = pdm_ctrl->pdm_ctrl_mask;

	if (!pdm_ctrl_mask || pdm_ctrl_mask & ~BIT_MASK(CONFIG_DAI_DMIC_HW_CONTROLLERS)) {
		LOG_ERR("dmic_set_config_nhlt(): invalid pdm_ctrl_mask %u", pdm_ctrl_mask);
		return -EINVAL;
	}

	LOG_DBG("dmic_set_config_nhlt(): pdm_ctrl_mask = %d, number of PDMs %d",
		pdm_ctrl_mask, POPCOUNT(pdm_ctrl_mask));

	/* increment pos_in_struct(located right after pdm_ctrl_mask) */
	pdm_cfg = (const struct nhlt_pdm_ctrl_cfg *)(pdm_ctrl + 1);


	dmic->dai_conf.array_enabled = false;
	dmic->dai_conf.sync_enabled  = false;
	for (pdm_idx = 0; pdm_idx < CONFIG_DAI_DMIC_HW_CONTROLLERS; pdm_idx++) {
		base = PDM_CONTROLLER_BASE + pdm_idx * PDM_CONTROLLER_REGS_SIZE;


		if ((pdm_ctrl_mask & BIT(pdm_idx)) == 0) {
			dai_dmic_pdm_config_clear(dmic, base);
			continue;
		}
	
		// TODO: Tutaj było wypełnianie dmic_control / misc_mask

		/* update pdm_config with fir_hiq information */
		val = pdm_cfg->fir_config[0].fir_control;
		dmic->dai_conf.pdm[pdm_idx].fir_0_stereo = !!(val & FIR_CONTROL_STEREO);

		/* update pdm_config with fir_voice information */
		val = pdm_cfg->fir_config[1].fir_control;
		dmic->dai_conf.pdm[pdm_idx].fir_1_stereo = !!(val & FIR_CONTROL_STEREO);

		dmic->dai_conf.array_enabled |= !!(val & FIR_CONTROL_ARRAY_START_EN);
		dmic->dai_conf.sync_enabled |= !!(val & FIR_CONTROL_PERIODIC_START_EN);

		/* Copy configuration to pdm registers. Skips CIC_CONTROL and MIC_CONTROL */
		dai_dmic_pdm_config_copy(dmic, base, (const uint32_t *)pdm_cfg);

		val = pdm_cfg->mic_control;
		/* this needs to be cleared to prevent too early PDM enable and should be enabled
		 * only during dmic_enable */
		val &= ~(MIC_CONTROL_PDM_EN_B | MIC_CONTROL_PDM_EN_A);
		dai_dmic_write(dmic, base + MIC_CONTROL, val);

		/* Configure fir coefficients */

		/* Check if FIR coeffs should be reused */
		if (pdm_cfg->reuse_fir_from_pdm == 0) {
			/* get ptr, where FIR coeffs starts */
			fir_coeffs = pdm_cfg->fir_coeffs;

			/* and save it for future pdms reference */
			pdm_coeff_ptr[pdm_idx] = fir_coeffs;
		} else {
			if (pdm_cfg->reuse_fir_from_pdm > pdm_idx) {
				LOG_ERR("dmic_set_config_nhlt(): invalid reuse fir index %u",
					pdm_cfg->reuse_fir_from_pdm);
				return -EINVAL;
			}

			/* get FIR coeffs from another pdm */
			fir_coeffs = pdm_coeff_ptr[pdm_cfg->reuse_fir_from_pdm - 1];

			if (!fir_coeffs) {
				LOG_ERR("dmic_set_config_nhlt(): unable to reuse fir from %u",
					pdm_cfg->reuse_fir_from_pdm);
				return -EINVAL;
			}
		}
		
		fir_coeffs = dai_dmic_configure_coeff(dmic, base, fir_coeffs);

		/* program register with soft_reset */
		dai_dmic_write(dmic, base + CIC_CONTROL, pdm_cfg->cic_control);

		// TODO: FIX ME!!!!!!!!!!!!
#if 0// SUPPORTED(ALH) && 0 // PDM soundwire streams is defeatured from ACE
		DMICxPyPDMSM pdmsm                   = {.full = pdm_ctrl_cfg->pdmsm};
		dmic_shim->pdmsm[pdm_ctrl_idx].full = pdmsm.full;
#endif // SUPPORTED(ALH)

		/* Update pdm_cfg ptr for next PDM Ctrl. */
		if (pdm_cfg->reuse_fir_from_pdm) {
			/* fir_coeffs array is empty if reusing previous coeffs */
			pdm_cfg = (const struct nhlt_pdm_ctrl_cfg *)&pdm_cfg->fir_coeffs;
		} else {
			pdm_cfg = (const struct nhlt_pdm_ctrl_cfg *)fir_coeffs;
		}
	}

	val = dai_dmic_read(dmic, PDM_CONTROLLER_BASE + MIC_CONTROL);
	val &= ~MIC_CONTROL_SLAVE_MODE;
	dai_dmic_write(dmic, PDM_CONTROLLER_BASE + MIC_CONTROL, val);
}


int dai_dmic_set_config_nhlt(struct dai_intel_dmic *dmic, const void *bespoke_cfg)
{
	struct nhlt_pdm_ctrl_cfg *pdm_cfg[DMIC_HW_CONTROLLERS_MAX];
	struct nhlt_pdm_ctrl_fir_cfg *fir_cfg_a[DMIC_HW_CONTROLLERS_MAX];
	struct nhlt_pdm_ctrl_fir_cfg *fir_cfg_b[DMIC_HW_CONTROLLERS_MAX];
	struct nhlt_pdm_fir_coeffs *fir_a[DMIC_HW_CONTROLLERS_MAX] = {NULL};
	struct nhlt_pdm_fir_coeffs *fir_b[DMIC_HW_CONTROLLERS_MAX];
	struct nhlt_dmic_channel_ctrl_mask *dmic_cfg;

	uint32_t out_control[DMIC_HW_FIFOS_MAX] = {0};
	uint32_t channel_ctrl_mask;
	uint32_t fir_control;
	uint32_t pdm_ctrl_mask;
	uint32_t ref = 0;
	uint32_t val;
	const uint8_t *p = bespoke_cfg;
	int num_fifos;
	int num_pdm;
	int fir_length_a;
	int fir_length_b;
	int n;
	int i;
	int rate_div;
	int clk_div;
	int comb_count;
	int fir_decimation, fir_shift, fir_length;
	int bf1, bf2, bf3, bf4, bf5, bf6, bf7, bf8;
#ifdef CONFIG_SOC_SERIES_INTEL_ACE
	int bf9, bf10, bf11, bf12, bf13;
#endif
	int bfth;
	int ret;
	int p_mcic = 0;
	int p_mfira = 0;
	int p_mfirb = 0;
	int p_clkdiv = 0;

	if (dmic->dai_config_params.dai_index >= DMIC_HW_FIFOS_MAX) {
		LOG_ERR("dmic_set_config_nhlt(): illegal DAI index %d",
			 dmic->dai_config_params.dai_index);
		return -EINVAL;
	}

	/* Skip not used headers */
	p += sizeof(struct nhlt_dmic_gateway_attributes);
	p += sizeof(struct nhlt_dmic_ts_group);
	p += sizeof(struct nhlt_dmic_global_config);

	/* Channel_ctlr_mask bits indicate the FIFOs enabled*/
	dmic_cfg = (struct nhlt_dmic_channel_ctrl_mask *)p;
	channel_ctrl_mask = dmic_cfg->channel_ctrl_mask;
	num_fifos = POPCOUNT(channel_ctrl_mask); /* Count set bits */
	p += sizeof(struct nhlt_dmic_channel_ctrl_mask);
	LOG_DBG("dmic_set_config_nhlt(): channel_ctrl_mask = %d", channel_ctrl_mask);

	/* Configure clock source */
	ret = dai_dmic_set_clock(dmic, dmic_cfg->clock_source);
	if (ret)
		return ret;

	/* Get OUTCONTROLx configuration */
	if (num_fifos < 1 || num_fifos > DMIC_HW_FIFOS_MAX) {
		LOG_ERR("dmic_set_config_nhlt(): illegal number of FIFOs %d", num_fifos);
		return -EINVAL;
	}

	for (n = 0; n < DMIC_HW_FIFOS_MAX; n++) {
		if (!(channel_ctrl_mask & (1 << n)))
			continue;

		val = *(uint32_t *)p;
		out_control[n] = val;
		bf1 = FIELD_GET(OUTCONTROL_TIE, val);
		bf2 = FIELD_GET(OUTCONTROL_SIP, val);
		bf3 = FIELD_GET(OUTCONTROL_FINIT, val);
		bf4 = FIELD_GET(OUTCONTROL_FCI, val);
		bf5 = FIELD_GET(OUTCONTROL_BFTH, val);
		bf6 = FIELD_GET(OUTCONTROL_OF, val);
		bf7 = FIELD_GET(OUTCONTROL_IPM, val);
		bf8 = FIELD_GET(OUTCONTROL_TH, val);
		LOG_INF("dmic_set_config_nhlt(): OUTCONTROL%d = %08x", n, out_control[n]);
		LOG_INF("  tie=%d, sip=%d, finit=%d, fci=%d", bf1, bf2, bf3, bf4);
		LOG_INF("  bfth=%d, of=%d, ipm=%d, th=%d", bf5, bf6, bf7, bf8);
		if (bf5 > OUTCONTROL_BFTH_MAX) {
			LOG_ERR("dmic_set_config_nhlt(): illegal BFTH value");
			return -EINVAL;
		}

#ifdef CONFIG_SOC_SERIES_INTEL_ACE
		bf9 = FIELD_GET(OUTCONTROL_IPM_SOURCE_1, val);
		bf10 = FIELD_GET(OUTCONTROL_IPM_SOURCE_2, val);
		bf11 = FIELD_GET(OUTCONTROL_IPM_SOURCE_3, val);
		bf12 = FIELD_GET(OUTCONTROL_IPM_SOURCE_4, val);
		bf13 = FIELD_GET(OUTCONTROL_IPM_SOURCE_MODE, val);
		LOG_INF("  ipms1=%d, ipms2=%d, ipms3=%d, ipms4=%d", bf9, bf10, bf11, bf12);
		LOG_INF("  ipms_mode=%d", bf13);
		ref =	FIELD_PREP(OUTCONTROL_TIE, bf1) | FIELD_PREP(OUTCONTROL_SIP, bf2) |
			FIELD_PREP(OUTCONTROL_FINIT, bf3) | FIELD_PREP(OUTCONTROL_FCI, bf4) |
			FIELD_PREP(OUTCONTROL_BFTH, bf5) | FIELD_PREP(OUTCONTROL_OF, bf6) |
			FIELD_PREP(OUTCONTROL_IPM, bf7) | FIELD_PREP(OUTCONTROL_IPM_SOURCE_1, bf9) |
			FIELD_PREP(OUTCONTROL_IPM_SOURCE_2, bf10) |
			FIELD_PREP(OUTCONTROL_IPM_SOURCE_3, bf11) |
			FIELD_PREP(OUTCONTROL_IPM_SOURCE_4, bf12) | FIELD_PREP(OUTCONTROL_TH, bf8) |
			FIELD_PREP(OUTCONTROL_IPM_SOURCE_MODE, bf13);
#else
		ref = FIELD_PREP(OUTCONTROL_TIE, bf1) | FIELD_PREP(OUTCONTROL_SIP, bf2) |
			FIELD_PREP(OUTCONTROL_FINIT, bf3) | FIELD_PREP(OUTCONTROL_FCI, bf4) |
			FIELD_PREP(OUTCONTROL_BFTH, bf5) | FIELD_PREP(OUTCONTROL_OF, bf6) |
			FIELD_PREP(OUTCONTROL_IPM, bf7) | FIELD_PREP(OUTCONTROL_TH, bf8);
#endif
		if (ref != val) {
			LOG_ERR("dmic_set_config_nhlt(): illegal OUTCONTROL%d = 0x%08x",
				n, val);
			return -EINVAL;
		}

		p += sizeof(uint32_t);
	}

	/* Write the FIFO control registers. The clear/set of bits is the same for
	 * all DMIC_HW_VERSION
	 */
	/* Clear TIE, SIP, FCI, set FINIT, the rest of bits as such */
	val = (out_control[dmic->dai_config_params.dai_index] &
		~(OUTCONTROL_TIE | OUTCONTROL_SIP | OUTCONTROL_FCI)) |
		OUTCONTROL_FINIT;
	if (dmic->dai_config_params.dai_index == 0)
		dai_dmic_write(dmic, OUTCONTROL0, val);
	else
		dai_dmic_write(dmic, OUTCONTROL1, val);

	LOG_INF("dmic_set_config_nhlt(): OUTCONTROL%d = %08x",
		 dmic->dai_config_params.dai_index, val);

	/* Pass 2^BFTH to plat_data fifo depth. It will be used later in DMA
	 * configuration
	 */
	bfth = FIELD_GET(OUTCONTROL_BFTH, val);
	dmic->fifo.depth = 1 << bfth;

	/* Get PDMx registers */
	pdm_ctrl_mask = ((struct nhlt_pdm_ctrl_mask *)p)->pdm_ctrl_mask;
	num_pdm = POPCOUNT(pdm_ctrl_mask); /* Count set bits */
	p += sizeof(struct nhlt_pdm_ctrl_mask);
	LOG_DBG("dmic_set_config_nhlt(): pdm_ctrl_mask = %d", pdm_ctrl_mask);
	if (num_pdm < 1 || num_pdm > CONFIG_DAI_DMIC_HW_CONTROLLERS) {
		LOG_ERR("dmic_set_config_nhlt(): illegal number of PDMs %d", num_pdm);
		return -EINVAL;
	}

	for (n = 0; n < CONFIG_DAI_DMIC_HW_CONTROLLERS; n++) {
		fir_cfg_a[n] = NULL;
		fir_cfg_b[n] = NULL;

		if (!(pdm_ctrl_mask & (1 << n))) {
			/* Set MIC_MUTE bit to unused PDM */
			dai_dmic_write(dmic, base[n] + CIC_CONTROL, CIC_CONTROL_MIC_MUTE);
			continue;
		}

		LOG_DBG("dmic_set_config_nhlt(): PDM%d", n);

		/* Get CIC configuration */
		pdm_cfg[n] = (struct nhlt_pdm_ctrl_cfg *)p;
		p += sizeof(struct nhlt_pdm_ctrl_cfg);

		comb_count = FIELD_GET(CIC_CONFIG_COMB_COUNT, pdm_cfg[n]->cic_config);
		p_mcic = comb_count + 1;
		clk_div = FIELD_GET(MIC_CONTROL_PDM_CLKDIV, pdm_cfg[n]->mic_control);
		p_clkdiv = clk_div + 2;
		if (dai_dmic_global.active_fifos_mask == 0) {
			val = pdm_cfg[n]->cic_control;
			bf1 = FIELD_GET(CIC_CONTROL_SOFT_RESET, val);
			bf2 = FIELD_GET(CIC_CONTROL_CIC_START_B, val);
			bf3 = FIELD_GET(CIC_CONTROL_CIC_START_A, val);
			bf4 = FIELD_GET(CIC_CONTROL_MIC_B_POLARITY, val);
			bf5 = FIELD_GET(CIC_CONTROL_MIC_A_POLARITY, val);
			bf6 = FIELD_GET(CIC_CONTROL_MIC_MUTE, val);
#ifndef CONFIG_SOC_SERIES_INTEL_ACE
			bf7 = FIELD_GET(CIC_CONTROL_STEREO_MODE, val);
#else
			bf7 = -1;
#endif
			LOG_DBG("dmic_set_config_nhlt(): CIC_CONTROL = %08x", val);
			LOG_DBG("  soft_reset=%d, cic_start_b=%d, cic_start_a=%d",
				bf1, bf2, bf3);
			LOG_DBG("  mic_b_polarity=%d, mic_a_polarity=%d, mic_mute=%d",
				bf4, bf5, bf6);
			ref = FIELD_PREP(CIC_CONTROL_SOFT_RESET, bf1) |
				FIELD_PREP(CIC_CONTROL_CIC_START_B, bf2) |
				FIELD_PREP(CIC_CONTROL_CIC_START_A, bf3) |
				FIELD_PREP(CIC_CONTROL_MIC_B_POLARITY, bf4) |
				FIELD_PREP(CIC_CONTROL_MIC_A_POLARITY, bf5) |
				FIELD_PREP(CIC_CONTROL_MIC_MUTE, bf6)
#ifndef CONFIG_SOC_SERIES_INTEL_ACE
				| FIELF_PREP(CIC_CONTROL_STEREO_MODE, bf7)
#endif
				;
			LOG_DBG("  stereo_mode=%d", bf7);
			if (ref != val) {
				LOG_ERR("dmic_set_config_nhlt(): illegal CIC_CONTROL = 0x%08x",
					val);
				return -EINVAL;
			}

			/* Clear CIC_START_A and CIC_START_B */
			val = (val & ~(CIC_CONTROL_CIC_START_A | CIC_CONTROL_CIC_START_B));
			dai_dmic_write(dmic, base[n] + CIC_CONTROL, val);
			LOG_DBG("dmic_set_config_nhlt(): CIC_CONTROL = %08x", val);

			val = pdm_cfg[n]->cic_config;
			bf1 = FIELD_GET(CIC_CONFIG_CIC_SHIFT, val);
			LOG_DBG("dmic_set_config_nhlt(): CIC_CONFIG = %08x", val);
			LOG_DBG("  cic_shift=%d, comb_count=%d", bf1, comb_count);

			/* Use CIC_CONFIG as such */
			dai_dmic_write(dmic, base[n] + CIC_CONFIG, val);
			LOG_DBG("dmic_set_config_nhlt(): CIC_CONFIG = %08x", val);

			val = pdm_cfg[n]->mic_control;
#ifndef CONFIG_SOC_SERIES_INTEL_ACE
			bf1 = FIELD_GET(MIC_CONTROL_PDM_SKEW, val);
#else
			bf1 = -1;
#endif
			bf2 = FIELD_GET(MIC_CONTROL_CLK_EDGE, val);
			bf3 = FIELD_GET(MIC_CONTROL_PDM_EN_B, val);
			bf4 = FIELD_GET(MIC_CONTROL_PDM_EN_A, val);
			LOG_DBG("dmic_set_config_nhlt(): MIC_CONTROL = %08x", val);
			LOG_DBG("  clkdiv=%d, skew=%d, clk_edge=%d", clk_div, bf1, bf2);
			LOG_DBG("  en_b=%d, en_a=%d", bf3, bf4);

			/* Clear PDM_EN_A and PDM_EN_B */
			val &= ~(MIC_CONTROL_PDM_EN_A | MIC_CONTROL_PDM_EN_B);
			dai_dmic_write(dmic, base[n] + MIC_CONTROL, val);
			LOG_DBG("dmic_set_config_nhlt(): MIC_CONTROL = %08x", val);
		}

		/* FIR A */
		fir_cfg_a[n] = (struct nhlt_pdm_ctrl_fir_cfg *)p;
		p += sizeof(struct nhlt_pdm_ctrl_fir_cfg);
		val = fir_cfg_a[n]->fir_config;
		fir_length = FIELD_GET(FIR_CONFIG_FIR_LENGTH, val);
		fir_length_a = fir_length + 1; /* Need for parsing */
		fir_decimation = FIELD_GET(FIR_CONFIG_FIR_DECIMATION, val);
		p_mfira = fir_decimation + 1;
		if (dmic->dai_config_params.dai_index == 0) {
			fir_shift = FIELD_GET(FIR_CONFIG_FIR_SHIFT, val);
			LOG_DBG("dmic_set_config_nhlt(): FIR_CONFIG_A = %08x", val);
			LOG_DBG("  fir_decimation=%d, fir_shift=%d, fir_length=%d",
				fir_decimation, fir_shift, fir_length);

			/* Use FIR_CONFIG_A as such */
			dai_dmic_write(dmic, base[n] + FIR_CONFIG_A, val);
			LOG_DBG("configure_registers(), FIR_CONFIG_A = %08x", val);

			val = fir_cfg_a[n]->fir_control;
			bf1 = FIELD_GET(FIR_CONTROL_START, val);
			bf2 = FIELD_GET(FIR_CONTROL_ARRAY_START_EN, val);
			bf3 = FIELD_GET(FIR_CONTROL_PERIODIC_START_EN, val);
			bf4 = FIELD_GET(FIR_CONTROL_DCCOMP, val);
			bf5 = FIELD_GET(FIR_CONTROL_MUTE, val);
			bf6 = FIELD_GET(FIR_CONTROL_STEREO, val);
			LOG_DBG("dmic_set_config_nhlt(): FIR_CONTROL_A = %08x", val);
			LOG_DBG("  start=%d, array_start_en=%d, periodic_start_en=%d",
				bf1, bf2, bf3);
			LOG_DBG("  dccomp=%d, mute=%d, stereo=%d", bf4, bf5, bf6);
			ref = FIELD_PREP(FIR_CONTROL_START, bf1) |
				FIELD_PREP(FIR_CONTROL_ARRAY_START_EN, bf2) |
				FIELD_PREP(FIR_CONTROL_PERIODIC_START_EN, bf3) |
				FIELD_PREP(FIR_CONTROL_DCCOMP, bf4) |
				FIELD_PREP(FIR_CONTROL_MUTE, bf5) |
				FIELD_PREP(FIR_CONTROL_STEREO, bf6);

			if (ref != val) {
				LOG_ERR("dmic_set_config_nhlt(): illegal FIR_CONTROL = 0x%08x",
					val);
				return -EINVAL;
			}

			/* Clear START, set MUTE */
			fir_control = (val & ~FIR_CONTROL_START) | FIR_CONTROL_MUTE;
			dai_dmic_write(dmic, base[n] + FIR_CONTROL_A, fir_control);
			LOG_DBG("dmic_set_config_nhlt(): FIR_CONTROL_A = %08x", fir_control);

			/* Use DC_OFFSET and GAIN as such */
			val = fir_cfg_a[n]->dc_offset_left;
			dai_dmic_write(dmic, base[n] + DC_OFFSET_LEFT_A, val);
			LOG_DBG("dmic_set_config_nhlt(): DC_OFFSET_LEFT_A = %08x", val);

			val = fir_cfg_a[n]->dc_offset_right;
			dai_dmic_write(dmic, base[n] + DC_OFFSET_RIGHT_A, val);
			LOG_DBG("dmic_set_config_nhlt(): DC_OFFSET_RIGHT_A = %08x", val);

			val = fir_cfg_a[n]->out_gain_left;
			dai_dmic_write(dmic, base[n] + OUT_GAIN_LEFT_A, val);
			LOG_DBG("dmic_set_config_nhlt(): OUT_GAIN_LEFT_A = %08x", val);

			val = fir_cfg_a[n]->out_gain_right;
			dai_dmic_write(dmic, base[n] + OUT_GAIN_RIGHT_A, val);
			LOG_DBG("dmic_set_config_nhlt(): OUT_GAIN_RIGHT_A = %08x", val);
		}

		/* FIR B */
		fir_cfg_b[n] = (struct nhlt_pdm_ctrl_fir_cfg *)p;
		p += sizeof(struct nhlt_pdm_ctrl_fir_cfg);
		val = fir_cfg_b[n]->fir_config;
		fir_length = FIELD_GET(FIR_CONFIG_FIR_LENGTH, val);
		fir_length_b = fir_length + 1; /* Need for parsing */
		fir_decimation = FIELD_GET(FIR_CONFIG_FIR_DECIMATION, val);
		p_mfirb = fir_decimation + 1;
		if (dmic->dai_config_params.dai_index == 1) {
			fir_shift = FIELD_GET(FIR_CONFIG_FIR_SHIFT, val);
			LOG_DBG("dmic_set_config_nhlt(): FIR_CONFIG_B = %08x", val);
			LOG_DBG("  fir_decimation=%d, fir_shift=%d, fir_length=%d",
				fir_decimation, fir_shift, fir_length);

			/* Use FIR_CONFIG_B as such */
			dai_dmic_write(dmic, base[n] + FIR_CONFIG_B, val);
			LOG_DBG("configure_registers(), FIR_CONFIG_B = %08x", val);

			val = fir_cfg_b[n]->fir_control;
			bf1 = FIELD_GET(FIR_CONTROL_START, val);
			bf2 = FIELD_GET(FIR_CONTROL_ARRAY_START_EN, val);
			bf3 = FIELD_GET(FIR_CONTROL_PERIODIC_START_EN, val);
			bf4 = FIELD_GET(FIR_CONTROL_DCCOMP, val);
			bf5 = FIELD_GET(FIR_CONTROL_MUTE, val);
			bf6 = FIELD_GET(FIR_CONTROL_STEREO, val);
			LOG_DBG("dmic_set_config_nhlt(): FIR_CONTROL_B = %08x", val);
			LOG_DBG("  start=%d, array_start_en=%d, periodic_start_en=%d",
				bf1, bf2, bf3);
			LOG_DBG("  dccomp=%d, mute=%d, stereo=%d", bf4, bf5, bf6);

			/* Clear START, set MUTE */
			fir_control = (val & ~FIR_CONTROL_START) | FIR_CONTROL_MUTE;
			dai_dmic_write(dmic, base[n] + FIR_CONTROL_B, fir_control);
			LOG_DBG("dmic_set_config_nhlt(): FIR_CONTROL_B = %08x", fir_control);

			/* Use DC_OFFSET and GAIN as such */
			val = fir_cfg_b[n]->dc_offset_left;
			dai_dmic_write(dmic, base[n] + DC_OFFSET_LEFT_B, val);
			LOG_DBG("dmic_set_config_nhlt(): DC_OFFSET_LEFT_B = %08x", val);

			val = fir_cfg_b[n]->dc_offset_right;
			dai_dmic_write(dmic, base[n] + DC_OFFSET_RIGHT_B, val);
			LOG_DBG("dmic_set_config_nhlt(): DC_OFFSET_RIGHT_B = %08x", val);

			val = fir_cfg_b[n]->out_gain_left;
			dai_dmic_write(dmic, base[n] + OUT_GAIN_LEFT_B, val);
			LOG_DBG("dmic_set_config_nhlt(): OUT_GAIN_LEFT_B = %08x", val);

			val = fir_cfg_b[n]->out_gain_right;
			dai_dmic_write(dmic, base[n] + OUT_GAIN_RIGHT_B, val);
			LOG_DBG("dmic_set_config_nhlt(): OUT_GAIN_RIGHT_B = %08x", val);
		}

		/* Set up FIR coefficients RAM */
		val = pdm_cfg[n]->reuse_fir_from_pdm;
		if (val == 0) {
			fir_a[n] = (struct nhlt_pdm_fir_coeffs *)p;
			p += sizeof(int32_t) * fir_length_a;
			fir_b[n] = (struct nhlt_pdm_fir_coeffs *)p;
			p += sizeof(int32_t) * fir_length_b;
		} else {
			val--;
			if (val >= n) {
				LOG_ERR("dmic_set_config_nhlt(): Illegal FIR reuse 0x%x", val);
				return -EINVAL;
			}

			if (!fir_a[val]) {
				LOG_ERR("dmic_set_config_nhlt(): PDM%d FIR reuse from %d fail",
					n, val);
				return -EINVAL;
			}

			fir_a[n] = fir_a[val];
			fir_b[n] = fir_b[val];
		}

		if (dmic->dai_config_params.dai_index == 0) {
			LOG_INF(
			  "dmic_set_config_nhlt(): clkdiv = %d, mcic = %d, mfir_a = %d, len = %d",
			   p_clkdiv, p_mcic, p_mfira, fir_length_a);
			for (i = 0; i < fir_length_a; i++)
				dai_dmic_write(dmic,
					       coef_base_a[n] + (i << 2), fir_a[n]->fir_coeffs[i]);
		} else {
			LOG_INF(
			  "dmic_set_config_nhlt(): clkdiv = %d, mcic = %d, mfir_b = %d, len = %d",
			  p_clkdiv, p_mcic, p_mfirb, fir_length_b);
			for (i = 0; i < fir_length_b; i++)
				dai_dmic_write(dmic,
					       coef_base_b[n] + (i << 2), fir_b[n]->fir_coeffs[i]);
		}
	}

	if (dmic->dai_config_params.dai_index == 0)
		ret = dai_nhlt_dmic_dai_params_get(dmic, out_control, pdm_cfg, fir_cfg_a);
	else
		ret = dai_nhlt_dmic_dai_params_get(dmic, out_control, pdm_cfg, fir_cfg_b);

	if (ret)
		return ret;

	if (dmic->dai_config_params.dai_index == 0)
		rate_div = p_clkdiv * p_mcic * p_mfira;
	else
		rate_div = p_clkdiv * p_mcic * p_mfirb;

	if (!rate_div) {
		LOG_ERR("dmic_set_config_nhlt(): zero clock divide or decimation factor");
		return -EINVAL;
	}

	dmic->dai_config_params.rate = adsp_clock_souce_frequency(dmic_cfg->clock_source) /
		rate_div;
	LOG_INF("dmic_set_config_nhlt(): rate = %d, channels = %d, format = %d",
		 dmic->dai_config_params.rate, dmic->dai_config_params.channels,
		 dmic->dai_config_params.format);

	LOG_INF("dmic_set_config_nhlt(): io_clk %u, rate_div %d",
		adsp_clock_souce_frequency(dmic_cfg->clock_source), rate_div);

	LOG_INF("dmic_set_config_nhlt(): enable0 %u, enable1 %u",
		dmic->enable[0], dmic->enable[1]);
	return 0;
}

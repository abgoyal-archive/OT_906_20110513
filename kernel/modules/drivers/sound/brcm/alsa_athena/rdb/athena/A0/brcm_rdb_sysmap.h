/***************************************************************************
 *     Copyright (c) 1999-2009, Broadcom Corporation
 *
 *
 *  Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *
 *  As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy
 * and distribute the resulting executable under terms of your choice,
 * provided that you also meet, for each linked independent module, the terms
 * and conditions of the license of that module.  An independent module is a
 * module which is not derived from this software.  The special exception
 * does not apply to any modifications of the software.
 *
 *
 *  Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 *
 * $brcm_Workfile: $
 * $brcm_Revision: $
 * $brcm_Date: $
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Thu Mar 19 18:04:10 2009
 *                 MD5 Checksum         93c8a72ac3ac1b3437a19d5e411b5fa8
 *
 * Compiled with:  RDB Utility          1.0
 *                 RDB Parser           3.0
 *                 rdbfirmware_split.pl 3.0
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Spec Versions:  AUXADC               1.0
 *                 AUXMIC               1.0
 *                 BBRX                 1
 *                 CAMINTF              1.0
 *                 CIPHER               1.0
 *                 CLK                  1.0
 *                 CPC_HARQ             1
 *                 CRC                  1.0
 *                 CRYPTO               1.0
 *                 CSTF                 1.0
 *                 CSTPIU               1.0
 *                 DATAPACKER           1.0
 *                 DES                  1.0
 *                 DISPC                1.1
 *                 DMAC                 1.0
 *                 DPE                  1.0
 *                 DSI                  1.0
 *                 DSP_3WIRE_INTF       1.0
 *                 DSP_AUDIO            3.0
 *                 DSP_CIPHER           1.0
 *                 DSP_CPMR             1
 *                 DSP_DIGRF            1.0
 *                 DSP_EQUALIZER        1.0
 *                 DSP_EVENT_CTRL       1.0
 *                 DSP_INTC             1
 *                 DSP_RF_GPO           1.0
 *                 DSP_RX_CONTROL       1.0
 *                 DSP_SYS_TIMER        1.0
 *                 DSP_TL3R             1
 *                 DSP_TPIF             1
 *                 DSP_TX_CONTROL       1.0
 *                 ECT                  1.0
 *                 EMI                  1
 *                 FSUSBHOST            1.0
 *                 GEA                  1.1
 *                 GPIO                 1.0
 *                 GPIO16               1.0
 *                 GPTIMERS             2.3
 *                 HARQ                 1
 *                 HSDPA_Turbo_Decoder_Module1
 *                 HSOTG                1.0
 *                 HSOTG_CTRL           1.0
 *                 HTM                  1.0
 *                 HUCM                 1
 *                 I2S                  1.0
 *                 INTC                 1.0
 *                 KEYPAD               1.0
 *                 LCDC                 1.0
 *                 MHARB                1.0
 *                 MLARB                1.0
 *                 MP                   1.0
 *                 MPHI                 1.0
 *                 MSPRO                1.0
 *                 NVSRAM               1.1
 *                 OTP                  1.0
 *                 PKA                  1.0
 *                 PTIMER               1.0
 *                 PWM_TOP              1.0
 *                 RNG                  1.0
 *                 SCLKCAL              1.0
 *                 SDHOST_SS            3.4
 *                 SIMI                 1.0
 *                 SLPTIMER             1.0
 *                 SOFTRST              1.0
 *                 SPI                  1.0
 *                 SYSCFG               1.0
 *                 UARTA                1.0
 *                 UARTB                1.0
 *                 UARTC                1.0
 *                 VEC                  1.1
 *                 WCDMA_BOOT           2.0
 *                 WCDMA_GPIO           1
 *                 WTI                  1.0
 *                 afc_top              1
 *                 bb_tx_filters_and_2091_if1
 *                 combiner             1
 *                 cpp_cluster          1
 *                 decoderh264          1.1
 *                 deinterleaver        1
 *                 edch_fng_core_top    1
 *                 encoderh264          1.1
 *                 fng_core_top         1
 *                 hs_scch              1
 *                 i2c_mm_hs            1.0
 *                 layer_1_int_cont     1
 *                 layer_2_int_cont     1
 *                 master_timer         1
 *                 modem_cfg_and_core_clks1
 *                 modem_clocks_part_1  1
 *                 modem_clocks_part_2  1
 *                 mpdxx_apb            1
 *                 pdm_top              1
 *                 prism                1
 *                 psync_for_10ppm      1
 *                 rake_fng_top         1
 *                 rake_tiny_fng_top    1
 *                 rf_interface_block1_top1
 *                 rf_interface_block2_top1
 *                 rf_interface_block3_top1
 *                 rf_interface_block4_top1
 *                 rf_interface_block5_top1
 *                 rfic_mst             1
 *                 rfic_reg_file        1
 *                 rfic_scheduler       1
 *                 rxbitlevel           1
 *                 sc_xy_state_derive   1
 *                 ssync_apb            1
 *                 sttd_apb             1
 *                 tiny_fng_core_top    1
 *                 tx                   1
 *                 wcdma_gp_timers      1
 *                 wcdma_uart           1
 *
 * RDB Files:  /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/athena_chip.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/auxmic_det.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/i2c_mm_hs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/sdhost_ss.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/GEA.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/lcdc_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/pka_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rng.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/uarta.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/uart_defs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/uartb.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/uart_defs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/uartc.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/uart_defs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/emi.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/softrst.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/ect_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/sim_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dmac.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dmac_defs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/crc.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/crypto_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/des.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/mipi_dsi_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/athena_MatrixL_reg.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/athena_MatrixH_reg.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/mp.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/encoderh264.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/decoder_h264.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/athena_intrpt_cntrl.rd
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/gptimers.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/ptimer.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/slptimer.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/wtimer.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/mspro.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/sclkcal.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/gpio16.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/gpio_defs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/gpio.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/gpio_defs.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/keypad.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/auxadc.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/secure_otp.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/athena_syscfg.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/da.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/athena_clk_pwr_mngr.rd
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/nv_sram_if.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/spi.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/datapacker.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/cipher.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/hucm.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/displayc.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/vec.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/cstf.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/cstpiu.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/hsotg.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/hsotg_ctrl.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/fsusbhost.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/htm.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/wcdma_boot.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/pwm_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dpe.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/mphi.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/camintf_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/afc_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/bbrx.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/combiner.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/cpc_harq.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/cpp_cluster.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/deinterleaver.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/edch_fng_core_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/fng_core_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/harq.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/hs_scch.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/hsdpa_turbo_decoder_mo
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/layer_1_int_cont.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/layer_2_int_cont.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/master_timer.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/modem_cfg_and_core_clk
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/modem_clocks_part_1.rd
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/modem_clocks_part_2.rd
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/mpdxx_apb.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/pdm_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/prism.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/psync_for_10ppm.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rake_fng_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rake_tiny_fng_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rf_interface_block1_to
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rf_interface_block2_to
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rf_interface_block3_to
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rf_interface_block4_to
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rf_interface_block5_to
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rfic_mst.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rfic_reg_file.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rxbitlevel.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/sc_xy_state_derive.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/ssync_apb.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/sttd_apb.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/tiny_fng_core_top.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/tx.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/wcdma_gpio.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/wcdma_gp_timers.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/wcdma_uart.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/rfic_scheduler.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/bb_tx_filters_and_2091
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_cpmr.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_tl3r.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_intc.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_tpif.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_cipher.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_equalizer.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_evnt_ctrl.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_rf_gpo.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_3_wire_intf.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_sys_timer.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_2g_digrf.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_rx_control.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_tx_control.rdb
 *             /projects/athena_ext/revA0/users/CRDB/athena_revA0/design_database/athena_chip/rdb/dsp_audio.rdb
 *
 * Revision History:
 *
 * $brcm_Log: $
 *
 ***************************************************************************/

#ifndef __ATHENA_SYSMAP_H__
#define __ATHENA_SYSMAP_H__

#ifdef UNDER_LINUX
#include <mach/io.h>
#include <mach/hardware.h>
#else
#define IO_ADDRESS(x) x
#endif
/****  DSP_CPMR  ****/
#define DSP_CPMR_BASE_ADDR      IO_ADDRESS(0x00000000)      /* DSP_CPMR core */

/****  DSP_TL3R  ****/
#define DSP_TL3R_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_TL3R core */

/****  DSP_INTC  ****/
#define DSP_INTC_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_INTC core */

/****  DSP_TPIF  ****/
#define DSP_TPIF_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_TPIF core */

/****  DSP_CIPHER  ****/
#define DSP_CIPHER_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_CIPHER core */

/****  DSP_EQUALIZER  ****/
#define DSP_EQUALIZER_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_EQUALIZER core */

/****  DSP_EVENT_CTRL  ****/
#define DSP_EVENT_CTRL_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_EVENT_CTRL core */

/****  DSP_RF_GPO  ****/
#define DSP_RF_GPO_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_RF_GPO core */

/****  DSP_3WIRE_INTF  ****/
#define DSP_3WIRE_INTF_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_3WIRE_INTF core */

/****  DSP_SYS_TIMER  ****/
#define DSP_SYS_TIMER_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_SYS_TIMER core */

/****  DSP_DIGRF  ****/
#define DSP_DIGRF_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_DIGRF core */

/****  DSP_RX_CONTROL  ****/
#define DSP_RX_CONTROL_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_RX_CONTROL core */

/****  DSP_TX_CONTROL  ****/
#define DSP_TX_CONTROL_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_TX_CONTROL core */

/****  DSP_AUDIO  ****/
#define DSP_AUDIO_BASE_ADDR      IO_ADDRESS(0x00800000)      /* DSP_AUDIO core */

/****  BMARBH  ****/
#define BMARBH_BASE_ADDR      IO_ADDRESS(0x08010000)      /* BMARBH core */

/****  BMARBL  ****/
#define BMARBL_BASE_ADDR      IO_ADDRESS(0x08018000)      /* BMARBL core */

/****  DMAC  ****/
#define DMAC_BASE_ADDR      IO_ADDRESS(0x08020000)      /* DMAC core */

/****  LCDC  ****/
#define LCDC_BASE_ADDR      IO_ADDRESS(0x08030000)      /* LCDC core */

/****  DPE  ****/
#define DPE_BASE_ADDR      IO_ADDRESS(0x08040000)      /* DPE core */

/****  MPHI  ****/
#define MPHI_BASE_ADDR      IO_ADDRESS(0x08050000)      /* MPHI core */

/****  MP  ****/
#define MP_BASE_ADDR      IO_ADDRESS(0x08070000)      /* MP core */

/****  NVSRAM  ****/
#define NVSRAM_BASE_ADDR      IO_ADDRESS(0x08090000)      /* NVSRAM core */

/****  DP  ****/
#define DP_BASE_ADDR      IO_ADDRESS(0x080A0000)      /* DP core */

/****  DES  ****/
#define DES_BASE_ADDR      IO_ADDRESS(0x080B0000)      /* DES core */

/****  SDIO1  ****/
#define SDIO1_BASE_ADDR      IO_ADDRESS(0x08110000)      /* SDIO1 core */

/****  SDIO2  ****/
#define SDIO2_BASE_ADDR      IO_ADDRESS(0x08120000)      /* SDIO2 core */

/****  CLK  ****/
#define CLK_BASE_ADDR      IO_ADDRESS(0x08140000)      /* CLK core */

/****  CIPHER  ****/
#define CIPHER_BASE_ADDR      IO_ADDRESS(0x08150000)      /* CIPHER core */

/****  WCDMAcm  ****/
#define WCDMAcm_BASE_ADDR      IO_ADDRESS(0x08168000)      /* WCDMAcm core */

/****  WCDMAmpd0_15  ****/
#define WCDMAmpd0_15_BASE_ADDR      IO_ADDRESS(0x08168400)      /* WCDMAmpd0_15 core */

/****  WCDMAmpd0_30  ****/
#define WCDMAmpd0_30_BASE_ADDR      IO_ADDRESS(0x08168500)      /* WCDMAmpd0_30 core */

/****  WCDMAssync  ****/
#define WCDMAssync_BASE_ADDR      IO_ADDRESS(0x08168C00)      /* WCDMAssync core */

/****  WCDMApsync  ****/
#define WCDMApsync_BASE_ADDR      IO_ADDRESS(0x08169000)      /* WCDMApsync core */

/****  WCDMATx  ****/
#define WCDMATx_BASE_ADDR      IO_ADDRESS(0x08169100)      /* WCDMATx core */

/****  WCDMAfngmux  ****/
#define WCDMAfngmux_BASE_ADDR      IO_ADDRESS(0x08169200)      /* WCDMAfngmux core */

/****  WCDMARxBit  ****/
#define WCDMARxBit_BASE_ADDR      IO_ADDRESS(0x08169280)      /* WCDMARxBit core */

/****  WCDMARxDeint  ****/
#define WCDMARxDeint_BASE_ADDR      IO_ADDRESS(0x08169300)      /* WCDMARxDeint core */

/****  WCDMAagc  ****/
#define WCDMAagc_BASE_ADDR      IO_ADDRESS(0x08169380)      /* WCDMAagc core */

/****  WCDMAtwif  ****/
#define WCDMAtwif_BASE_ADDR      IO_ADDRESS(0x08169400)      /* WCDMAtwif core */

/****  WCDMASttd  ****/
#define WCDMASttd_BASE_ADDR      IO_ADDRESS(0x08169480)      /* WCDMASttd core */

/****  WCDMAafc  ****/
#define WCDMAafc_BASE_ADDR      IO_ADDRESS(0x08169500)      /* WCDMAafc core */

/****  WCDMAtnyrake_1  ****/
#define WCDMAtnyrake_1_BASE_ADDR      IO_ADDRESS(0x08169600)      /* WCDMAtnyrake_1 core */

/****  WCDMAtnyrake_2  ****/
#define WCDMAtnyrake_2_BASE_ADDR      IO_ADDRESS(0x08169700)      /* WCDMAtnyrake_2 core */

/****  WCDMAtnyrake_3  ****/
#define WCDMAtnyrake_3_BASE_ADDR      IO_ADDRESS(0x08169800)      /* WCDMAtnyrake_3 core */

/****  WCDMAtnyctrl  ****/
#define WCDMAtnyctrl_BASE_ADDR      IO_ADDRESS(0x08169900)      /* WCDMAtnyctrl core */

/****  WCDMAScramXY  ****/
#define WCDMAScramXY_BASE_ADDR      IO_ADDRESS(0x08169980)      /* WCDMAScramXY core */

/****  WCDMAbbrx  ****/
#define WCDMAbbrx_BASE_ADDR      IO_ADDRESS(0x08169A00)      /* WCDMAbbrx core */

/****  WCDMAPrism  ****/
#define WCDMAPrism_BASE_ADDR      IO_ADDRESS(0x08169B00)      /* WCDMAPrism core */

/****  WCDMAcpp  ****/
#define WCDMAcpp_BASE_ADDR      IO_ADDRESS(0x08169C00)      /* WCDMAcpp core */

/****  WCDMAcpcharq  ****/
#define WCDMAcpcharq_BASE_ADDR      IO_ADDRESS(0x08169E00)      /* WCDMAcpcharq core */

/****  WCDMAmc  ****/
#define WCDMAmc_BASE_ADDR      IO_ADDRESS(0x08169F00)      /* WCDMAmc core */

/****  WCDMASpinclk1  ****/
#define WCDMASpinclk1_BASE_ADDR      IO_ADDRESS(0x08169F80)      /* WCDMASpinclk1 core */

/****  WCDMASpinclk2  ****/
#define WCDMASpinclk2_BASE_ADDR      IO_ADDRESS(0x08169FC0)      /* WCDMASpinclk2 core */

/****  WCDMAhtdm  ****/
#define WCDMAhtdm_BASE_ADDR      IO_ADDRESS(0x0816A100)      /* WCDMAhtdm core */

/****  WCDMAscch  ****/
#define WCDMAscch_BASE_ADDR      IO_ADDRESS(0x0816A200)      /* WCDMAscch core */

/****  WCDMAharq  ****/
#define WCDMAharq_BASE_ADDR      IO_ADDRESS(0x0816A300)      /* WCDMAharq core */

/****  WCDMAtnyrake_4  ****/
#define WCDMAtnyrake_4_BASE_ADDR      IO_ADDRESS(0x0816A400)      /* WCDMAtnyrake_4 core */

/****  WCDMAtnyrake_5  ****/
#define WCDMAtnyrake_5_BASE_ADDR      IO_ADDRESS(0x0816A500)      /* WCDMAtnyrake_5 core */

/****  WCDMAtnyrake_6  ****/
#define WCDMAtnyrake_6_BASE_ADDR      IO_ADDRESS(0x0816A600)      /* WCDMAtnyrake_6 core */

/****  WCDMArfic  ****/
#define WCDMArfic_BASE_ADDR      IO_ADDRESS(0x0816A780)      /* WCDMArfic core */

/****  WCDMArficBuff  ****/
#define WCDMArficBuff_BASE_ADDR      IO_ADDRESS(0x0816A800)      /* WCDMArficBuff core */

/****  WCDMAmpd1_15  ****/
#define WCDMAmpd1_15_BASE_ADDR      IO_ADDRESS(0x0816AA00)      /* WCDMAmpd1_15 core */

/****  WCDMAmpd1_30  ****/
#define WCDMAmpd1_30_BASE_ADDR      IO_ADDRESS(0x0816AB00)      /* WCDMAmpd1_30 core */

/****  WCDMASched  ****/
#define WCDMASched_BASE_ADDR      IO_ADDRESS(0x0816AC00)      /* WCDMASched core */

/****  WCDMABBRfTx  ****/
#define WCDMABBRfTx_BASE_ADDR      IO_ADDRESS(0x0816AE00)      /* WCDMABBRfTx core */

/****  WCDMArxadc  ****/
#define WCDMArxadc_BASE_ADDR      IO_ADDRESS(0x0816B000)      /* WCDMArxadc core */

/****  WCDMAgp  ****/
#define WCDMAgp_BASE_ADDR      IO_ADDRESS(0x0816B080)      /* WCDMAgp core */

/****  WCDMApdm  ****/
#define WCDMApdm_BASE_ADDR      IO_ADDRESS(0x0816B100)      /* WCDMApdm core */

/****  WCDMATxDac  ****/
#define WCDMATxDac_BASE_ADDR      IO_ADDRESS(0x0816B180)      /* WCDMATxDac core */

/****  WCDMAGpTimer  ****/
#define WCDMAGpTimer_BASE_ADDR      IO_ADDRESS(0x0816C000)      /* WCDMAGpTimer core */

/****  WCDMAl2int  ****/
#define WCDMAl2int_BASE_ADDR      IO_ADDRESS(0x0816C180)      /* WCDMAl2int core */

/****  WCDMAl1irq  ****/
#define WCDMAl1irq_BASE_ADDR      IO_ADDRESS(0x0816C280)      /* WCDMAl1irq core */

/****  WCDMAl1fiq  ****/
#define WCDMAl1fiq_BASE_ADDR      IO_ADDRESS(0x0816C2C0)      /* WCDMAl1fiq core */

/****  WCDMAAsicClk  ****/
#define WCDMAAsicClk_BASE_ADDR      IO_ADDRESS(0x0816C300)      /* WCDMAAsicClk core */

/****  WCDMAgpio  ****/
#define WCDMAgpio_BASE_ADDR      IO_ADDRESS(0x0816C500)      /* WCDMAgpio core */

/****  WCDMAuart1  ****/
#define WCDMAuart1_BASE_ADDR      IO_ADDRESS(0x0816C600)      /* WCDMAuart1 core */

/****  WCDMArake_1  ****/
#define WCDMArake_1_BASE_ADDR      IO_ADDRESS(0x0816D000)      /* WCDMArake_1 core */

/****  WCDMArake_2  ****/
#define WCDMArake_2_BASE_ADDR      IO_ADDRESS(0x0816D200)      /* WCDMArake_2 core */

/****  WCDMArake_3  ****/
#define WCDMArake_3_BASE_ADDR      IO_ADDRESS(0x0816D400)      /* WCDMArake_3 core */

/****  WCDMArake_4  ****/
#define WCDMArake_4_BASE_ADDR      IO_ADDRESS(0x0816D600)      /* WCDMArake_4 core */

/****  WCDMArake_5  ****/
#define WCDMArake_5_BASE_ADDR      IO_ADDRESS(0x0816D800)      /* WCDMArake_5 core */

/****  WCDMArake_6  ****/
#define WCDMArake_6_BASE_ADDR      IO_ADDRESS(0x0816DA00)      /* WCDMArake_6 core */

/****  WCDMArake_7  ****/
#define WCDMArake_7_BASE_ADDR      IO_ADDRESS(0x0816DC00)      /* WCDMArake_7 core */

/****  WCDMArake_8  ****/
#define WCDMArake_8_BASE_ADDR      IO_ADDRESS(0x0816DE00)      /* WCDMArake_8 core */

/****  WCDMAl2int_Async  ****/
#define WCDMAl2int_Async_BASE_ADDR      IO_ADDRESS(0x0816F000)      /* WCDMAl2int_Async core */

/****  CRC  ****/
#define CRC_BASE_ADDR      IO_ADDRESS(0x08180000)      /* CRC core */

/****  VIDEO_DEC  ****/
#define VIDEO_DEC_BASE_ADDR      IO_ADDRESS(0x081A0000)      /* VIDEO_DEC core */

/****  VIDEO_ENC  ****/
#define VIDEO_ENC_BASE_ADDR      IO_ADDRESS(0x081A8000)      /* VIDEO_ENC core */

/****  ECT0_CTI0  ****/
#define ECT0_CTI0_BASE_ADDR      IO_ADDRESS(0x081C0000)      /* ECT0_CTI0 core */

/****  ECT0_CTI1  ****/
#define ECT0_CTI1_BASE_ADDR      IO_ADDRESS(0x081C1000)      /* ECT0_CTI1 core */

/****  ECT1_CTI0  ****/
#define ECT1_CTI0_BASE_ADDR      IO_ADDRESS(0x081C2000)      /* ECT1_CTI0 core */

/****  ECT1_CTI1  ****/
#define ECT1_CTI1_BASE_ADDR      IO_ADDRESS(0x081C3000)      /* ECT1_CTI1 core */

/****  HSOTG  ****/
#define HSOTG_BASE_ADDR      IO_ADDRESS(0x08200000)      /* HSOTG core */

/****  HSOTG_CTRL  ****/
#define HSOTG_CTRL_BASE_ADDR      IO_ADDRESS(0x08280000)      /* HSOTG_CTRL core */

/****  FSHOST  ****/
#define FSHOST_BASE_ADDR      IO_ADDRESS(0x08300000)      /* FSHOST core */

/****  FSHOST_CTRL  ****/
#define FSHOST_CTRL_BASE_ADDR      IO_ADDRESS(0x08380000)      /* FSHOST_CTRL core */

/****  SDIO3  ****/
#define SDIO3_BASE_ADDR      IO_ADDRESS(0x08400000)      /* SDIO3 core */

/****  EMI  ****/
#define EMI_BASE_ADDR      IO_ADDRESS(0x08420000)      /* EMI core */

/****  CAM  ****/
#define CAM_BASE_ADDR      IO_ADDRESS(0x08440000)      /* CAM core */

/****  HUCM  ****/
#define HUCM_BASE_ADDR      IO_ADDRESS(0x08480000)      /* HUCM core */

/****  DISPC  ****/
#define DISPC_BASE_ADDR      IO_ADDRESS(0x08490000)      /* DISPC core */

/****  DSI  ****/
#define DSI_BASE_ADDR      IO_ADDRESS(0x084A0000)      /* DSI core */
#define L210_BASE_ADDR            IO_ADDRESS(0x084A1000) /* brcm_rdb_l210.h */
/****  SLPTMR  ****/
#define SLPTMR_BASE_ADDR      IO_ADDRESS(0x08800000)      /* SLPTMR core */

/****  SLPTMR2  ****/
#define SLPTMR2_BASE_ADDR      IO_ADDRESS(0x08800100)      /* SLPTMR2 core */

/****  INTC  ****/
#define INTC_BASE_ADDR      IO_ADDRESS(0x08810000)      /* INTC core */

/****  UARTA  ****/
#define UARTA_BASE_ADDR      IO_ADDRESS(0x08820000)      /* UARTA core */

/****  UARTB  ****/
#define UARTB_BASE_ADDR      IO_ADDRESS(0x08821000)      /* UARTB core */

/****  UARTC  ****/
#define UARTC_BASE_ADDR      IO_ADDRESS(0x08822000)      /* UARTC core */

/****  PTIMER  ****/
#define PTIMER_BASE_ADDR      IO_ADDRESS(0x08830000)      /* PTIMER core */

/****  AUXADC  ****/
#define AUXADC_BASE_ADDR      IO_ADDRESS(0x08830020)      /* AUXADC core */

/****  GPTIMER  ****/
#define GPTIMER_BASE_ADDR      IO_ADDRESS(0x08830100)      /* GPTIMER core */

/****  SIMI  ****/
#define SIMI_BASE_ADDR      IO_ADDRESS(0x08860000)      /* SIMI core */

/****  SCLKCAL  ****/
#define SCLKCAL_BASE_ADDR      IO_ADDRESS(0x08870010)      /* SCLKCAL core */

/****  SYSCFG  ****/
#define SYSCFG_BASE_ADDR      IO_ADDRESS(0x08880000)      /* SYSCFG core */

/****  GEA  ****/
#define GEA_BASE_ADDR      IO_ADDRESS(0x08890000)      /* GEA core */

/****  WATCHDOG  ****/
#define WATCHDOG_BASE_ADDR      IO_ADDRESS(0x088A0000)      /* WATCHDOG core */

/****  BSC1  ****/
#define BSC1_BASE_ADDR      IO_ADDRESS(0x088A0000)      /* BSC1 core */

/****  WATCHDOG2  ****/
#define WATCHDOG2_BASE_ADDR      IO_ADDRESS(0x088A0010)      /* WATCHDOG2 core */

/****  BSC2  ****/
#define BSC2_BASE_ADDR      IO_ADDRESS(0x088B0000)      /* BSC2 core */

/****  I2S  ****/
#define I2S_BASE_ADDR      IO_ADDRESS(0x088C0000)      /* I2S core */

/****  GPIO  ****/
#define GPIO_BASE_ADDR      IO_ADDRESS(0x088CE000)      /* GPIO core */

/****  KEYPAD  ****/
#define KEYPAD_BASE_ADDR      IO_ADDRESS(0x088CE000)      /* KEYPAD core */

/****  GPIO16  ****/
#define GPIO16_BASE_ADDR      IO_ADDRESS(0x088CE000)      /* GPIO16 core */

/****  SPI  ****/
#define SPI_BASE_ADDR      IO_ADDRESS(0x088D0000)      /* SPI core */

/****  SOFTRST  ****/
#define SOFTRST_BASE_ADDR      IO_ADDRESS(0x088E0000)      /* SOFTRST core */

/****  MSPRO  ****/
#define MSPRO_BASE_ADDR      IO_ADDRESS(0x08900000)      /* MSPRO core */

/****  AUXMIC  ****/
#define AUXMIC_BASE_ADDR      IO_ADDRESS(0x08911000)      /* AUXMIC core */

/****  SPI2  ****/
#define SPI2_BASE_ADDR      IO_ADDRESS(0x08920000)      /* SPI2 core */

/****  SIMI2  ****/
#define SIMI2_BASE_ADDR      IO_ADDRESS(0x08930000)      /* SIMI2 core */

/****  PWM  ****/
#define PWM_BASE_ADDR      IO_ADDRESS(0x08940000)      /* PWM core */

/****  VEC  ****/
#define VEC_BASE_ADDR      IO_ADDRESS(0x08B00000)      /* VEC core */

/****  HTM0  ****/
#define HTM0_BASE_ADDR      IO_ADDRESS(0x08C00000)      /* HTM0 core */

/****  HTM1  ****/
#define HTM1_BASE_ADDR      IO_ADDRESS(0x08C01000)      /* HTM1 core */

/****  TFUNNEL  ****/
#define TFUNNEL_BASE_ADDR      IO_ADDRESS(0x08C02000)      /* TFUNNEL core */

/****  TPIU  ****/
#define TPIU_BASE_ADDR      IO_ADDRESS(0x08C03000)      /* TPIU core */

/****  CRYPTO  ****/
#define CRYPTO_BASE_ADDR      IO_ADDRESS(0x0C080000)      /* CRYPTO core */

/****  PKA  ****/
#define PKA_BASE_ADDR      IO_ADDRESS(0x0C0C8000)      /* PKA core */

/****  OTP  ****/
#define OTP_BASE_ADDR      IO_ADDRESS(0x0C0C9000)      /* OTP core */

/****  RNG  ****/
#define RNG_BASE_ADDR      IO_ADDRESS(0x0C0CA000)      /* RNG core */

/****  WCDMA_BOOT  ****/
#define WCDMA_BOOT_BASE_ADDR      IO_ADDRESS(0xFFFF0000)      /* WCDMA_BOOT core */

/****  AHB_TL3R  ****/
//#define AHB_TL3R_BASE_ADDR      IO_ADDRESS(0x30400000)      /* AHB_TL3R core */
#define AHB_TL3R_BASE_ADDR         0x30400000      /* AHB_TL3R core */  /* This is physical address */

/****  AHB_INTC  ****/
#define AHB_INTC_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_INTC core */

/****  AHB_TPIF  ****/
#define AHB_TPIF_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_TPIF core */

/****  AHB_CIPHER  ****/
#define AHB_CIPHER_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_CIPHER core */

/****  AHB_EQUALIZER  ****/
#define AHB_EQUALIZER_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_EQUALIZER core */

/****  AHB_EVENT_CTRL  ****/
#define AHB_EVENT_CTRL_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_EVENT_CTRL core */

/****  AHB_RF_GPO  ****/
#define AHB_RF_GPO_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_RF_GPO core */

/****  AHB_3WIRE_INTF  ****/
#define AHB_3WIRE_INTF_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_3WIRE_INTF core */

/****  AHB_SYS_TIMER  ****/
#define AHB_SYS_TIMER_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_SYS_TIMER core */

/****  AHB_DIGRF  ****/
#define AHB_DIGRF_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_DIGRF core */

/****  AHB_RX_CONTROL  ****/
#define AHB_RX_CONTROL_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_RX_CONTROL core */

/****  AHB_TX_CONTROL  ****/
#define AHB_TX_CONTROL_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_TX_CONTROL core */

/****  AHB_AUDIO  ****/
//#define AHB_AUDIO_BASE_ADDR      IO_ADDRESS(0x30800000)      /* AHB_AUDIO core */ /* This is physical address */

#define AHB_AUDIO_BASE_ADDR      0x30800000      /* AHB_AUDIO core */ /* This is physical address */

#endif  /* __ATHENA_SYSMAP_H__  */

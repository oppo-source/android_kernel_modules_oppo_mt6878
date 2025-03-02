/******************************************************************************
 *
 * This file is provided under a dual license.  When you use or
 * distribute this software, you may choose to be licensed under
 * version 2 of the GNU General Public License ("GPLv2 License")
 * or BSD License.
 *
 * GPLv2 License
 *
 * Copyright(C) 2019 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 *
 * BSD LICENSE
 *
 * Copyright(C) 2019 MediaTek Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/*! \file   hal_wfsys_reset_mt7925.h
*    \brief  WFSYS reset HAL API for MT7925
*
*    This file contains all routines which are exported
     from MediaTek 802.11 Wireless LAN driver stack to GLUE Layer.
*/

#ifndef _HAL_WFSYS_RESET_MT7925_H
#define _HAL_WFSYS_RESET_MT7925_H

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/
#if defined(_HIF_USB)
/* Accessing conn_sys CR by UHW needs remap CR address to conn_infra view */
#define WF_TOP_CFG_ON_ROMCODE_INDEX_REMAP_ADDR 0x184C1604
#endif

#if defined(_HIF_PCIE)
#define PCIE_MAC_IREG_ISTATUS_HOST_WDT_INT_MASK    0x00010000
#define CONN_INFRA_BUS_CR_PCIE2AP_REMAP_WF_0_54_DEFAULT 0x18451844
#define CONN_INFRA_BUS_CR_PCIE2AP_REMAP_WF_0_54_ADDR 0x7c021008
#define CBTOP_RGU_BASE 0x70028600
#define R_PCIE2AP_PUBLIC_REMAPPING_4_BUS_ADDR 0x40000

#define WF_TOP_CFG_ON_ROMCODE_INDEX_REMAP_ADDR 0x81021604
#endif

/*******************************************************************************
*                   F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

u_int8_t mt7925HalCbInfraRguWfRst(struct ADAPTER *prAdapter,
				u_int8_t fgAssertRst);

u_int8_t mt7925HalPollWfsysSwInitDone(struct ADAPTER *prAdapter);

#if defined(_HIF_PCIE)
void mt7925GetSemaphore(struct ADAPTER *prAdapter);
void mt7925GetSemaReport(struct ADAPTER *prAdapter);
#endif /* defined(_HIF_PCIE) */

#if defined(_HIF_USB)

u_int8_t mt7925HalUsbEpctlRstOpt(struct ADAPTER *prAdapter,
				 u_int8_t fgIsRstScopeIncludeToggleBit);

#endif /* defined(_HIF_USB) */

#if defined(_HIF_SDIO)

#endif

#endif /* _HAL_WFSYS_RESET_MT7925_H */

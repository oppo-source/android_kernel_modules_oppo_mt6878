/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

/*! \file   "qosmap.h"
 *    \brief  This file including the qosmap related function.
 *
 *    This file provided the macros and functions library support for the
 *    protocol layer qosmap related function.
 *
 */

#ifndef _QOSMAP_H
#define _QOSMAP_H

/*******************************************************************************
 *                         C O M P I L E R   F L A G S
 *******************************************************************************
 */

/*******************************************************************************
 *                    E X T E R N A L   R E F E R E N C E S
 *******************************************************************************
 */

/*******************************************************************************
 *                              C O N S T A N T S
 *******************************************************************************
 */

/*******************************************************************************
 *                         D A T A   T Y P E S
 *******************************************************************************
 */


/*******************************************************************************
 *                            P U B L I C   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                           P R I V A T E   D A T A
 *******************************************************************************
 */

/*******************************************************************************
 *                                 M A C R O S
 *******************************************************************************
 */

/*******************************************************************************
 *                  F U N C T I O N   D E C L A R A T I O N S
 *******************************************************************************
 */
void handleQosMapConf(struct ADAPTER *prAdapter,
		      struct SW_RFB *prSwRfb);

int qosHandleQosMapConfigure(struct ADAPTER *prAdapter,
			     struct SW_RFB *prSwRfb);

void qosMapSetInit(struct STA_RECORD *prStaRec);

void qosParseQosMapSet(struct ADAPTER *prAdapter,
	struct STA_RECORD *prStaRec, const uint8_t *qosMapSet);

/*******************************************************************************
 *                              F U N C T I O N S
 *******************************************************************************
 */

#endif /* _QOSMAP_H */

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 Oplus. All rights reserved.
 */
#ifndef __OPLUS_KD_IMGSENSOR_H
#define __OPLUS_KD_IMGSENSOR_H

#define S5KJN1_SENSOR_ID                        0x38E1
#define SENSOR_ID_OFFSET_LUNA                       0x1000
#define IMX800LUNA_SENSOR_ID                        (0x0800 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX800LUNA_MIPI_RAW  "imx800luna_mipi_raw"
#define IMX709LUNA_SENSOR_ID                        (0x0709 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX709LUNA_MIPI_RAW  "imx709luna_mipi_raw"
#define S5KJN1LUNA_SENSOR_ID                        (0x38E1 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_S5KJN1LUNA_MIPI_RAW  "s5kjn1luna_mipi_raw"
#define IMX766LUNA_SENSOR_ID                        (0x0766 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX766LUNA_MIPI_RAW  "imx766luna_mipi_raw"
#define IMX890TELELUNA_SENSOR_ID                    (0x0890 + SENSOR_ID_OFFSET_LUNA + 0x1)
#define SENSOR_DRVNAME_IMX890TELELUNA_MIPI_RAW  "imx890teleluna_mipi_raw"
#define IMX890LUNA_SENSOR_ID                        (0x0890 + SENSOR_ID_OFFSET_LUNA)
#define SENSOR_DRVNAME_IMX890LUNA_MIPI_RAW      "imx890luna_mipi_raw"

#define SENSOR_ID_OFFSET_OMEGAS2                        0x3265
#define OMEGAS2MAIN_SENSOR_ID                           0x3AE7     /*(0x0882 + SENSOR_ID_OFFSET_OMEGAS2) */
#define SENSOR_DRVNAME_OMEGAS2MAIN_MIPI_RAW             "omegas2main_mipi_raw"
#define OMEGAS2WIDE_SENSOR_ID                           0x35BA     /*(0x0355 + SENSOR_ID_OFFSET_OMEGAS2) */
#define SENSOR_DRVNAME_OMEGAS2WIDE_MIPI_RAW             "omegas2wide_mipi_raw"
#define OMEGAS2TELE_SENSOR_ID                           0x6B4B     /*(0x38E5 + SENSOR_ID_OFFSET_OMEGAS2 + 0x1) */
#define SENSOR_DRVNAME_OMEGAS2TELE_MIPI_RAW             "omegas2tele_mipi_raw"
#define OMEGAS2FRONT_SENSOR_ID                          0x6B4A     /*(0x38E5 + SENSOR_ID_OFFSET_OMEGAS2) */
#define SENSOR_DRVNAME_OMEGAS2FRONT_MIPI_RAW            "omegas2front_mipi_raw"

#define SENSOR_ID_OFFSET_ALPHAH                       0x4051
#define ALPHAHMAIN_SENSOR_ID                          0x9095     /*(0x5044 + SENSOR_ID_OFFSET_ALPHAH) */
#define SENSOR_DRVNAME_ALPHAHMAIN_MIPI_RAW            "alphahmain_mipi_raw"
#define ALPHAHFRONT_SENSOR_ID                         0x44C2     /*(0x0471 + SENSOR_ID_OFFSET_ALPHAH) */
#define SENSOR_DRVNAME_ALPHAHFRONT_MIPI_RAW           "alphahfront_mipi_raw"
#define ALPHAHMONO_SENSOR_ID                          0x4331     /*(0x02e0 + SENSOR_ID_OFFSET_ALPHAH) */
#define SENSOR_DRVNAME_ALPHAHMONO_MIPI_RAW            "alphahmono_mipi_raw"

#define SENSOR_ID_OFFSET_GREENLANDFRONT                  0x3
#define SENSOR_ID_OFFSET_GREENLANDMAIN                   0x4
#define GREENLANDFRONT_SENSOR_ID                         0xD157      /*(0xD154 + SENSOR_ID_OFFSET_GREENLANDFRONT)*/
#define SENSOR_DRVNAME_GREENLANDFRONT_MIPI_RAW          "greenlandfront_mipi_raw"
#define GREENLANDMAIN_SENSOR_ID                          0xD158      /*(0xD154 + SENSOR_ID_OFFSET_GREENLANDMAIN)*/
#define SENSOR_DRVNAME_GREENLANDMAIN_MIPI_RAW           "greenlandmain_mipi_raw"

#define SENSOR_ID_OFFSET_BAIKALM                         0x0111
#define BAIKALMMAIN_SENSOR_ID                            0x5155  /*(0x5044 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMMAIN_MIPI_RAW             "baikalmmain_mipi_raw"
#define BAIKALMFRONT_SENSOR_ID                           0x09B9  /*(0x08A8 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMFRONT_MIPI_RAW            "baikalmfront_mipi_raw"
#define BAIKALMFRONT2_SENSOR_ID                          0x321B  /*(0x3109 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMFRONT2_MIPI_RAW           "baikalmfront2_mipi_raw"
#define BAIKALMFRONT3_SENSOR_ID                          0x0582     /*(0x0471 + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMFRONT3_MIPI_RAW           "baikalmfront3_mipi_raw"
#define BAIKALMMONO_SENSOR_ID                            0x013C  /*(0x002B + SENSOR_ID_OFFSET_BAIKALM) */
#define SENSOR_DRVNAME_BAIKALMMONO_MIPI_RAW             "baikalmmono_mipi_raw"

#define SENSOR_ID_OFFSET_OMEGAS3                        0x3261
#define OMEGAS3MAIN_SENSOR_ID                           0x3AE3     /*(0x0882 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3MAIN_MIPI_RAW             "omegas3main_mipi_raw"
#define OMEGAS3WIDE_SENSOR_ID                           0x35B6     /*(0x0355 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3WIDE_MIPI_RAW             "omegas3wide_mipi_raw"
#define OMEGAS3MACRO_SENSOR_ID                           0x328D     /*(0x002B + SENSOR_ID_OFFSET_OMEGAS3 + 0x1) */
#define SENSOR_DRVNAME_OMEGAS3MACRO_MIPI_RAW            "omegas3macro_mipi_raw"
#define OMEGAS3FRONT_SENSOR_ID                          0x6B46     /*(0x38E5 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3FRONT_MIPI_RAW            "omegas3front_mipi_raw"
#define OMEGAS3FRONT2_SENSOR_ID                          0x6543     /*(0x33E2 + SENSOR_ID_OFFSET_OMEGAS3) */
#define SENSOR_DRVNAME_OMEGAS3FRONT2_MIPI_RAW            "omegas3front2_mipi_raw"

#endif    /* __OPLUS_KD_IMGSENSOR_H */

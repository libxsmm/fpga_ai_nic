/******************************************************************************
* Copyright (c) Intel Corporation - All rights reserved.                      *
* This file is part of the LIBXSMM library.                                   *
*                                                                             *
* For information on the license, see the LICENSE file.                       *
* Further information: https://github.com/libxsmm/libxsmm/                    *
* SPDX-License-Identifier: BSD-3-Clause                                       *
******************************************************************************/
/* Rui Ma (Intel Corp.)
******************************************************************************/

// Application default configuration can override the base configuration.
`include "cci_mpf_app_conf.vh"

//
// MPF default configuration.
//
`ifndef MPF_CONF_SORT_READ_RESPONSES
  `define MPF_CONF_SORT_READ_RESPONSES 0
`endif

`ifndef MPF_CONF_PRESERVE_WRITE_MDATA
  `define MPF_CONF_PRESERVE_WRITE_MDATA 0
`endif

`ifndef MPF_CONF_ENABLE_VTP
  `define MPF_CONF_ENABLE_VTP 1
`endif

`ifndef MPF_CONF_ENABLE_VC_MAP
  `define MPF_CONF_ENABLE_VC_MAP 0
`endif

`ifndef MPF_CONF_ENABLE_DYNAMIC_VC_MAPPING
  // This flag only matters when MPF_CONF_ENABLE_VC_MAP is 1
  `define MPF_CONF_ENABLE_DYNAMIC_VC_MAPPING 1
`endif

`ifndef MPF_CONF_ENABLE_LATENCY_QOS
  `define MPF_CONF_ENABLE_LATENCY_QOS 0
`endif

`ifndef MPF_CONF_ENFORCE_WR_ORDER
  `define MPF_CONF_ENFORCE_WR_ORDER 0
`endif

`ifndef MPF_CONF_ENABLE_PARTIAL_WRITES
  `define MPF_CONF_ENABLE_PARTIAL_WRITES 0
`endif

`ifndef MPF_CONF_MERGE_DUPLICATE_READS
  `define MPF_CONF_MERGE_DUPLICATE_READS 0
`endif

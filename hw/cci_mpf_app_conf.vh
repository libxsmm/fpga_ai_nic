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

//
// Application configuration.
//

// Use virtual addresses in the AFU
`define MPF_CONF_ENABLE_VTP 1

// Ordered responses are required by the application
`define MPF_CONF_SORT_READ_RESPONSES 1

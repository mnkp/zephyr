# Copyright (c) 2018, Evry ASA
#
# SPDX-License-Identifier: Apache-2.0
#
board_runner_args(jlink "--device=EFR32MG1PxxxF256")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

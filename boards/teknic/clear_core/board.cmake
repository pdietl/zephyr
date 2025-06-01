# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=atsame53n19a" "--speed=4000")

# Since this one is first, it will be the default
include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

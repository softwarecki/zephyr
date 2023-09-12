# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2023 Intel Corporation
#
# Author: Adrian Warecki <adrian.warecki@intel.com>

string(REPLACE " " ";" lib_list ${LIBS})

file(WRITE ${OUT_FILE} "/* File generated by link_loadable_mods.cmake */\n")

foreach(lib ${lib_list})
  set(prefix "loadable-${lib}")

  file(APPEND ${OUT_FILE} "\n")
  file(APPEND ${OUT_FILE} "  .${prefix}-text : ALIGN(4096) {\n")
  file(APPEND ${OUT_FILE} "    *libloadable_library_${lib}.a:(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)\n")
  file(APPEND ${OUT_FILE} "  } >ram\n")
  file(APPEND ${OUT_FILE} "\n")
  file(APPEND ${OUT_FILE} "  .${prefix}-rodata : ALIGN(4096) {\n")
  file(APPEND ${OUT_FILE} "    *libloadable_library_${lib}.a:(.rodata .rodata.*)\n")
  file(APPEND ${OUT_FILE} "  } >ram\n")
  file(APPEND ${OUT_FILE} "\n")
  file(APPEND ${OUT_FILE} "  .${prefix}-data SEGSTART_UNCACHED : ALIGN(4096) {\n")
  file(APPEND ${OUT_FILE} "    *libloadable_library_${lib}.a:(.data .data.*)\n")
  file(APPEND ${OUT_FILE} "  } >ucram\n")
  file(APPEND ${OUT_FILE} "\n")
  file(APPEND ${OUT_FILE} "  .${prefix}-bss SEGSTART_UNCACHED (NOLOAD) : ALIGN(4096) {\n")
  file(APPEND ${OUT_FILE} "    *libloadable_library_${lib}.a:(.bss .bss.*)\n")
  file(APPEND ${OUT_FILE} "  } >ucram\n")
  file(APPEND ${OUT_FILE} "\n")

  # rimage module manifest headers
  file(APPEND ${OUT_FILE} "  .${prefix}-manifest : {\n")
  file(APPEND ${OUT_FILE} "    KEEP(*libloadable_library_${lib}.a:(.module))\n")
  file(APPEND ${OUT_FILE} "  } >noload\n")
endforeach()

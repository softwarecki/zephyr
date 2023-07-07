# SPDX-License-Identifier: Apache-2.0

add_compile_options(-Wall)
add_compile_options(-Wunused)
add_compile_options(-Werror)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_DAI_INTEL_DMIC	dmic.c)
zephyr_library_sources_ifdef(CONFIG_DAI_INTEL_DMIC_NHLT	dmic_nhlt.c)

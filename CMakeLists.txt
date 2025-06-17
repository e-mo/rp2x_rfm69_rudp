cmake_minimum_required(VERSION 3.13)

set(target "rp2x_rfm69_rudp_lib")
project(${target} C CXX ASM)

add_subdirectory(libs/rp2x_gpio_irq)
add_subdirectory(libs/rp2x_rfm69)

add_library(${target} INTERFACE)

target_sources(${target} INTERFACE
	src/rp2x_rfm69_rudp.c
	src/rfm69_lp.c
)

target_include_directories(${target} INTERFACE
	src
)

target_link_libraries(${target} INTERFACE
	rp2x_gpio_irq_lib
	rp2x_rfm69_lib

	pico_rand
	pico_sync
	pico_time
	hardware_dma
)

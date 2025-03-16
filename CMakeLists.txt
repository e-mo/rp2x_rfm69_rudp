cmake_minimum_required(VERSION 3.13)

project(rp2x_rfm69_rudp_lib C CXX ASM)

add_subdirectory(rp2x_gpio_irq)
add_subdirectory(rp2x_rfm69)

add_library(rp2x_rfm69_rudp_lib INTERFACE)

target_sources(rp2x_rfm69_rudp_lib INTERFACE
	#src/rfm69_rp2040_interface.c
	#src/rfm69_rp2040_rudp.c
	src/rp2x_rfm69_rudp.c
	#src/rfm69_rp2040_rudp_irq.c
)

target_include_directories(rp2x_rfm69_rudp_lib INTERFACE
	src
)

target_link_libraries(rp2x_rfm69_rudp_lib INTERFACE
	rp2x_gpio_irq_lib
	rp2x_rfm69_lib
	pico_rand
	pico_sync
)

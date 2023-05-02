cmake_minimum_required(VERSION 3.15)

project(Stm32CubeMXFreeRTOS LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.project_importers.stm32.cubemx
	SOURCE_PROJECT Stm32CubeMXFreeRTOS.ioc
	FRAMEWORKS com.sysprogs.embedded.semihosting_and_profiler
	DUMP_STACK_USAGE)

add_bsp_based_executable(
	NAME Stm32CubeMXFreeRTOS
	GENERATE_BIN
	GENERATE_MAP
	OUTPUT_RELOCATION_RECORDS)


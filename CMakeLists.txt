cmake_minimum_required(VERSION 3.15)

project(Stm32CubeMXFreeRTOS LANGUAGES C CXX ASM)

find_bsp(
	ID com.sysprogs.project_importers.stm32.cubemx
	SOURCE_PROJECT Stm32CubeMXFreeRTOS.ioc
	FRAMEWORKS com.sysprogs.embedded.semihosting_and_profiler
	DUMP_STACK_USAGE
	HWREGISTER_LIST_FILE ../../Users/volod/AppData/Local/VisualGDB/EmbeddedBSPs/arm-eabi/com.sysprogs.arm.stm32/STM32F7xxxx/DeviceDefinitions/stm32f767xx.xml.gz
	FWCONFIGURATION
		com.sysprogs.efp.semihosting.buffer_size=16384
		com.sysprogs.efp.profiling.counter=SYSPROGS_PROFILER_USE_DWT_CYCLE_COUNTER=0
		com.sysprogs.efp.profiling.hold_interrupts=FAST_SEMIHOSTING_HOLD_INTERRUPTS=0
	CONFIGURATION
		"com.sysprogs.toolchainoptions.arm.libctype=--specs=nano.specs -u _printf_float -u _scanf_float"
		com.sysprogs.toolchainoptions.arm.syscallspecs=)

add_bsp_based_executable(
	NAME Stm32CubeMXFreeRTOS
	GENERATE_BIN
	GENERATE_MAP
	OUTPUT_RELOCATION_RECORDS)


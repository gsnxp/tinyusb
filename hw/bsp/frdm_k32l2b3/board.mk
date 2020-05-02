CFLAGS += \
  -flto \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m0plus \
  -DCPU_K32L2B31VLH0A \
  -DCFG_TUSB_MCU=OPT_MCU_K32L2BXX \
  -DCFG_TUSB_MEM_SECTION='__attribute__((section(".data")))' \
  -DCFG_TUSB_MEM_ALIGN='__attribute__((aligned(64)))' 

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter

MCU_DIR = hw/mcu/nxp/sdk/devices/K32L2B31A

# All source paths should be relative to the top level.
LD_FILE = $(MCU_DIR)/gcc/K32L2B31xxxxA_flash.ld

SRC_C += \
	$(MCU_DIR)/system_K32L2B31A.c \
	$(MCU_DIR)/project_template/clock_config.c \
	$(MCU_DIR)/project_template/pin_mux.c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(MCU_DIR)/drivers/fsl_gpio.c \
	$(MCU_DIR)/drivers/fsl_lpuart.c
#	$(MCU_DIR)/drivers/fsl_smc.c 

INC += \
	$(TOP)/$(MCU_DIR)/../../CMSIS/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(MCU_DIR)/project_template

SRC_S += $(MCU_DIR)/gcc/startup_K32L2B31A.S

# For TinyUSB port source
VENDOR = nxp
CHIP_FAMILY = kinetis

# For freeRTOS port source
FREERTOS_PORT = ARM_CM0

# For flash-jlink target
JLINK_DEVICE = K32L2B31A
JLINK_IF = swd

# flash using pyocd
flash: $(BUILD)/$(BOARD)-firmware.hex
	pyocd flash -t K32L2B31A $<

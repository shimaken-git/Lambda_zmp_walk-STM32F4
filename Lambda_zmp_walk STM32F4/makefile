SHELL = cmd.exe
TARGET_ARCH   = -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -march=armv7e-m -mtune=cortex-m4
#TARGET_ARCH   = -mcpu=cortex-m4 -mthumb
INCLUDE_DIRS  = -I ../../Libraries \
				-I ../../Libraries/STM32F4xx_StdPeriph_Driver/inc \
				-I ../../Libraries/CMSIS/Device/ST/STM32F4xx/include \
				-I ../../Libraries/CMSIS/Include \
				-I $(TOOLDIR)../arm-none-eabi/include \
				-I $(TOOLDIR)../arm-none-eabi/include/c++/5.4.1
STARTUP_DIR = ../../Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7/
BOARD_OPTS = -DHSE_VALUE=((uint32_t)12000000) -DSTM32F4XX
FIRMWARE_OPTS = -DUSE_STDPERIPH_DRIVER
COMPILE_OPTS  = -Os -g3 -ffunction-sections -fdata-sections -fsigned-char -fno-rtti -fno-exceptions -Wall -fmessage-length=0 $(INCLUDE_DIRS) $(BOARD_OPTS) $(FIRMWARE_OPTS)

#TOOLDIR = ../../yagarto/bin/
TOOLDIR = ../../../cygwin64/opt/GNU_Tools_ARM_Embedded/5.4_2016q3/bin/
CC      = $(TOOLDIR)arm-none-eabi-gcc
CXX		= $(CC)
AS      = $(CC)
LD      = $(CC)
AR      = $(TOOLDIR)arm-none-eabi-ar
OBJCOPY = $(TOOLDIR)arm-none-eabi-objcopy
CFLAGS  = $(COMPILE_OPTS)
CXXFLAGS= $(COMPILE_OPTS)
ASFLAGS = -x assembler-with-cpp -c $(TARGET_ARCH) $(COMPILE_OPTS) 
LDFLAGS = -Wl,--gc-sections,-Map=bin\main.map,-cref -T stm32_flash.ld $(INCLUDE_DIRS) -lstdc++ -L $(TOOLDIR)../arm-none-eabi/lib/armv7e-m/fpu -L ../../Libraries
#LDFLAGS = -Wl,--gc-sections,-Map=bin\main.map,-cref -T stm32_flash.ld $(INCLUDE_DIRS) -lstdc++ -L $(TOOLDIR)../arm-none-eabi/lib/armv7e-m -L ../../Libraries

all: libstm32f4xx startup bin\main.hex

# main.o is compiled by suffix rule automatucally
bin\main.hex: $(patsubst %.c,%.o,$(wildcard *.c)) $(patsubst %.cpp,%.o,$(wildcard *.cpp)) $(STARTUP_DIR)startup_stm32f4xx.o ../../Libraries/libstm32f4xx.a 
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o bin\main.elf 
	$(OBJCOPY) -O ihex bin\main.elf bin\main.hex

# many of xxx.o are compiled by suffix rule automatically
LIB_OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard ../../Libraries/STM32F4xx_StdPeriph_Driver/src/*.c)))

libstm32f4xx: $(LIB_OBJS)
	$(AR) cr ../../Libraries/libstm32f4xx.a $(LIB_OBJS)
	
startup:
	$(AS) -o $(STARTUP_DIR)/startup_stm32f4xx.o $(ASFLAGS) $(STARTUP_DIR)startup_stm32f4xx.s

$(LIB_OBJS): \
 $(wildcard ../../Libraries/STM32F4xx_StdPeriph_Driver/inc/*.h) \
 $(wildcard ../../Libraries/STM32F4xx_StdPeriph_Driver/src/*.c) \
 makefile

clean:
	del /f /q *.o *.s bin\*
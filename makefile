PROJ=INSReceive

PREFIX:=arm-none-eabi-
CC:=$(PREFIX)gcc
CXX:=$(PREFIX)g++
LD:=$(CXX)
OBJCOPY:=$(PREFIX)objcopy
OBJDUMP:=$(PREFIX)objdump

ifndef TGMEM
TGMEM=FLASH
endif

INCLUDES=Drivers/CMSIS/Device/ST/STM32F4xx/Include Drivers/CMSIS/Include
DEFINES=STM32F407xx
IFLAGS=$(foreach dir,$(INCLUDES),-I$(dir))
DFLAGS=$(foreach dir,$(DEFINES),-D$(dir))
PF_FLAGS=-mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS:=$(IFLAGS) $(DFLAGS) $(PF_FLAGS) -ffunction-sections -fdata-sections -O0 -Wall -Wextra -std=c11 -g
CXXFLAGS:=$(IFLAGS) $(DFLAGS) $(PF_FLAGS) -ffunction-sections -fdata-sections -O0 -Wall -Wextra -fno-rtti -fno-exceptions -std=c++14 -g
BUILDDIR=build/
LDSCRIPT=STM32F407VGTx_$(TGMEM).ld
LDFLAGS:=$(PF_FLAGS) -Wl,--gc-sections -specs=nano.specs


rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

C_SOURCES	:= $(call rwildcard,,*.c)
CXX_SOURCES	:= $(call rwildcard,,*.cc)
S_SOURCES	:= startup/startup_stm32f407xx_$(TGMEM).s

C_OBJECTS	:= $(addprefix $(BUILDDIR),$(patsubst %.c,%.o,$(C_SOURCES)))
CXX_OBJECTS	:= $(addprefix $(BUILDDIR),$(patsubst %.cc,%.o,$(CXX_SOURCES)))
S_OBJECTS	:= $(addprefix $(BUILDDIR),$(patsubst %.s,%.o,$(S_SOURCES)))

C_DEP		:= $(addprefix $(BUILDDIR),$(patsubst %.c,%.dep,$(C_SOURCES)))
CXX_DEP		:= $(addprefix $(BUILDDIR),$(patsubst %.cc,%.dep,$(CXX_SOURCES)))
DEP			:= $(C_DEP) $(CXX_DEP)

OBJECTS		:= $(C_OBJECTS) $(CXX_OBJECTS) $(S_OBJECTS)
ELF			:= $(BUILDDIR)$(PROJ)_$(TGMEM).elf
BIN			:= $(BUILDDIR)$(PROJ)_$(TGMEM).bin
DISAS		:= $(BUILDDIR)$(PROJ)_$(TGMEM).S

.PHONY : clean

all : $(DISAS) $(BIN) $(ELF)

$(DISAS) : $(ELF)
	$(OBJDUMP) -d -C $< > $@

$(BIN) : $(ELF)
	@ sh -c "mkdir -p $(dir $@)"
	$(OBJCOPY) -O binary $< $@

$(ELF) : $(OBJECTS) $(LDSCRIPT)
	@ sh -c "mkdir -p $(dir $@)"
	${LD} -o $@ $(OBJECTS) -T$(LDSCRIPT) $(LDFLAGS)

$(S_OBJECTS) : $(BUILDDIR)%.o : %.s
	@ sh -c "mkdir -p $(dir $@)"
	$(CC) -c -o $@ $< $(CFLAGS)

$(C_OBJECTS) : $(BUILDDIR)%.o : %.c
	@ sh -c "mkdir -p $(dir $@)"
	$(CC) -c -o $@ $< $(CFLAGS)

$(CXX_OBJECTS) : $(BUILDDIR)%.o : %.cc
	@ sh -c "mkdir -p $(dir $@)"
	$(CXX) -c -o $@ $< $(CXXFLAGS)

$(C_DEP) : $(BUILDDIR)%.dep : %.c
	@ sh -c "mkdir -p $(dir $@)"
	$(CC) -M -MM -MF $@ $< -MT $(patsubst %.dep,%.o,$@) $(CFLAGS)

$(CXX_DEP) : $(BUILDDIR)%.dep : %.cc
	@ sh -c "mkdir -p $(dir $@)"
	$(CXX) -M -MM -MF $@ $< -MT $(patsubst %.dep,%.o,$@) $(CXXFLAGS)

-include $(DEP)

$(ODIRS)	:
	mkdir -p $@

clean :
	rm -f $(OBJECTS) $(ELF) $(BIN)

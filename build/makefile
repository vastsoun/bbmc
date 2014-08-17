#
# Locating the root directories
#
STARTERWARE_ROOT=../../bbmc-dev
BBMC_ROOT=..

# Project Directories
BBMC_SRC=$(BBMC_ROOT)/src
BBMC_BLD=$(BBMC_ROOT)/build
BBMC_BIN=$(BBMC_ROOT)/build/bin


#
# The build targets (definitionis for the C pre-processor)
#
EVM=beaglebone
DEVICE=am335x


#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
ROOT=$(STARTERWARE_ROOT)
include makedefs

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${SYSCONFIG_BLD} ${UTILITY_BLD}

#
# The application name
#
APPNAME=bbmc-0.6

#
# Where the application will be loaded to. This is required to generate
# image with Header (Load Address and Size)
#
IMG_LOAD_ADDR = 0x80000000

#
# Application Location
#
APP=$(BBMC_ROOT)
APP_BIN=$(BBMC_BIN)

#
# Application source files
#
APP_SRC=$(BBMC_SRC)/*.c


#
# Required library files
#
APP_LIB=-ldrivers -lutils -lplatform -lsystem_config

#
# ECHO configs for output coloring
#

ECHO_Blue=\e[0;34m
ECHO_LBlue=\e[1;34m
ECHO_Green=\e[0;32m
ECHO_Cyan=\e[0;36m
ECHO_Red=\e[0;31m
ECHO_Purple=\e[0;35m
ECHO_NC=\e[0m



#
# Rules for building the application and library
#
#all: debug release
all: debug

debug:
	@echo -e "\n\n${ECHO_LBlue}Building the BBMC application${ECHO_NC}\n";
	make TARGET_MODE=debug lib
	@echo -e "\n${ECHO_Blue}Building StarterWare Completed.${ECHO_NC}\n";
	make TARGET_MODE=Debug compile
	make TARGET_MODE=Debug link
	make TARGET_MODE=Debug bincpy
	make TARGET_MODE=Debug ti_header

release:
	make TARGET_MODE=release lib
	make TARGET_MODE=Release bin

lib:
	@if [ ${TARGET_MODE} != "clean" ];		\
	then		\
		echo -e "\n${ECHO_Blue}Building StarterWare Firmware and Libraries${ECHO_NC}\n"; \
	fi; 
	@for i in ${DIRS};				\
	do						\
		if [ -f $${i}/makefile ] ;		    \
		then					  \
			if [ ${TARGET_MODE} != "clean" ];		\
			then		\
				echo -e "\n${ECHO_Blue}Building: $${i} ${ECHO_NC}"; \
			fi;		\
			make $(TARGET_MODE) -C $${i} || exit $$?; \
		fi;					   \
	done;


compile:
	@echo -e "\n${ECHO_Blue}Compiling BBMC source.${ECHO_NC}";
	$(CC) $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)/
	@mv *.o* $(TARGET_MODE)/
	@echo -e "${ECHO_Blue}..Compiling Completed.${ECHO_NC}\n";

link:
	@echo -e "\n${ECHO_Blue}Linking objects with static libraries..${ECHO_NC}";
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)/$(APPNAME).out -Map $(TARGET_MODE)/$(APPNAME).map \
	$(TARGET_MODE)/*.o* -T $(APPNAME).lds $(APP_LIB) -lc -lm -lgcc $(APP_LIB) -lc -lm -lgcc 
	@mkdir -p $(APP_BIN)/$(TARGET_MODE);
	@cp $(TARGET_MODE)/$(APPNAME).out $(APP_BIN)/$(TARGET_MODE)/$(APPNAME).out;
	@echo -e "${ECHO_Blue}..Linking Completed.${ECHO_NC}\n";

bincpy:
	@echo -e "\n${ECHO_Blue}Bincopy is doing its this..${ECHO_NC}";
	$(BIN) $(BINFLAGS) $(APP_BIN)/$(TARGET_MODE)/$(APPNAME).out \
    $(APP_BIN)/$(TARGET_MODE)/$(APPNAME).bin;
	@echo -e "${ECHO_Blue}..Bincopy Completed.${ECHO_NC}\n";

ti_header:
	@echo -e "\n${ECHO_Blue}Appending TI Image Header..${ECHO_NC}";
	@cd $(STARTERWARE_ROOT)/tools/ti_image/; gcc tiimage.c -o temp.out; cd -
	$(STARTERWARE_ROOT)/tools/ti_image/temp.out $(IMG_LOAD_ADDR) $(BOOT) \
        $(APP_BIN)/$(TARGET_MODE)/$(APPNAME).bin \
        $(APP_BIN)/$(TARGET_MODE)/$(APPNAME)_ti.bin;
	@rm -rf $(STARTERWARE_ROOT)/tools/ti_image/temp.out;
	@echo -e "${ECHO_Blue}..Header Creation Completed.${ECHO_NC}\n";


#
# Rules for cleaning
#
clean:
	@echo -e "\n${ECHO_Cyan}Cleaning the BBMC application files. Removing *.o and *.bin files.${ECHO_NC}";
	@rm -rf $(APP_BIN)/Debug $(APP_BIN)/Release
	@rm -rf Debug Release

clean+: clean
	@echo -e "${ECHO_Cyan}Cleaning the built StartWare Libraries. Removing *.o and *.bin files.${ECHO_NC}";
	make TARGET_MODE=clean lib
	@rm -rf $(STARTERWARE_ROOT)/bin/*
	@echo -e "${ECHO_Cyan}Cleaning Complete.\n${ECHO_NC}";



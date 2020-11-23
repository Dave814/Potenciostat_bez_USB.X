#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=heflash/Flash.c heflash/HEFlash.c spi/spi_software.c main.c eusart.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/heflash/Flash.p1 ${OBJECTDIR}/heflash/HEFlash.p1 ${OBJECTDIR}/spi/spi_software.p1 ${OBJECTDIR}/main.p1 ${OBJECTDIR}/eusart.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/heflash/Flash.p1.d ${OBJECTDIR}/heflash/HEFlash.p1.d ${OBJECTDIR}/spi/spi_software.p1.d ${OBJECTDIR}/main.p1.d ${OBJECTDIR}/eusart.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/heflash/Flash.p1 ${OBJECTDIR}/heflash/HEFlash.p1 ${OBJECTDIR}/spi/spi_software.p1 ${OBJECTDIR}/main.p1 ${OBJECTDIR}/eusart.p1

# Source Files
SOURCEFILES=heflash/Flash.c heflash/HEFlash.c spi/spi_software.c main.c eusart.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=16F1459
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/heflash/Flash.p1: heflash/Flash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/heflash" 
	@${RM} ${OBJECTDIR}/heflash/Flash.p1.d 
	@${RM} ${OBJECTDIR}/heflash/Flash.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1  --debugger=pickit3    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/heflash/Flash.p1 heflash/Flash.c 
	@-${MV} ${OBJECTDIR}/heflash/Flash.d ${OBJECTDIR}/heflash/Flash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/heflash/Flash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/heflash/HEFlash.p1: heflash/HEFlash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/heflash" 
	@${RM} ${OBJECTDIR}/heflash/HEFlash.p1.d 
	@${RM} ${OBJECTDIR}/heflash/HEFlash.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1  --debugger=pickit3    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/heflash/HEFlash.p1 heflash/HEFlash.c 
	@-${MV} ${OBJECTDIR}/heflash/HEFlash.d ${OBJECTDIR}/heflash/HEFlash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/heflash/HEFlash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/spi/spi_software.p1: spi/spi_software.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/spi" 
	@${RM} ${OBJECTDIR}/spi/spi_software.p1.d 
	@${RM} ${OBJECTDIR}/spi/spi_software.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1  --debugger=pickit3    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/spi/spi_software.p1 spi/spi_software.c 
	@-${MV} ${OBJECTDIR}/spi/spi_software.d ${OBJECTDIR}/spi/spi_software.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/spi/spi_software.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1  --debugger=pickit3    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/eusart.p1: eusart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eusart.p1.d 
	@${RM} ${OBJECTDIR}/eusart.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1  --debugger=pickit3    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/eusart.p1 eusart.c 
	@-${MV} ${OBJECTDIR}/eusart.d ${OBJECTDIR}/eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/heflash/Flash.p1: heflash/Flash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/heflash" 
	@${RM} ${OBJECTDIR}/heflash/Flash.p1.d 
	@${RM} ${OBJECTDIR}/heflash/Flash.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/heflash/Flash.p1 heflash/Flash.c 
	@-${MV} ${OBJECTDIR}/heflash/Flash.d ${OBJECTDIR}/heflash/Flash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/heflash/Flash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/heflash/HEFlash.p1: heflash/HEFlash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/heflash" 
	@${RM} ${OBJECTDIR}/heflash/HEFlash.p1.d 
	@${RM} ${OBJECTDIR}/heflash/HEFlash.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/heflash/HEFlash.p1 heflash/HEFlash.c 
	@-${MV} ${OBJECTDIR}/heflash/HEFlash.d ${OBJECTDIR}/heflash/HEFlash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/heflash/HEFlash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/spi/spi_software.p1: spi/spi_software.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/spi" 
	@${RM} ${OBJECTDIR}/spi/spi_software.p1.d 
	@${RM} ${OBJECTDIR}/spi/spi_software.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/spi/spi_software.p1 spi/spi_software.c 
	@-${MV} ${OBJECTDIR}/spi/spi_software.d ${OBJECTDIR}/spi/spi_software.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/spi/spi_software.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/eusart.p1: eusart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eusart.p1.d 
	@${RM} ${OBJECTDIR}/eusart.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist -DXPRJ_default=$(CND_CONF)  --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib $(COMPARISON_BUILD)  --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     -o${OBJECTDIR}/eusart.p1 eusart.c 
	@-${MV} ${OBJECTDIR}/eusart.d ${OBJECTDIR}/eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.map  -D__DEBUG=1  --debugger=pickit3  -DXPRJ_default=$(CND_CONF)    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"        $(COMPARISON_BUILD)  -odist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.hex 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)    --double=32 --float=32 --opt=+asm,+asmfile,-speed,+space,-debug --addrqual=ignore --mode=free -P -N255 -I"C:/Users/david/Desktop/Potenciostat/Projekt/Potenciostat.X" --warn=-3 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-osccal,-resetbits,-download,-stackcall,+clib --output=-mcof,+elf "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"     $(COMPARISON_BUILD)  -odist/${CND_CONF}/${IMAGE_TYPE}/Potenciostat_bez_USB.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif

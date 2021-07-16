################################################################################
# compile.mk
# 
# HISTORY
# 17/06/2021  |                                             | Daich
################################################################################

ifeq "$(COMPILER)" "IAR"
TC_CC:= iccarm.exe
TC_CPP:= iccarm.exe
TC_ASM:= iasmarm.exe
TC_LINK:= ilinkarm.exe
TC_AR:= iarchive.exe


else
TC_CC:= gcc
TC_CPP:= g++
TC_LINK:= gcc
TC_AR:= ar
endif



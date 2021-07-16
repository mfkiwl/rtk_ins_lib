################################################################################
# Makefile
# 
# HISTORY
# 17/06/2021  |                                             | Daich
################################################################################

ifndef TG
	TG=WINDOWS
	COMPILER=IAR
endif


BUILD_OUTPUT              :=  ins_lib_output


# Path to SDK root dir
INS_LIB_ROOTDIR:=$(realpath $(dir $(realpath $(lastword ${MAKEFILE_LIST})))../)

# Absolute path to makefiles
MAKEFILES_ROOTDIR:=${INS_LIB_ROOTDIR}/build/modules

# Absolute path to output
OUTPUT_ROOTDIR:=${INS_LIB_ROOTDIR}/build/output


# including product options
#


include ${MAKEFILES_ROOTDIR}/build.mk


.PHONY : all_test

all: prebuild build 



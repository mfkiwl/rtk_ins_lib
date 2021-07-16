################################################################################
# common.mk
# 
# HISTORY
# 17/06/2021  |                                             | Daich
################################################################################
include ${MAKEFILES_ROOTDIR}/common/macros.mk
include ${MAKEFILES_ROOTDIR}/ins_core/ins_defs.mk


TC_COPTS:=\
  --endian=little         \
  --cpu=Cortex-M7         \
  --fpu=VFPv5_D16         \
  --no_cse                \
  --no_unroll             \
  --no_inline             \
  --no_code_motion        \
  --no_tbaa               \
  --no_clustering         \
  --no_scheduling         \
  --debug                 \
  -e                      

#  -std=c11               \
#  -MD                    \
  -MF                   
#  -ferror-limit=0       
#  -fbracket-depth=512   

CORE_COPTS:=            \
                     

CORE_CPPOPTS:=          \
  -O1       

# folder for tools
TOOLS_DIR:=${INS_LIB_ROOTDIR}/build/tools


# folder for target files
OUTPUT_DIR:=${OUTPUT_ROOTDIR}
TARGET_ROOTDIR:=${OUTPUT_ROOTDIR}/${BUILD_OUTPUT}


# folder for common includes
COMMON_LIBDIR:=


#
# Build sources list
#

PROJ_CSOURCES:= ${INS_C_SOURCES} 

PROJ_CPPSOURCES:= ${INS_CPP_SOURCES}

PROJ_SOURCES:= ${PROJ_CSOURCES}
#
# compiler section
#

COPTS:=$(strip ${TC_COPTS} ${CORE_COPTS})
CPPOPTS:=$(strip ${TC_COPTS} ${CORE_CPPOPTS} )


ifeq "$(TG)" "WINDOWS"
PLATFORM_CDEFS:= WINDOWS
else
PLATFORM_CDEFS:= LINUX
endif

CDEFS:=$(foreach def,$(strip ${PLATFORM_CDEFS} ${INS_CDEFS} ),-D${def})

CINCDIRS:=$(foreach inc,$(strip ${INS_CINCDIRS} ),-I$(call MK_TC_PATH,${INS_LIB_ROOTDIR}/${inc}))

ifeq "$(TG)" "LINUX"
ALLLIBS:= -lm
else
ALLLIBS:= 
endif
# C options file
COPTS_FILE:=${TARGET_ROOTDIR}/gccflags.txt

ifeq "$(TG)" "WINDOWS"
TARGET:=${TARGET_ROOTDIR}/ins_lib.a
else
TARGET:=${TARGET_ROOTDIR}/ins_lib.a
endif

# objects folder
OBJS_FOLDER:=${TARGET_ROOTDIR}/objs

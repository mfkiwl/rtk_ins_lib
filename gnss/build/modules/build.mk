################################################################################
# build.mk
# 
# HISTORY
# 17/06/2021  |                                             | Daich
################################################################################

include ${MAKEFILES_ROOTDIR}/common/common.mk

include ${MAKEFILES_ROOTDIR}/common/compile.mk

# C thumb sources
BUILD_CSOURCES:=$(foreach src,${PROJ_CSOURCES},${GNSS_LIB_ROOTDIR}/$(src))
BUILD_CPPSOURCES:=$(foreach src,${PROJ_CPPSOURCES},${GNSS_LIB_ROOTDIR}/$(src))

# C ARM sources
BUILD_CARMSOURCES:=$(foreach src,${PROJ_CARMSOURCES},${GNSS_LIB_ROOTDIR}/$(src))

# Assembly sources
BUILD_ASMSOURCES:=$(foreach src,${PROJ_ASMSOURCES},${GNSS_LIB_ROOTDIR}/$(src))

# All sources
BUILD_CSOURCES:=${BUILD_CSOURCES} ${BUILD_CARMSOURCES} ${BUILD_ASMSOURCES}
BUILD_CPPSOURCES:=${BUILD_CPPSOURCES} 


# All sources
PROJ_COBJS:=$(foreach obj,$(basename $(notdir ${BUILD_CSOURCES})),${OBJS_FOLDER}/${obj}.o)
PROJ_CPPOBJS:=$(foreach obj,$(basename $(notdir ${BUILD_CPPSOURCES})),${OBJS_FOLDER}/${obj}.o)

GCCFLAGS = ${COPTS}  ${CINCDIRS}  ${CDEFS} ${ALLLIBS}

GPPFLAGS = ${CPPOPTS}  ${CINCDIRS}  ${CDEFS} ${ALLLIBS}
################################################################################
#
# Targets section
#
################################################################################

# Via files targets
#

${COPTS_FILE}:  
	$(call MK_ECHO,Generating C compiler via file for ${PROJ_TC} builder)
	$(call MK_ECHO,${CDEFS})
	$(call MK_ECHO,${CINCDIRS})
	$(call MK_APPEND,${COPTS},"${COPTS_FILE}")
	$(call MK_APPEND,${CINCDIRS},"${COPTS_FILE}")
	$(call MK_APPEND,${CDEFS},"${COPTS_FILE}")

#
# Compiler implicit targets
#




-include ${OBJS_FOLDER}/%.d



$(PROJ_COBJS):${BUILD_CSOURCES} #${COPTS_FILE}
	$(eval SRCFILE:=${filter %${patsubst %.o,%.c,${notdir $@}} ,${BUILD_CSOURCES} })
	@$(call MK_ECHO,flags ${GCCFLAGS} )
# $(call MK_ECHO,BUILD_CSOURCES =  ${BUILD_CSOURCES} )
# $(call MK_ECHO,PROJ_COBJS =  ${PROJ_COBJS} )
	${TC_CC} ${SRCFILE} ${GCCFLAGS}  -o $@



$(PROJ_CPPOBJS):${BUILD_CPPSOURCES} 
	$(eval SRCFILE:=${filter %${patsubst %.o,%.cpp,${notdir $@}} ,${BUILD_CPPSOURCES} })
	$(call MK_ECHO,Compiling ${SRCFILE} )
	${TC_CPP}  ${GPPFLAGS} ${SRCFILE}  -o $@


${TARGET}:  ${PROJ_COBJS}  ${PROJ_CPPOBJS}
	$(call MK_ECHO,Linking to $(subst ${GNSS_LIB_ROOTDIR}/,,$@))
	@${TC_AR} ${PROJ_COBJS} ${PROJ_CPPOBJS} ${ALLLIBS} --create -o $@ 


prebuild:	
	$(info prebuild)
ifeq ($(call MK_FILEEXIST, $(OBJS_FOLDER)) ,  )
		$(call MK_MKDIR,"$(call MK_PATHTOWIN,${OBJS_FOLDER}))
else
		$(info exist )
endif


build:	${TARGET}



.PHONY : clean

clean: 
	$(call MK_RMDIR,${OBJS_FOLDER})
	
all_test:  
	$(prebuild)





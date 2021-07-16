################################################################################
# macors.mk
# 
# HISTORY
# 17/06/2021  |                                             | Daich
################################################################################
ifeq (${MACROS_MAKE},)

MACROS_MAKE:=included

# Build process constants
MK_COMMA:= ,
MK_EMPTY:=
MK_SPACE:= ${MK_EMPTY} ${MK_EMPTY}


MK_LOOKUPSRC=$(filter %/$(notdir ${1}),${2})

################################################################################
#
# Macros section
#
################################################################################

# convert path to win path
# 1 - Paths to convert
ifeq "$(TG)" "WINDOWS"
MK_PATHTOWIN=$(subst /,\,${1})
else
MK_PATHTOWIN=$(subst /,/,${1})
endif


# check if file exists
# 1 - File to check
MK_FILEEXIST=$(wildcard $(subst ", ,${1}))
# MK_FILEEXIST=$(wildcard ${1} )
# Echo a string
# 1 - files to type/append
MK_ECHO=$(if ${1}, echo ${1})

MK_TC_PATH=$(call MK_PATHTOWIN,${1})

# type files/append files to another one
# 1 - files to type/append
# 2 (opt) - destination file
MK_CAT=$(if $(and ${1},${2}),@${MK_BUSYBOX} cat ${1} >> ${2})

# create a folder if it does not exist
# 1 - folder name
ifeq "$(TG)" "WINDOWS"
MK_MKDIR=$(if ${1}, mkdir  $(1))
else
MK_MKDIR=$(if ${1}, mkdir -p $(1))
endif
# remove files
# 1 - files to remove
MK_RMFILE=$(if ${1}, rm  ${1})

# remove directory
# 1 - dir to remove
# MK_RMDIR=$(if ${1},@${MK_BUSYBOX} rm -rf ${1})
MK_RMDIR=$(if ${1}, rm  ${1} )

# copy files
# 1 - source file(s)
# 2 - destination file/folder
MK_CP=$(if $(and ${1},${2}),@${MK_BUSYBOX} cp ${1} ${2})

# append text to a file
# 1 - text to append
# 2 - destination file



ifeq "$(TG)" "WINDOWS"
$(info ------------------------------------------------------windows)
MK_APPEND=$(if $(and ${1},${2}),echo ${1} >> ${2})
else
$(info -----------------------------------------------------linux)
MK_APPEND=$(if $(and ${1},${2}),echo ${1}>>${2})
endif

endif
################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=-Wno-deprecated-enum-enum-conversion

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:=

# Exclude large image assets, LightLib, and EZ-Template from the hot binary -
# they live in the cold package instead, so they only need to re-upload when
# those sources actually change.
EXCLUDE_SRCDIRS := $(SRCDIR)/images $(SRCDIR)/LightLib $(SRCDIR)/EZ-Template

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS!
LIBNAME:=EZ-Template
VERSION:=3.2.2

# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/autons $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported

TEMPLATE_FILES=$(INCDIR)/EZ-Template/*.hpp $(INCDIR)/EZ-Template/drive/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

################################################################################
# Asset library — image data compiled into the cold package.
# Symbols are available to hot code via -R cold.elf at link time.
# Delete firmware/libgfxassets.a manually if you need a full cold rebuild.
ASSET_SRCS := $(wildcard $(SRCDIR)/images/*.c)
ASSET_OBJS := $(addprefix $(BINDIR)/,$(patsubst $(SRCDIR)/%,%.o,$(ASSET_SRCS)))
ASSET_LIB  := $(FWDIR)/libgfxassets.a

# Inject asset library into the cold package link only ($^ → --whole-archive).
# Hot/monolith use order-only so the library is never duplicated into the hot binary.
$(COLD_ELF): $(ASSET_LIB)
$(HOT_ELF): | $(ASSET_LIB)
$(MONOLITH_ELF): | $(ASSET_LIB)

# Only add the build rule when the library is completely absent (e.g. fresh git clone).
# firmware/ is NOT cleaned by `make clean`, so once built the library persists.
# When images change, run `make assets` explicitly to rebuild.
ifeq ($(wildcard $(ASSET_LIB)),)
$(ASSET_LIB): $(ASSET_OBJS)
	$(call test_output_2,Creating asset library $@ ,$(AR) rcs $@ $^,$(DONE_STRING))
endif

.PHONY: assets
assets: $(ASSET_OBJS)
	$(call test_output_2,Rebuilding asset library $(ASSET_LIB) ,$(AR) rcs $(ASSET_LIB) $^,$(DONE_STRING))

################################################################################
# LightLib library — LightLib source compiled into the cold package.
# Same pattern as the asset library above: the sources are excluded from the
# hot binary (via EXCLUDE_SRCDIRS) and linked into cold.elf as a static archive.
# Delete firmware/liblightlib.a manually if you need a full cold rebuild.
LIGHTLIB_SRCS := $(wildcard $(SRCDIR)/LightLib/*.cpp)
LIGHTLIB_OBJS := $(addprefix $(BINDIR)/,$(patsubst $(SRCDIR)/%,%.o,$(LIGHTLIB_SRCS)))
LIGHTLIB_LIB  := $(FWDIR)/liblightlib.a

# Inject LightLib into the cold package link only ($^ → --whole-archive).
# Hot/monolith use order-only so the library is never duplicated into the hot binary.
$(COLD_ELF): $(LIGHTLIB_LIB)
$(HOT_ELF): | $(LIGHTLIB_LIB)
$(MONOLITH_ELF): | $(LIGHTLIB_LIB)

# Only add the build rule when the library is completely absent (e.g. fresh git clone).
# firmware/ is NOT cleaned by `make clean`, so once built the library persists.
# When LightLib sources change, run `make lightlib` explicitly to rebuild.
ifeq ($(wildcard $(LIGHTLIB_LIB)),)
$(LIGHTLIB_LIB): $(LIGHTLIB_OBJS)
	$(call test_output_2,Creating LightLib library $@ ,$(AR) rcs $@ $^,$(DONE_STRING))
endif

.PHONY: lightlib
lightlib: $(LIGHTLIB_OBJS)
	$(call test_output_2,Rebuilding LightLib library $(LIGHTLIB_LIB) ,$(AR) rcs $(LIGHTLIB_LIB) $^,$(DONE_STRING))

################################################################################
# EZ-Template library — EZ-Template source compiled into the cold package.
# Same pattern as LightLib / assets above.  Moving EZ-Template to cold lets
# LightLib (also cold) resolve ez::* symbols at cold-link time.
# Delete firmware/libeztemplate.a manually if you need a full cold rebuild.
EZTEMPLATE_SRCS := $(call rwildcard,$(SRCDIR)/EZ-Template/,*.cpp,)
EZTEMPLATE_OBJS := $(addprefix $(BINDIR)/,$(patsubst $(SRCDIR)/%,%.o,$(EZTEMPLATE_SRCS)))
EZTEMPLATE_LIB  := $(FWDIR)/libeztemplate.a

$(COLD_ELF): $(EZTEMPLATE_LIB)
$(HOT_ELF): | $(EZTEMPLATE_LIB)
$(MONOLITH_ELF): | $(EZTEMPLATE_LIB)

ifeq ($(wildcard $(EZTEMPLATE_LIB)),)
$(EZTEMPLATE_LIB): $(EZTEMPLATE_OBJS)
	$(call test_output_2,Creating EZ-Template library $@ ,$(AR) rcs $@ $^,$(DONE_STRING))
endif

.PHONY: eztemplate
eztemplate: $(EZTEMPLATE_OBJS)
	$(call test_output_2,Rebuilding EZ-Template library $(EZTEMPLATE_LIB) ,$(AR) rcs $(EZTEMPLATE_LIB) $^,$(DONE_STRING))

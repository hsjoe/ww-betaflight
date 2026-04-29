
# handle only this directory as in-tree config directory
CONFIGS_SUBMODULE_DIR := src/config
BASE_CONFIGS           = $(sort $(notdir $(patsubst %/,%,$(dir $(wildcard $(CONFIG_DIR)/configs/*/config.h)))))

ifneq ($(words $(CONFIG_DIR)),1)
$(error CONFIG_DIR/BETAFLIGHT_CONFIG path contains whitespace; unsupported by GNU make wildcard.)
endif

ifneq ($(filter-out %_sdk %_install test% %_clean clean% %-print %.hex %.h hex checks help configs $(BASE_TARGETS) $(BASE_CONFIGS),$(MAKECMDGOALS)),)
ifeq ($(wildcard $(CONFIG_DIR)/configs/),)
$(error `$(CONFIG_DIR)` not found. Expected in-tree configs under `src/config` or set CONFIG_DIR to a valid config directory.)
endif
endif

ifneq ($(CONFIG),)

ifneq ($(TARGET),)
$(error TARGET or CONFIG should be specified. Not both.)
endif

CONFIG_HEADER_FILE  = $(CONFIG_DIR)/configs/$(CONFIG)/config.h
CONFIG_SOURCE_FILE  = $(CONFIG_DIR)/configs/$(CONFIG)/config.c
INCLUDE_DIRS       += $(CONFIG_DIR)/configs/$(CONFIG)

ifneq ($(wildcard $(CONFIG_HEADER_FILE)),)

CONFIG_SRC :=
ifneq ($(wildcard $(CONFIG_SOURCE_FILE)),)
CONFIG_SRC += $(CONFIG_SOURCE_FILE)
TARGET_FLAGS += -DUSE_CONFIG_SOURCE
endif

CONFIG_REVISION := norevision
ifeq ($(shell git -C "$(CONFIG_DIR)" rev-parse --is-inside-work-tree 2>/dev/null),true)
ifeq ($(strip $(shell git -C "$(CONFIG_DIR)" status --porcelain -uno 2>/dev/null)),)
CONFIG_REVISION := $(shell git -C "$(CONFIG_DIR)" rev-parse --short=7 HEAD 2>/dev/null)
CONFIG_REVISION_DEFINE := -D'__CONFIG_REVISION__="$(CONFIG_REVISION)"'
endif
endif

# Extract constants from $(CONFIG_HEADER_FILE) via preprocessor expansion
# Oscillator frequency (MHz) -> Hz
HSE_VALUE_MHZ := $(call pp_def_value,$(CONFIG_HEADER_FILE),SYSTEM_HSE_MHZ)
ifneq ($(strip $(HSE_VALUE_MHZ)),)
HSE_VALUE     := $(shell echo $$(( $(HSE_VALUE_MHZ) * 1000000 )) )
endif

# MCU target from config header
TARGET        := $(call pp_def_value,$(CONFIG_HEADER_FILE),FC_TARGET_MCU)
ifeq ($(strip $(TARGET)),)
$(error No TARGET identified. Is the $(CONFIG_HEADER_FILE) valid for $(CONFIG)?)
endif

# Optional EXST address adjustment
EXST_ADJUST_VMA := $(call pp_def_value,$(CONFIG_HEADER_FILE),FC_VMA_ADDRESS)
ifneq ($(strip $(EXST_ADJUST_VMA)),)
EXST = yes
endif

else #exists
$(error `$(CONFIG_HEADER_FILE)` not found. Ensure the target config exists under `$(CONFIG_DIR)/configs`.)
endif #CONFIG_HEADER_FILE exists
endif #config

.PHONY: configs
configs:
ifeq ($(wildcard $(CONFIG_DIR)/configs/),)
	$(error `$(CONFIG_DIR)` not found. Expected in-tree configs under `src/config` or set CONFIG_DIR to a valid config directory.)
else
	@echo "Configs already available in $(CONFIG_DIR)"
endif

$(BASE_CONFIGS):
	@echo "Building target config $@"
	$(V0) $(MAKE) fwo CONFIG=$@
	@echo "Building target config $@ succeeded."

## <CONFIG>_rev    : build configured target and add revision to filename
$(addsuffix _rev,$(BASE_CONFIGS)):
	$(V0) $(MAKE) fwo CONFIG=$(subst _rev,,$@) REV=yes

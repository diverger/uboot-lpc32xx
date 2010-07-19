PLATFORM_CPPFLAGS += -march=armv5te 
PLATFORM_CPPFLAGS += $(call cc-option,-mtune=arm926ejs,)

PLATFORM_RELFLAGS += -fno-strict-aliasing  -fno-common -ffixed-r8

PLATFORM_RELFLAGS +=$(call cc-option,-mshort-load-bytes,$(call cc-option,-malignment-traps,))

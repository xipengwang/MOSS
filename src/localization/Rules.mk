.PHONY: moss_localization_clean moss_localization

moss_localization:  common vx lcmtypes scanmatch graph sim

moss_localization:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/localization/ -f Build.mk

moss_localization_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/localization/ -f Build.mk clean

all: moss_localization

clean: moss_localization_clean

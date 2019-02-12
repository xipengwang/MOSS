.PHONY: moss_mapping_clean moss_mapping

moss_mapping:  common vx lcmtypes scanmatch graph sim

moss_mapping:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/mapping/ -f Build.mk

moss_mapping_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/mapping/ -f Build.mk clean

all: moss_mapping

clean: moss_mapping_clean

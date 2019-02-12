.PHONY: moss_sim_manual_clean moss_sim_manual

moss_sim_manual:  common vx lcmtypes sim apriltag

moss_sim_manual:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/sim-manual/ -f Build.mk

moss_sim_manual_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/sim-manual/ -f Build.mk clean

all: moss_sim_manual

clean: moss_sim_manual_clean

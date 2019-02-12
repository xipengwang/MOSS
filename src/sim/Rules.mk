.PHONY: moss_sim_clean moss_sim

moss_sim:  common vx lcmtypes sim apriltag

moss_sim:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/sim/ -f Build.mk

moss_sim_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/sim/ -f Build.mk clean

all: moss_sim

clean: moss_sim_clean

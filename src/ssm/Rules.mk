.PHONY: ssm_clean ssm

ssm:  common vx lcmtypes sim apriltag graph scanmatch

ssm:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/ssm/ -f Build.mk

ssm_clean:
	@echo $@
	@$(MAKE) -C $(ROOT_PATH)/src/ssm/ -f Build.mk clean

all: ssm

clean: ssm_clean

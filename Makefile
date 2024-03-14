DATA_DIR=/data
SSH_PRIV_KEY=`cat ~/.ssh/id_rsa`

help: ## Help
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m %s\n", $$1, $$2}'

install_docker: ## Install Docker (Ubuntu only)
	@bash scripts/install_docker.bash

build_docker:  ## Build srl_mav_sim docker
	cd docker && ./build_docker_image.bash

run_docker: ## Run srl_mav_sim docker
	@xhost +local:docker && docker run \
		-e DISPLAY \
		-v $(DATA_DIR):$(DATA_DIR) \
		-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
		-v /dev/shm:/dev/shm \
		-v /dev/dri:/dev/dri \
		--privileged \
		--ipc=host \
		--pid=host \
		--network="host" \
		-it --rm srl_mav_sim /bin/bash

fix_px4: ## Fix PX4 in gazebo mode
	rosrun mavros mavparam set COM_RCL_EXCEPT 4  # RC LOSS EXCEPTION -> 4 (Not documented)
	rosrun mavros mavparam set NAV_DLL_ACT 0     # GCS loss failsafe mode -> 0 (Disabled)
	rosrun mavros mavparam set NAV_RCL_ACT 0     # RC loss failsafe modea -> 0 (Not documented)

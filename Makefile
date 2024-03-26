help: ## Help
	@echo -e "\033[1;34m[make targets]:\033[0m"
	@egrep -h '\s##\s' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-20s\033[0m %s\n", $$1, $$2}'

install_docker: ## Install Docker (Ubuntu only)
	@bash scripts/install_docker.bash

build_docker:  ## Build srl-mav-sim docker
	@cd docker && \
		docker build -t srl-mav-sim \
			--build-arg USERNAME=${USER} \
			--build-arg USER_ID=`id -u` \
			--build-arg GROUP_ID=`id -g` .

run_docker: ## Run srl-mav-sim docker
	@xhost +local:docker && docker run \
		-e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /tmp:/tmp \
		-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
		-v /dev/shm:/dev/shm \
		-v /dev/dri:/dev/dri \
		-v $(HOME)/.cache:$(HOME)/.cache \
		--privileged \
		--ipc=host \
		--pid=host \
		--network="host" \
		-it --rm srl-mav-sim /bin/bash

IMAGE_NAME=ros2-isaac-sim
VERSION=0.1
CONTAINER_NAME=isaac-forest-sim-ros2-jazzy
ROS_DOMAIN_ID=0
WEBRTC_STREAMING_CLIENT=isaacsim-webrtc-streaming-client-1.1.5-linux-x64.AppImage
WEBRTC_STREAMING_CLIENT_URL=https://download.isaacsim.omniverse.nvidia.com/isaacsim-webrtc-streaming-client-1.1.4-linux-x64.AppImage
XSOCK=/tmp/.X11-unix
XAUTH=$(HOME)/.Xauthority
FOREST_DEMO_OUTPUT=data/digital_twin/forest_demo.usda
FOREST_DEMO_MANIFEST=data/digital_twin/forest_demo_manifest.json
FOREST_DEMO_CACHE=data/digital_twin/forest_demo_cache.npz
FOREST_DEMO_OUTPUT_DIR=data/digital_twin
FOREST_GENERATOR_SCRIPT=script/digitwin/forest_env_demo.py
FOREST_POINTCLOUD_DIR=script/data/example_forest_pointcloud
FOREST_CANOPY_ASSET_DIR=$(HOME)/isaac-sim/plan/canopy_assets
GROUND_POINTCLOUD=script/data/example_forest_pointcloud/porvoo-20250520-000013_ground_sec.ply

build-isaac-sim-ros2-jazzy:
	docker build -t $(IMAGE_NAME):$(VERSION) -f Dockerfile .

attach-running-container:
	docker exec -it $(CONTAINER_NAME) bash -lc "export ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) && source /opt/ros/jazzy/setup.bash && exec bash"


run-sim-ros2-jazzy-container:
	echo "Running container $(CONTAINER_NAME)..."
	xhost +si:localuser:root
	@if docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; then \
		echo "Container $(CONTAINER_NAME) already exists."; \
		if [ "$$(docker inspect -f '{{.State.Running}}' $(CONTAINER_NAME))" != "true" ]; then \
			echo "Starting existing container $(CONTAINER_NAME)..."; \
			docker start $(CONTAINER_NAME); \
		fi; \
	else \
		echo "Running new container $(CONTAINER_NAME) in background..."; \
		docker run -d --name $(CONTAINER_NAME) \
			--entrypoint bash \
			--user root \
			--gpus all \
			--network=host \
			-e DISPLAY=$$DISPLAY \
			-e XAUTHORITY=/root/.Xauthority \
			-e QT_X11_NO_MITSHM=1 \
			-e ACCEPT_EULA=Y \
			-e PRIVACY_CONSENT=Y \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
			-v $(XSOCK):$(XSOCK):rw \
			-v $(XAUTH):/root/.Xauthority:ro \
			-v $$HOME/IsaacForestSim/isaac-sim/cache/main:/isaac-sim/.cache:rw \
			-v $$HOME/IsaacForestSim/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
			-v $$HOME/IsaacForestSim/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
			-v $$HOME/IsaacForestSim/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
			-v $$HOME/IsaacForestSim/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
			-v $$HOME/IsaacForestSim/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
			-v $$HOME/IsaacForestSim/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
			-v $$HOME/IsaacForestSim/data/digital_twin:/isaac-sim/.local/share/ov/digital_twin:rw \
			$(IMAGE_NAME):$(VERSION) \
			-lc "tail -f /dev/null"; \
	fi; \
	echo "Starting Isaac Sim headless..."; \
	docker exec -it $(CONTAINER_NAME) bash -lc "export ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) && source /opt/ros/jazzy/setup.bash && cd /isaac-sim && ./runheadless.sh"


run-webrtc-streaming-client:
	@if [ ! -f ./$(WEBRTC_STREAMING_CLIENT) ]; then \
		echo "Downloading $(WEBRTC_STREAMING_CLIENT) ..."; \
		curl -fL "$(WEBRTC_STREAMING_CLIENT_URL)" -o ./$(WEBRTC_STREAMING_CLIENT); \
		chmod +x ./$(WEBRTC_STREAMING_CLIENT); \
	fi
	./$(WEBRTC_STREAMING_CLIENT)

run-rviz:
	docker exec -it $(CONTAINER_NAME) bash -lc "export ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) && source /opt/ros/jazzy/setup.bash && rviz2"

run-teleop:
	docker exec -it $(CONTAINER_NAME) bash -lc "export ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) && source /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

run-forest-usda-from-pointcloud:
	@set -e; \
	if ! docker container inspect $(CONTAINER_NAME) >/dev/null 2>&1; then \
		echo "Container $(CONTAINER_NAME) does not exist. Start it first with: make run-sim-ros2-jazzy-container"; \
		exit 1; \
	fi; \
	if [ "$$(docker inspect -f '{{.State.Running}}' $(CONTAINER_NAME))" != "true" ]; then \
		echo "Starting existing container $(CONTAINER_NAME)..."; \
		docker start $(CONTAINER_NAME) >/dev/null; \
	fi; \
	mkdir -p $(FOREST_DEMO_OUTPUT_DIR); \
	echo "Copying point cloud inputs into $(CONTAINER_NAME)..."; \
	docker cp $(FOREST_GENERATOR_SCRIPT) $(CONTAINER_NAME):/tmp/forest_env_demo.py; \
	docker cp $(FOREST_POINTCLOUD_DIR) $(CONTAINER_NAME):/tmp/forest_pointcloud; \
	CANOPY_ARG=""; \
	if [ -d "$(FOREST_CANOPY_ASSET_DIR)" ] && [ -f "$(FOREST_CANOPY_ASSET_DIR)/pine_tree.usd" ]; then \
		echo "Copying canopy assets from $(FOREST_CANOPY_ASSET_DIR)..."; \
		docker cp "$(FOREST_CANOPY_ASSET_DIR)" $(CONTAINER_NAME):/tmp/canopy_assets; \
		CANOPY_ARG="--canopy-prototype /tmp/canopy_assets/pine_tree.usd"; \
	fi; \
	echo "Generating $(FOREST_DEMO_OUTPUT_DIR)..."; \
	docker exec $(CONTAINER_NAME) bash -lc "cd /isaac-sim/.local/share/ov/pkg && /isaac-sim/python.sh /tmp/forest_env_demo.py --source-dir /tmp/forest_pointcloud --ground porvoo-20250520-000013_ground_sec.ply --trunk porvoo-20250520-000013_trunk_sec.ply --canopy porvoo-20250520-000013_canopy_sec.ply --output-dir /tmp/forest_digital_twin --ground-resolution 0.5 --ground-padding 4.0 --ground-max-dim 2048 --ground-voxel-size 0.2 --ground-clip-low-q 1.0 --ground-clip-high-q 99.5 --ground-smoothing-sigma 0.5 --ground-thickness 0.5 --ground-collider-approximation sdf --add-canopy $$CANOPY_ARG"; \
	docker cp $(CONTAINER_NAME):/tmp/forest_digital_twin/. $(FOREST_DEMO_OUTPUT_DIR)/

run-forest-demo:
	@if [ ! -f $(FOREST_DEMO_OUTPUT) ]; then \
		echo "Missing $(FOREST_DEMO_OUTPUT)."; \
		exit 1; \
	fi
	@if [ ! -f $(FOREST_DEMO_MANIFEST) ]; then \
		echo "Missing $(FOREST_DEMO_MANIFEST)."; \
		exit 1; \
	fi
	@echo "Ready for manual import:"; \
	echo "  $(FOREST_DEMO_OUTPUT)"; \
	echo "  $(FOREST_DEMO_MANIFEST)"

stop-container:
	docker stop $(CONTAINER_NAME)

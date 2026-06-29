RUN_ARGS := "--ipc=host"
HEADLESS ?= false

DOCKER_IMAGE := jeyakaranm/autonomy_dev:humble
CONTAINER_NAME := autonomy_prod
COMPOSE_PROD := docker-compose-prod.yml
SOURCE_WS := source /opt/ros/humble/setup.bash && source /root/install/setup.bash
LAUNCH_ARGS := headless:=$(HEADLESS)

# =====================================================
# Docker Build
# =====================================================

.PHONY: build-docker
build-docker:
	@echo "Building Docker image..."
	docker build -t $(DOCKER_IMAGE) .

# =====================================================
# Production (built image, no source mount)
# =====================================================

.PHONY: run-prod
run-prod: rm-prod
	@echo "Starting production container..."
	docker compose -f $(COMPOSE_PROD) up -d

.PHONY: exec-prod
exec-prod:
	@echo "Exec into production container..."
	docker exec -it $(CONTAINER_NAME) bash

.PHONY: rm-prod
rm-prod:
	@echo "Removing production container..."
	@docker kill $(CONTAINER_NAME) 2>/dev/null || true
	@docker rm $(CONTAINER_NAME) 2>/dev/null || true


.PHONY: kill-gz
kill-gz:
	@echo "Killing hanging Gazebo processes..."
	@docker exec $(CONTAINER_NAME) bash -c "killall -9 gzserver gzclient 2>/dev/null || true"

.PHONY: clean
clean: rm-prod
	@echo "All containers removed."

# =====================================================
# LiDAR Docking (run these from the HOST; exec into the container)
# =====================================================
DOCK_DEMO := cd /root/src/autonomy_dev && ./scripts/run_demo.sh

.PHONY: run-baseline-docking
run-baseline-docking: rm-prod
	@echo "Baseline docking (manual start; call run-start-docking to begin)..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) baseline --rviz"

.PHONY: run-baseline-docking-autostart
run-baseline-docking-autostart: rm-prod
	@echo "Baseline docking (auto-start after 20s settle)..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) baseline --rviz --autostart"

.PHONY: run-start-docking
run-start-docking:
	@echo "Triggering docking start service..."
	docker exec $(CONTAINER_NAME) bash -c \
		"$(SOURCE_WS) && ros2 service call /docking/start std_srvs/srv/Trigger"

.PHONY: run-publish-docking-status
run-publish-docking-status:
	@echo "Echoing docking status..."
	docker exec $(CONTAINER_NAME) bash -c \
		"$(SOURCE_WS) && ros2 topic echo /docking/status"

.PHONY: run-disturbed-docking-case1
run-disturbed-docking-case1: rm-prod
	@echo "Disturbed docking case 1 (corruption; manual start)..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) disturbed --case 1 --rviz"

.PHONY: run-disturbed-docking-case1-autostart
run-disturbed-docking-case1-autostart: rm-prod
	@echo "Disturbed docking case 1 (corruption; auto-start after 20s settle)..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) disturbed --case 1 --rviz --autostart"

.PHONY: run-disturbed-docking-case2
run-disturbed-docking-case2: rm-prod
	@echo "Disturbed docking case 2 (scan dropout; manual start)..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) disturbed --case 2 --rviz"

.PHONY: run-disturbed-docking-case2-autostart
run-disturbed-docking-case2-autostart: rm-prod
	@echo "Disturbed docking case 2 (scan dropout; auto-start after 20s settle)..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) disturbed --case 2 --rviz --autostart"

.PHONY: dock-eval
dock-eval: rm-prod
	@echo "Multi-trial evaluation (baseline; TRIALS=$(or $(TRIALS),10))..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) baseline --eval $(or $(TRIALS),10)"

.PHONY: dock-eval-case1
dock-eval-case1: rm-prod
	@echo "Multi-trial evaluation (case 1 corruption; TRIALS=$(or $(TRIALS),10))..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) disturbed --case 1 --eval $(or $(TRIALS),10)"

.PHONY: dock-eval-case2
dock-eval-case2: rm-prod
	@echo "Multi-trial evaluation (case 2 dropout; TRIALS=$(or $(TRIALS),10))..."
	docker compose -f $(COMPOSE_PROD) run --name $(CONTAINER_NAME) --rm \
		$(CONTAINER_NAME) \
		bash -c "$(SOURCE_WS) && $(DOCK_DEMO) disturbed --case 2 --eval $(or $(TRIALS),10)"



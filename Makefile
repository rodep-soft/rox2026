.PHONY: dev sync

dev:
	docker compose up -d
	docker compose exec ros2_rox2026 bash

sync:
	git submodule update --init --recursive



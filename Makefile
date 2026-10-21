.DEFAULT_GOAL := help
PYTHON ?= .venv/bin/python
PLATFORM ?= cf2
.PHONY: help setup test firmware simulate mission clean
help:
	@echo 'setup     repository-local Python dependencies'
	@echo 'test      planner, transport-independent mission, and native C checks'
	@echo 'firmware  build OOT app (PLATFORM=cf2 or cf21bl); never flashes'
	@echo 'simulate  solve and independently integrate nominal SCvx descent'
	@echo 'mission   dry-run takeoff/descent mission; --execute is explicit in CLI'
	@echo 'clean     remove generated app builds and simulation output'
setup:
	uv sync --extra dev
test:
	$(PYTHON) -m pytest -q
firmware:
	./scripts/build_firmware.sh $(PLATFORM)
simulate:
	$(PYTHON) -m pdg simulate --height 1.0
mission:
	$(PYTHON) -m pdg mission --height 1.0
clean:
	rm -rf apps/pdg_controller/build output
	find apps/pdg_controller -type f \( -name '*.o' -o -name '.*.cmd' -o -name '*.o.d' \) -delete

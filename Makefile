.PHONY: help venv test lint preflight link calib imu vision rc override loop clean

PY      ?= python3
VENV    ?= .venv
VPY      = $(VENV)/bin/python
BRINGUP  = $(PY) -m tools.bringup

help:
	@echo "setup"
	@echo "  make venv        create $(VENV) (shares the system numpy/pyserial)"
	@echo "  make test        run the offline test suite (no hardware)"
	@echo "  make lint        byte-compile everything"
	@echo ""
	@echo "bring-up, in order — PROPS OFF for all of it"
	@echo "  make preflight   who is on the port, what is configured"
	@echo "  make link        FC telemetry"
	@echo "  make calib       measure IMU scale + axis map (you move the airframe)"
	@echo "  make imu         verify the (q, w, accel) contract"
	@echo "  make vision      verify bearings from the tracker"
	@echo "  make rc          stream idle RC"
	@echo "  make override    guided takeover / handback / dead-man test"
	@echo "  make loop        full plumbing loop with the idle command module"
	@echo ""
	@echo "  make loop-fake   the whole loop with no hardware at all"

venv:
	$(PY) -m venv --system-site-packages $(VENV)
	$(VPY) -m pip install -q --upgrade pip
	$(VPY) -m pip install -q -r requirements.txt
	@echo "activate with: source $(VENV)/bin/activate"

test:
	$(PY) -m unittest discover -s tests -v

lint:
	$(PY) -m compileall -q companion tools tests && echo "compile OK"

preflight: ; $(BRINGUP) preflight
link:      ; $(BRINGUP) link
calib:     ; $(BRINGUP) calib-imu
imu:       ; $(BRINGUP) imu
vision:    ; $(BRINGUP) vision
rc:        ; $(BRINGUP) rc
override:  ; $(BRINGUP) override
loop:      ; $(BRINGUP) loop

loop-fake:
	$(BRINGUP) loop --fake-imu --fake-vision --no-fc

clean:
	rm -rf $(VENV) **/__pycache__ .pytest_cache
	find . -name '__pycache__' -type d -exec rm -rf {} +

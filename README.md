# simbricks

A drop-in simulator for Pybricks MicroPython, re-implementing the Pybricks API on top of a small physics core, so unmodified robot scripts run against a virtual robot on computer. Because the same script also runs on a real LEGO hub, it supports a simulate-then-deploy workflow, and closing the sim-to-real gap through calibration against logged hardware data is a future goal.
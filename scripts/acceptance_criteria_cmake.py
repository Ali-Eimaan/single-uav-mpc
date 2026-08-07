#!/usr/bin/env python3
"""Emit the C++-bound acceptance criteria as KEY=VALUE lines for CMake configure_file.

The single source of truth is acceptance_criteria.yaml at the repo root ():
the SITL assertions read it directly, and the C++ solve-time budgets (A2) are substituted
into test/acceptance_criteria.h.in at configure time via this script. If a number is edited
in the YAML, both consumers pick it up; CMake fails loudly when this script cannot run.

Usage (from CMake):
    execute_process(COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/acceptance_criteria_cmake.py
        OUTPUT_VARIABLE ACCEPTANCE_LINES ...)
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
CRIT = yaml.safe_load((REPO_ROOT / "acceptance_criteria.yaml").read_text())

lines = [
    f"A2_SOLVE_P99_MS={CRIT['a2_solve_time_ms']['p99']}",
    f"A2_SOLVE_MEDIAN_MS={CRIT['a2_solve_time_ms']['median']}",
]
sys.stdout.write("\n".join(lines) + "\n")

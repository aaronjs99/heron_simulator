#!/usr/bin/env python3
import os
import re
import unittest


TEST_DIR = os.path.dirname(__file__)
REPO_ROOT = os.path.abspath(os.path.join(TEST_DIR, "..", ".."))
BENCHMARK_CONFIG = os.path.join(
    REPO_ROOT,
    "heron",
    "heron_description",
    "urdf",
    "configs",
    "ig_handle_benchmark",
)


class BenchmarkHullProfileTests(unittest.TestCase):
    def test_ig_handle_benchmark_uses_deeper_draft_mass_profile(self):
        with open(BENCHMARK_CONFIG, "r", encoding="utf-8") as handle:
            content = handle.read()

        self.assertRegex(content, r"export HERON_BASE_MASS=42\.0\b")
        self.assertRegex(content, r"export HERON_BASE_INERTIA_IXX=3\.8\b")
        self.assertRegex(content, r"export HERON_BASE_INERTIA_IYY=6\.6\b")
        self.assertRegex(content, r"export HERON_BASE_INERTIA_IZZ=10\.0\b")


if __name__ == "__main__":
    unittest.main()

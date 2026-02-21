#!/usr/bin/env bash

set -euo pipefail

cargo fmt --all
ruff format
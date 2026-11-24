#!/usr/bin/env bash
set -euo pipefail
unset WINE_MSVC_RAW_STDOUT
exec mt "$@"

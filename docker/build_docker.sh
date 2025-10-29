#!/usr/bin/env bash
set -euo pipefail

# locate script dir and default context (one level above script dir)
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
context_dir="$(dirname "$script_dir")"
dockerfile="$script_dir/Dockerfile"
image_tag="srl-mav-sim-image"

docker build -f "$dockerfile" -t "$image_tag" "$context_dir"
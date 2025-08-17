#!/usr/bin/env bash
set -euo pipefail
mkdir -p dist
for d in template_projects/*; do
  [[ -d "$d" ]] || continue
  name="$(basename "$d")"
  zip -r "dist/${name}.zip" "$d" \
    -x "**/.git/**" "**/.gradle/**" "**/build/**"
done
ls -lh dist
#!/usr/bin/env bash
# Build docs/api_reference.pdf from docs/api_reference.md.
# This script is the single home of the PDF build options; the docs-pdf
# GitHub workflow runs it, and it can be run locally from any directory.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

stderr_file="$(mktemp)"
trap 'rm -f "$stderr_file"' EXIT

rc=0
pandoc docs/api_reference.md -o docs/api_reference.pdf \
  --pdf-engine=xelatex \
  -V mainfont='DejaVu Serif' \
  -V monofont='DejaVu Sans Mono' \
  2>"$stderr_file" || rc=$?
cat "$stderr_file" >&2

if [ "$rc" -ne 0 ]; then
  echo "build_pdf.sh: pandoc failed with exit status $rc" >&2
  exit "$rc"
fi

# pandoc exits 0 when a font lacks a glyph; treat that warning as a failure.
if grep -q 'Missing character' "$stderr_file"; then
  echo "build_pdf.sh: pandoc reported missing characters (see above); the PDF would lack those glyphs" >&2
  exit 1
fi

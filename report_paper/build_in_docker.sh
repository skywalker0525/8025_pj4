#!/usr/bin/env bash
set -euo pipefail

mkdir -p output figures

latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=output main.tex

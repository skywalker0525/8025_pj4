#!/usr/bin/env bash
set -euo pipefail

mkdir -p output figures

if [ -f "media/mission.webm" ]; then
  ffmpeg -y -ss 00:00:05 -i media/mission.webm -frames:v 1 -vf "scale=1280:-1" figures/video_mission_05.png >/tmp/ffmpeg_05.log 2>&1 || true
  ffmpeg -y -ss 00:00:15 -i media/mission.webm -frames:v 1 -vf "scale=1280:-1" figures/video_mission_15.png >/tmp/ffmpeg_15.log 2>&1 || true
  ffmpeg -y -ss 00:00:30 -i media/mission.webm -frames:v 1 -vf "scale=1280:-1" figures/video_mission_30.png >/tmp/ffmpeg_30.log 2>&1 || true
fi

latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir=output main.tex

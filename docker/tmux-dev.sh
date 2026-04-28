#!/bin/bash
set -e

SESSION_NAME="${1:-dev}"

exec tmux new-session -A -s "${SESSION_NAME}"

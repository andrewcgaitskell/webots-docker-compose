#!/bin/zsh
#
# aider-ollama.sh
# Launches Aider configured against a local Ollama model, without needing
# to manually export OLLAMA_API_BASE or activate the venv each time.
#
# Usage:
#   ./aider-ollama.sh                 # uses default model below
#   ./aider-ollama.sh <model-name>    # override model, e.g. ./aider-ollama.sh qwen3.5:9b-q4_k_m
#
# Optional: symlink or alias this somewhere on PATH, e.g.
#   ln -s ~/scripts/aider-ollama.sh /opt/homebrew/bin/aider-ollama

set -euo pipefail

# --- Config: adjust these paths for your setup ---
VENV_PATH="$HOME/venvs/aider"
DEFAULT_MODEL="qwen3.5:9b-q4_k_m"
OLLAMA_HOST="http://127.0.0.1:11434"

# --- Resolve model (arg1 overrides default) ---
MODEL="${1:-$DEFAULT_MODEL}"

# --- Env vars Aider/litellm need to reach Ollama ---
export OLLAMA_API_BASE="$OLLAMA_HOST"

# --- Check Ollama is actually up before bothering to launch Aider ---
if ! curl -s --max-time 2 "${OLLAMA_HOST}/api/tags" > /dev/null; then
  echo "Warning: Ollama doesn't seem to be running at ${OLLAMA_HOST}."
  echo "Start it in another terminal with: ollama serve"
  read -q "REPLY?Continue anyway? (y/n) "
  echo
  if [[ "$REPLY" != "y" ]]; then
    exit 1
  fi
fi

# --- Activate venv ---
if [[ -f "${VENV_PATH}/bin/activate" ]]; then
  source "${VENV_PATH}/bin/activate"
else
  echo "Error: venv not found at ${VENV_PATH}"
  echo "Edit VENV_PATH in this script or create the venv first."
  exit 1
fi

echo "Launching Aider with model: ollama/${MODEL}"
aider --model "ollama/${MODEL}" "${@:2}"


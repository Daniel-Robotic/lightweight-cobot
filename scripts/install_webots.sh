#!/bin/bash
# Download and install the Webots simulator from the Cyberbotics GitHub release.
# Emits PROGRESS:<pct>:<label> lines so the Python caller can update its progress bar.
# Скачивает и устанавливает симулятор Webots из релизов GitHub Cyberbotics.
# Выводит строки PROGRESS:<pct>:<метка> для обновления прогресс-бара в Python.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

PROGRESS() { echo "PROGRESS:$1:$2"; }

WEBOTS_VERSION="2025a"
DEB_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb"
TMP_DIR="$(mktemp -d)"
DEB_PATH="${TMP_DIR}/webots_${WEBOTS_VERSION}_amd64.deb"

cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT

PROGRESS 5 "Downloading Webots ${WEBOTS_VERSION}..."
echo "Downloading Webots ${WEBOTS_VERSION}..."
echo "URL: ${DEB_URL}"

# wget writes download progress to stderr; redirect to stdout so it appears in the TUI log.
wget --progress=dot:mega -O "$DEB_PATH" "$DEB_URL" 2>&1

PROGRESS 70 "Installing Webots package..."
echo "Download complete. Installing..."

sudo apt-get install -y "$DEB_PATH"

PROGRESS 100 "Done"
echo "Webots ${WEBOTS_VERSION} installed successfully."

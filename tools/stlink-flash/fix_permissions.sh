#!/usr/bin/env bash
# Allow flashing with an ST-Link without root: installs a udev rule (needs sudo, run once).
set -euo pipefail

RULE_FILE=/etc/udev/rules.d/49-stlink.rules
# ST-Link V2, V2-1, V3 (all variants share the STMicroelectronics vendor id)
RULE='SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3744|3748|374b|374d|374e|374f|3752|3753|3754|3755|3757", MODE="0660", GROUP="plugdev", TAG+="uaccess"'

if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

echo "${RULE}" > "${RULE_FILE}"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=0483

if [ -n "${SUDO_USER:-}" ] && ! id -nG "${SUDO_USER}" | tr ' ' '\n' | grep -qx plugdev; then
    usermod -aG plugdev "${SUDO_USER}"
    echo "Added ${SUDO_USER} to group plugdev: log out and back in for it to apply."
fi

echo "Installed ${RULE_FILE}. Unplug and replug the ST-Link."

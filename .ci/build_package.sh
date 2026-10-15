#!/usr/bin/env bash
# Entry point following the workspace .ci/build_package.sh artifact convention.
set -euo pipefail
wifi_ci=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
wifi_output=${1:?Usage: .ci/build_package.sh ARTIFACTS_FOLDER}
# CI images and source builders need the distribution supplicant's build headers.
# Install missing tools once; the library and package builds run as the caller.
wifi_missing=()
for wifi_dependency in build-essential binutils cmake git perl dpkg-dev pkg-config patch \
	libssl-dev libdbus-1-dev libnl-3-dev libnl-genl-3-dev libnl-route-3-dev \
	libpcsclite-dev libreadline-dev wpasupplicant; do
	if [[ $(dpkg-query -W -f='${db:Status-Status}' "$wifi_dependency" 2>/dev/null || true) != installed ]]; then
		wifi_missing+=("$wifi_dependency")
	fi
done
if ((${#wifi_missing[@]})); then
	wifi_admin=()
	if ((EUID != 0)); then wifi_admin=(sudo); fi
	"${wifi_admin[@]}" apt-get update || printf 'Trying the available authenticated package indexes.\n' >&2
	"${wifi_admin[@]}" apt-get install -y --no-install-recommends "${wifi_missing[@]}"
fi
bash "$wifi_ci/build_support_package.sh" "$wifi_output"

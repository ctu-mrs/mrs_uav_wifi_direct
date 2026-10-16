#!/usr/bin/env bash
# Build the Wi-Fi Direct support deb from released MsQuic sources as a normal user.
# Select the MsQuic release here, or set MSQUIC_VERSION in the build environment.
MSQUIC_VERSION=${MSQUIC_VERSION:-2.6.3}
SUPPORT_PACKAGE=mrs-uav-wifi-direct-support
set -euo pipefail
umask 022

wifi_arch=$(dpkg --print-architecture)
wifi_multiarch=$(dpkg-architecture -qDEB_HOST_MULTIARCH)
wifi_version=${MSQUIC_VERSION}-mrs11
wifi_ci=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
wifi_output=${1:?Usage: .ci/build_support_package.sh ARTIFACTS_FOLDER}
wifi_jobs=${MRS_BUILD_JOBS:-$(nproc)}
mkdir -p "$wifi_output"
wifi_output=$(cd "$wifi_output" && pwd)
wifi_work=$(mktemp -d /tmp/mrs-wifi-direct-support.XXXXXX)
trap 'rm -rf "$wifi_work"' EXIT
wifi_source=$wifi_work/source
wifi_build=$wifi_work/build
wifi_root=$wifi_work/package

# Fetch the release together with its matching TLS source.
git clone --quiet --depth 1 --branch "v$MSQUIC_VERSION" https://github.com/microsoft/msquic.git "$wifi_source"
git -C "$wifi_source" submodule update --init --depth 1 submodules/quictls
cmake -S "$wifi_source" -B "$wifi_build" \
	-DCMAKE_BUILD_TYPE=Release \
	-DQUIC_TLS_LIB=quictls \
	-DQUIC_USE_SYSTEM_LIBCRYPTO=OFF \
	-DQUIC_BUILD_SHARED=ON \
	-DQUIC_BUILD_TEST=OFF \
	-DQUIC_BUILD_PERF=OFF \
	-DQUIC_BUILD_TOOLS=OFF \
	-DQUIC_ENABLE_LOGGING=OFF
cmake --build "$wifi_build" --parallel "$wifi_jobs"

# Include the public shared library and its matching portable development headers.
mkdir -p "$wifi_root/DEBIAN" "$wifi_root/usr/lib/$wifi_multiarch/pkgconfig" "$wifi_root/usr/include/msquic" "$wifi_root/usr/share/doc/$SUPPORT_PACKAGE"
wifi_library=$(find "$wifi_build/bin" -name "libmsquic.so.$MSQUIC_VERSION" -type f -print -quit)
test -n "$wifi_library"
install -m 755 "$wifi_library" "$wifi_root/usr/lib/$wifi_multiarch/"
strip --strip-unneeded "$wifi_root/usr/lib/$wifi_multiarch/libmsquic.so.$MSQUIC_VERSION"
ln -s "libmsquic.so.$MSQUIC_VERSION" "$wifi_root/usr/lib/$wifi_multiarch/libmsquic.so.2"
ln -s libmsquic.so.2 "$wifi_root/usr/lib/$wifi_multiarch/libmsquic.so"
for wifi_header in msquic.h msquic_posix.h quic_sal_stub.h; do
	install -m 644 "$wifi_source/src/inc/$wifi_header" "$wifi_root/usr/include/msquic/"
done
install -m 644 "$wifi_source/LICENSE" "$wifi_root/usr/share/doc/$SUPPORT_PACKAGE/copyright"
install -m 644 "$wifi_source/submodules/quictls/LICENSE.txt" "$wifi_root/usr/share/doc/$SUPPORT_PACKAGE/quictls-copyright"

# The ROS process updates one runtime drop-in; networkd remains the address manager.
install -d "$wifi_root/usr/lib/tmpfiles.d"
cat >"$wifi_root/usr/lib/tmpfiles.d/mrs-uav-wifi-direct.conf" <<'TMPFILES'
d /run/systemd/network/10-mrs-wifi-direct.network.d 0755 root root - -
f /run/systemd/network/10-mrs-wifi-direct.network.d/50-address.conf 0664 root netdev - -
TMPFILES
{
	git -C "$wifi_source" rev-parse HEAD
	git -C "$wifi_source" submodule status submodules/quictls
} >"$wifi_root/usr/share/doc/$SUPPORT_PACKAGE/source-revisions"

# DKMS keeps the kernel build and removal steps with the module it manages.
wifi_dkms=$wifi_root/usr/src/$SUPPORT_PACKAGE-$wifi_version
install -d "$wifi_dkms" "$wifi_root/usr/sbin"
cat >"$wifi_dkms/source.sh" <<'SOURCE'
#!/usr/bin/env bash
# Download the exact source version using the system's trusted apt repositories.
fetch_source() {
	local package=$1 version=$2 destination=$3 work source_file
	[[ $package =~ ^[a-z0-9][a-z0-9.+-]*$ && $version =~ ^[A-Za-z0-9.+:~_-]+$ ]] || return 1
	work=$(mktemp -d)
	mkdir -p "$work/repos" "$work/lists/partial"
	local options=(-o "Dir::Etc::sourceparts=$work/repos" -o Dir::Etc::sourcelist=/dev/null
		-o "Dir::State::lists=$work/lists" -o "Dir::Cache::pkgcache=$work/pkgcache.bin"
		-o "Dir::Cache::srcpkgcache=$work/srcpkgcache.bin" -o APT::Get::AllowUnauthenticated=false)
	# Copy repository URLs and signing settings into a temporary source-only list.
	# The system's apt configuration stays available for ordinary package updates.
	for source_file in /etc/apt/sources.list /etc/apt/sources.list.d/*.list; do
		[[ -f $source_file ]] || continue
		sed -En '/^[[:space:]]*deb(-src)?[[:space:]]/ { /cdrom:/! s/^[[:space:]]*deb(-src)?[[:space:]]+/deb-src /p; }' "$source_file" >>"$work/repos/sources.list"
	done
	for source_file in /etc/apt/sources.list.d/*.sources; do
		[[ -f $source_file ]] || continue
		# Ubuntu's introductory comments also mention Types:. Change the field
		# at the start of a line, preserving comments and repository signing keys.
		awk 'BEGIN { RS=""; ORS="\n\n" }
			$0 !~ /(^|\n)Enabled:[ \t]*no/ && $0 !~ /cdrom:/ {
				if ($0 ~ /(^|\n)Types:/) print
			}' "$source_file" | sed 's/^Types:.*/Types: deb-src/' >"$work/repos/$(basename "$source_file")"
	done
	apt-get "${options[@]}" update || printf 'Some source repositories are unavailable; trying the authenticated indexes that were downloaded.\n' >&2
	if ! (cd "$work" && apt-get "${options[@]}" source --download-only --only-source "$package=$version"); then
		rm -rf -- "$work"
		return 1
	fi
	local descriptions=("$work"/*.dsc)
	if [[ ${#descriptions[@]} != 1 || ! -f ${descriptions[0]} ]] ||
		! dpkg-source -x "${descriptions[0]}" "$destination"; then
		rm -rf -- "$work"
		return 1
	fi
	rm -rf -- "$work"
}
SOURCE

cat >"$wifi_dkms/dkms.conf" <<'DKMS'
PACKAGE_NAME="mrs-uav-wifi-direct-support"
PACKAGE_VERSION="@VERSION@"
BUILT_MODULE_NAME[0]="cfg80211"
BUILT_MODULE_LOCATION[0]="wireless"
DEST_MODULE_LOCATION[0]="/updates/dkms"
AUTOINSTALL="yes"
MAKE[0]="bash build.sh $kernelver $kernel_source_dir"
CLEAN="rm -rf wireless source"
PRE_INSTALL="check-signing.sh $kernelver $dkms_tree/$PACKAGE_NAME/$PACKAGE_VERSION/$kernelver/$arch/module"
POST_INSTALL="refresh.sh $kernelver"
POST_REMOVE="refresh.sh $kernelver"
# Intel radios need this option on kernels that disable concurrent GO support.
# Radios with native support keep their distribution module.
wifi_needed=false
for wifi_driver in /sys/class/net/*/device/driver; do
	[[ $(basename "$(readlink -f "$wifi_driver")") != iwlwifi ]] || wifi_needed=true
done
if [[ $wifi_needed == false || ! -f $kernel_source_dir/.config ]] ||
	! grep -qx 'CONFIG_CFG80211=m' "$kernel_source_dir/.config" ||
	grep -qx 'CONFIG_CFG80211_REG_RELAX_NO_IR=y' "$kernel_source_dir/.config" ||
	[[ $kernelver != "$(uname -r)" && ! -e /boot/vmlinuz-$kernelver ]]; then
	BUILD_EXCLUSIVE_KERNEL='a^'
	# DKMS checks AUTOINSTALL before trying a build. Empty also works with
	# versions that treat any nonempty value, including "no", as enabled.
	AUTOINSTALL=""
fi
DKMS
sed -i "s/@VERSION@/$wifi_version/g" "$wifi_dkms/dkms.conf"

cat >"$wifi_dkms/build.sh" <<'BUILD'
#!/usr/bin/env bash
# Build cfg80211 from the target kernel's distribution source and check its ABI.
set -euo pipefail
wifi_kernel=${1:?Kernel release required}
wifi_headers=${2:?Kernel headers required}
[[ $wifi_kernel =~ ^[A-Za-z0-9.+_~-]+$ && -f $wifi_headers/Module.symvers ]] || exit 1
source "$(dirname -- "$0")/source.sh"
wifi_package='' wifi_source_version=''
for wifi_image in "linux-modules-$wifi_kernel" "linux-image-unsigned-$wifi_kernel" "linux-image-$wifi_kernel"; do
	if wifi_identity=$(dpkg-query -W -f='${source:Package} ${source:Version}' "$wifi_image" 2>/dev/null); then
		read -r wifi_package wifi_source_version <<<"$wifi_identity"
		[[ $wifi_package == *signed* ]] || break
	fi
done
[[ -n $wifi_package && $wifi_package != *signed* ]] || { echo "Matching packaged kernel sources are required." >&2; exit 1; }
fetch_source "$wifi_package" "$wifi_source_version" "$PWD/source"
grep -q CONFIG_CFG80211_REG_RELAX_NO_IR source/net/wireless/chan.c
grep -q REGULATORY_ENABLE_RELAX_NO_IR source/net/wireless/chan.c
cp -a source/net/wireless wireless
rm -rf source
# Enable the existing kernel option while keeping its channel and driver checks.
cat >>wireless/Makefile <<'MAKE'
ccflags-y += -DCONFIG_CFG80211_REG_RELAX_NO_IR=1
MAKE
cat >>wireless/core.c <<'CORE'
/* Let the ROS node check whether this build is loaded. */
static bool mrs_wifi_direct_relax_no_ir = true;
module_param(mrs_wifi_direct_relax_no_ir, bool, 0444);
MODULE_PARM_DESC(mrs_wifi_direct_relax_no_ir, "MRS concurrent GO support enabled");
MODULE_INFO(mrs_wifi_direct_support, "1");
CORE
wifi_compiler=()
if grep -qx CONFIG_CC_IS_CLANG=y "$wifi_headers/.config"; then
	wifi_major=$(sed -n 's/^CONFIG_CLANG_VERSION=//p' "$wifi_headers/.config")
	wifi_compiler=("LLVM=-$((wifi_major / 10000))")
else
	wifi_major=$(sed -n 's/^CONFIG_GCC_VERSION=//p' "$wifi_headers/.config")
	wifi_compiler=("CC=gcc-$((wifi_major / 10000))")
fi
nice -n 19 make -C "$wifi_headers" "M=$PWD/wireless" "${wifi_compiler[@]}" -j"${MRS_BUILD_JOBS:-4}" modules
wifi_original=(/lib/modules/"$wifi_kernel"/kernel/net/wireless/cfg80211.ko*)
[[ ${#wifi_original[@]} == 1 && -f ${wifi_original[0]} ]]
[[ $(modinfo -F vermagic wireless/cfg80211.ko) == "$(modinfo -F vermagic "${wifi_original[0]}")" ]]
awk '$3 ~ /(^|\/)cfg80211$/ {print $1, $2}' "$wifi_headers/Module.symvers" | sort >expected.exports
awk '$3 ~ /(^|\/)cfg80211$/ {print $1, $2}' wireless/Module.symvers | sort >built.exports
[[ -s expected.exports ]]
cmp expected.exports built.exports
modprobe --show-modversions wireless/cfg80211.ko >imports
awk 'function crc(v) {sub(/^0x0*/, "", v); return tolower(v)}
	NR==FNR {known[$2]=crc($1); next}
	!($2 in known) || known[$2]!=crc($1) {bad=1}
	END {exit bad}' "$wifi_headers/Module.symvers" imports
BUILD

cat >"$wifi_dkms/check-signing.sh" <<'SIGNING'
#!/usr/bin/env bash
# DKMS signs its module. Check trust before allowing it to replace a boot module.
set -euo pipefail
wifi_kernel=${1:?Kernel release required}
wifi_modules=${2:?Built module directory required}
if grep -qx CONFIG_MODULE_SIG_FORCE=y "/lib/modules/$wifi_kernel/build/.config"; then
	echo "This kernel requires a trusted built-in signing key; keeping its distribution module." >&2
	exit 1
fi
if [[ ! -d /sys/firmware/efi ]]; then
	if [[ -f /sys/kernel/security/lockdown ]] && grep -Eq '\[(integrity|confidentiality)\]' /sys/kernel/security/lockdown; then
		echo "Kernel lockdown requires a trusted signing key." >&2
		exit 1
	fi
	exit 0
fi
wifi_secure_boot=$(mokutil --sb-state)
[[ $wifi_secure_boot != 'SecureBoot disabled' ]] || exit 0
[[ $wifi_secure_boot == 'SecureBoot enabled' ]] || { echo "Cannot check Secure Boot trust." >&2; exit 1; }
# Match the enrolled certificate to the signature on this particular module.
# An unrelated enrolled key cannot authorize a module signed by another key.
for wifi_config in /etc/dkms/framework.conf /etc/dkms/framework.conf.d/*.conf; do
	[[ ! -f $wifi_config ]] || source "$wifi_config"
done
wifi_module=("$wifi_modules"/cfg80211.ko*)
[[ ${#wifi_module[@]} == 1 && -f ${wifi_module[0]} ]]
wifi_signer=$(modinfo -F sig_key "${wifi_module[0]}" | tr -d ':' | tr '[:lower:]' '[:upper:]' | sed 's/^0*//')
for wifi_certificate in "${mok_certificate:-}" /var/lib/shim-signed/mok/MOK.der /var/lib/dkms/mok.pub; do
	[[ -f $wifi_certificate ]] || continue
	wifi_serial=$(openssl x509 -inform DER -in "$wifi_certificate" -noout -serial 2>/dev/null) || continue
	wifi_serial=${wifi_serial#serial=}
	wifi_serial=$(tr '[:lower:]' '[:upper:]' <<<"$wifi_serial" | sed 's/^0*//')
	if [[ -n $wifi_signer && $wifi_signer == "$wifi_serial" ]] && mokutil --test-key "$wifi_certificate"; then exit 0; fi
done
echo "Enroll the DKMS signing key before installing this wireless module." >&2
exit 1
SIGNING

cat >"$wifi_dkms/refresh.sh" <<'REFRESH'
#!/usr/bin/env bash
# Refresh next-boot files after DKMS installs or removes the module.
set -euo pipefail
wifi_kernel=${1:?Kernel release required}
depmod -a "$wifi_kernel"
if [[ -f /boot/initrd.img-$wifi_kernel ]]; then update-initramfs -u -k "$wifi_kernel"; fi
REFRESH
chmod 755 "$wifi_dkms/"*.sh

# Build the distribution supplicant with the P2P patch and its normal features.
# The distribution package supplies service files. The bundled executable's
# library requirements are calculated below by dpkg-shlibdeps.
wifi_wpa_source_version=$(dpkg-query -W -f='${source:Version}' wpasupplicant)
source "$wifi_dkms/source.sh"
fetch_source wpa "$wifi_wpa_source_version" "$wifi_work/wpa"
patch --batch --fuzz=0 -d "$wifi_work/wpa" -p1 <"$wifi_ci/patches/wpasupplicant-concurrent-go.patch"
cp "$wifi_work/wpa/debian/config/wpasupplicant/linux" "$wifi_work/wpa/wpa_supplicant/.config"
export DEB_BUILD_MAINT_OPTIONS=hardening=+all
export CFLAGS="$(dpkg-buildflags --get CFLAGS) $(dpkg-buildflags --get CPPFLAGS)"
export LDFLAGS="$(dpkg-buildflags --get LDFLAGS)"
make -C "$wifi_work/wpa/wpa_supplicant" -j"$wifi_jobs" wpa_supplicant
install -m 755 "$wifi_work/wpa/wpa_supplicant/wpa_supplicant" "$wifi_root/usr/sbin/wpa_supplicant"
strip --strip-debug "$wifi_root/usr/sbin/wpa_supplicant"
install -m 644 "$wifi_work/wpa/debian/copyright" "$wifi_root/usr/share/doc/$SUPPORT_PACKAGE/wpa-copyright"
printf '%s\n' "$wifi_wpa_source_version" >"$wifi_root/usr/share/doc/$SUPPORT_PACKAGE/wpa-source-version"

# Generate the Debian lifecycle hooks here so packaging has one entry point.
cat >"$wifi_root/DEBIAN/preinst" <<'PREINST'
#!/usr/bin/env bash
set -euo pipefail
case ${1:-} in
	install | upgrade)
		dpkg-divert --package mrs-uav-wifi-direct-support --add --rename --divert /usr/sbin/wpa_supplicant.distrib /usr/sbin/wpa_supplicant
		;;
esac
PREINST
cat >"$wifi_root/DEBIAN/postinst" <<'POSTINST'
#!/usr/bin/env bash
set -euo pipefail
[[ ${1:-} == configure ]] || exit 0
# Prepare this package's module without changing other DKMS drivers.
prepare_kernel_modules() (
	if [[ -f /etc/dkms/no-autoinstall ]]; then
		printf 'Automatic DKMS installation is disabled by the administrator.\n'
		exit 0
	fi
	wifi_name=mrs-uav-wifi-direct-support
	wifi_version=@VERSION@
	wifi_source=/usr/src/$wifi_name-$wifi_version
	wifi_current=$(uname -r)
	wifi_kernels=()
	if ! dkms status -m "$wifi_name" -v "$wifi_version" | grep -q .; then dkms add -m "$wifi_name" -v "$wifi_version"; fi
	# Header packages can outlive their kernel images. Prepare the running kernel
	# and newest installed image, following the usual DKMS installation policy.
	for wifi_path in /lib/modules/*; do
		wifi_kernel=${wifi_path##*/}
		[[ $wifi_kernel == "$wifi_current" || -e /boot/vmlinuz-$wifi_kernel ]] || continue
		wifi_kernels+=("$wifi_kernel")
	done
	wifi_latest=$(printf '%s\n' "${wifi_kernels[@]}" | sort -V | tail -n 1)
	wifi_targets=("$wifi_current")
	if [[ -n $wifi_latest && $wifi_latest != "$wifi_current" ]]; then wifi_targets+=("$wifi_latest"); fi
	for wifi_kernel in "${wifi_targets[@]}"; do
		if [[ ! -f /lib/modules/$wifi_kernel/build/Module.symvers ]]; then
			printf 'Kernel %s: module build deferred until its headers are installed.\n' "$wifi_kernel"
			continue
		fi
		# Reuse the packaged eligibility check before invoking DKMS. This avoids
		# reporting excluded radios/kernels as failed builds during installation.
		if ! (
			kernelver=$wifi_kernel
			kernel_source_dir=/lib/modules/$wifi_kernel/build
			dkms_tree=/var/lib/dkms
			arch=$(uname -m)
			source "$wifi_source/dkms.conf"
			[[ -n $AUTOINSTALL ]]
		); then
			printf 'Kernel %s uses the distribution Wi-Fi module.\n' "$wifi_kernel"
			continue
		fi
		# install builds this module when needed; autoinstall also processes other
		# registered modules, even when -m and -v are supplied in DKMS 3.0.
		dkms install -m "$wifi_name" -v "$wifi_version" -k "$wifi_kernel"
	done
)
prepare_kernel_modules
# Configure the existing Wi-Fi services once for the local UAV user.
setup_wifi() (
set -euo pipefail
umask 077
wifi_state=/var/lib/mrs-uav-wifi-direct/setup
fail() { printf 'Wi-Fi setup: %s\n' "$*" >&2; exit 1; }
if [[ -e $wifi_state ]]; then
	[[ ! -L $wifi_state && $(stat -c %u "$wifi_state") == 0 && $(stat -c %a "$wifi_state") == 700 ]] || fail "Unsafe saved setup permissions"
fi
if [[ -d $wifi_state ]]; then
	bash /var/lib/dpkg/info/mrs-uav-wifi-direct-support.prerm rollback-setup
fi
wifi_user=${SUDO_USER:-${USER:-}}
if [[ -z $wifi_user || $wifi_user == root ]]; then wifi_user=$(getent passwd 1000 | cut -d: -f1 || true); fi
if [[ -z $wifi_user ]]; then
	printf 'QUIC support installed; automatic P2P setup requires a local user account.\n'
	exit 0
fi
wifi_group=netdev wifi_control=/run/wpa_supplicant wifi_p2p=''
if [[ -d /sys/class/net/wlan0/phy80211 ]]; then
	wifi_interface=wlan0
else
	mapfile -t wifi_interfaces < <(iw dev | awk '$1=="Interface" {name=$2} $1=="type" && $2=="managed" {print name}')
	if [[ ${#wifi_interfaces[@]} != 1 ]]; then
		printf 'QUIC support installed; automatic P2P setup requires one managed Wi-Fi interface.\n'
		exit 0
	fi
	wifi_interface=${wifi_interfaces[0]}
fi
wifi_service=netplan-wpa-$wifi_interface.service
if ! systemctl is-active --quiet "$wifi_service" || ! systemctl is-active --quiet systemd-networkd; then
	printf 'QUIC support installed; automatic P2P setup requires the MRS netplan Wi-Fi service and systemd-networkd.\n'
	exit 0
fi
[[ $wifi_user =~ ^[a-z_][a-z0-9_-]*\$?$ ]] || fail "Invalid local user name"
[[ $wifi_interface =~ ^[A-Za-z0-9_.-]{1,15}$ ]] || fail "Invalid Wi-Fi interface name"
	for wifi_tool in getent id ip wpa_cli systemctl networkctl busctl systemd-tmpfiles systemd-analyze groupadd usermod runuser install stat cmp; do
		command -v "$wifi_tool" >/dev/null || fail "Missing command: $wifi_tool"
	done
	if ! getent passwd "$wifi_user" >/dev/null || [[ $(id -u "$wifi_user") == 0 ]]; then fail "Target must be an existing non-root account"; fi
	[[ $(getent group "$wifi_group" | cut -d: -f3) != 0 ]] || fail "Use a non-root control group"
	[[ ! -L $wifi_state ]] || fail "Setup state must not be a symbolic link"
	[[ -d /sys/class/net/$wifi_interface/phy80211 ]] || fail "$wifi_interface is not a Wi-Fi interface"
	systemctl is-active --quiet "$wifi_service" || fail "$wifi_service must already be active"
	systemctl is-active --quiet systemd-networkd || fail "Existing systemd-networkd must already be active"
	if [[ -z $wifi_p2p ]]; then
		wifi_p2p=$wifi_interface
		[[ ! -S $wifi_control/p2p-dev-$wifi_interface ]] || wifi_p2p=p2p-dev-$wifi_interface
	fi
	[[ $wifi_p2p =~ ^[A-Za-z0-9_.-]{1,32}$ ]] || fail "Invalid P2P control interface"
	for wifi_socket in "$wifi_control/$wifi_interface" "$wifi_control/$wifi_p2p"; do
		[[ -S $wifi_socket && ! -L $wifi_socket ]] || fail "Missing supplicant socket: $wifi_socket"
	done
	compgen -G "/sys/class/net/p2p-$wifi_interface-*" >/dev/null && fail "Stop the ROS node and its P2P group before setup"
	wifi_pid=$(systemctl show "$wifi_service" --property=MainPID --value)
	[[ $wifi_pid =~ ^[1-9][0-9]*$ && -r /proc/$wifi_pid/cmdline ]] || fail "Cannot read the running supplicant command"
	mapfile -d '' -t wifi_arguments <"/proc/$wifi_pid/cmdline"
	[[ ${wifi_arguments[0]##*/} == wpa_supplicant ]] || fail "$wifi_service does not run wpa_supplicant directly"
	wifi_found=false
	for ((wifi_i = 1; wifi_i < ${#wifi_arguments[@]}; ++wifi_i)); do
		[[ ${wifi_arguments[wifi_i]} != "-i$wifi_interface" ]] || wifi_found=true
		if [[ ${wifi_arguments[wifi_i]} == -i && ${wifi_arguments[wifi_i + 1]:-} == "$wifi_interface" ]]; then wifi_found=true; fi
		[[ ${wifi_arguments[wifi_i]} != -N ]] || fail "Use a per-interface supplicant service"
	done
	[[ $wifi_found == true ]] || fail "$wifi_service does not manage $wifi_interface"

	wifi_unit=/etc/systemd/system/$wifi_service.d/permissions.conf
	wifi_profile=/etc/systemd/network/10-mrs-wifi-direct.network
	wifi_policy=/etc/polkit-1/rules.d/50-mrs-uav-wifi-direct.rules
	wifi_restart=/etc/needrestart/conf.d/50-mrs-uav-wifi-direct.conf
	wifi_requested=$(printf '%s\n' "$wifi_user" "$wifi_group" "$wifi_interface" "$wifi_service" "$wifi_control" "$wifi_p2p")

	# Read and update the Wi-Fi control socket permissions.
	wpa_value() {
		wpa_cli -p "$wifi_control" -i "$1" "${@:2}"
	}
	for wifi_endpoint in "$wifi_interface" "$wifi_p2p"; do
		[[ $(wpa_value "$wifi_endpoint" ping) == PONG ]] || fail "Supplicant on $wifi_endpoint is not responding"
	done

	wifi_work=$(mktemp -d)
	wifi_started=false wifi_success=false wifi_temp=''

	# Roll back a failed first setup using the package's removal hook.
	cleanup() {
		local status=$?
		trap - EXIT
		if [[ $wifi_started == true && $wifi_success == false ]]; then
			printf 'Wi-Fi setup failed; restoring previous settings.\n' >&2
			bash /var/lib/dpkg/info/mrs-uav-wifi-direct-support.prerm rollback-setup || printf 'Settings could not be fully restored; the saved files have been kept.\n' >&2
		fi
		[[ -z $wifi_temp ]] || rm -f -- "$wifi_temp"
		rm -rf -- "$wifi_work"
		exit "$status"
	}
	trap cleanup EXIT
	trap 'exit 130' INT
	trap 'exit 143' TERM

	# Systemd performs its own $ and % expansion even inside quoted arguments.
	# Escape those characters and backslashes; keep each original argv token intact.
	{
		printf '[Service]\nExecStart=\nExecStart='
		for ((wifi_i = 0; wifi_i < ${#wifi_arguments[@]}; ++wifi_i)); do
			wifi_argument=${wifi_arguments[wifi_i]}
			if [[ $wifi_argument == -O ]]; then
				((wifi_i += 1))
				continue
			fi
			[[ $wifi_argument != -O* ]] || continue
			[[ $wifi_argument != *$'\n'* && $wifi_argument != *$'\r'* ]] || fail "Unsupported control character in service arguments"
			wifi_argument=${wifi_argument//\\/\\\\}
			wifi_argument=${wifi_argument//\"/\\\"}
			wifi_argument=${wifi_argument//\$/\$\$}
			wifi_argument=${wifi_argument//%/%%}
			printf '"%s" ' "$wifi_argument"
		done
		printf '%s\n' "-O \"DIR=$wifi_control GROUP=$wifi_group\""
	} >"$wifi_work/unit"
	cat >"$wifi_work/network" <<-EOF
	[Match]
	Name=p2p-$wifi_interface-*

	[Link]
	RequiredForOnline=no

	[Network]
	DHCP=no
	LinkLocalAddressing=no
	IPv6AcceptRA=no
	ConfigureWithoutCarrier=yes
	EOF
	cat >"$wifi_work/policy" <<-EOF
	// Allow only the installing account to reload profiles after P2P address updates.
	polkit.addRule(function(action, subject) {
		if (action.id === "org.freedesktop.network1.reload" && subject.user === "$wifi_user") {
			return polkit.Result.YES;
		}
	});
	EOF
	cat >"$wifi_work/restart" <<-EOF
	# Apply the replacement Wi-Fi executable at reboot, preserving the live AP link.
	\$nrconf{override_rc}{qr(^\\Q$wifi_service\\E\$)} = 0;
	EOF
	# Verify syntax before replacing either persistent file or changing membership.
	cp "$wifi_work/unit" "$wifi_work/permissions-check.service"
	systemd-analyze verify "$wifi_work/permissions-check.service"
	printf 'User: %s; group: %s; AP interface: %s\n' "$wifi_user" "$wifi_group" "$wifi_interface"
	printf 'Existing service: %s\nFiles: %s\n       %s\n' "$wifi_service" "$wifi_unit" "$wifi_profile"

	# Save the original files, membership and live socket metadata only once.
	# The state directory is private to root and contains no network credentials.
	install -d -m 700 /var/lib/mrs-uav-wifi-direct
	install -d -m 700 "$wifi_state"
	wifi_started=true
	printf '%s\n' "$wifi_requested" >"$wifi_state/settings"
	for wifi_kind in unit network policy restart; do
		wifi_path=$wifi_unit
		[[ $wifi_kind != network ]] || wifi_path=$wifi_profile
		[[ $wifi_kind != policy ]] || wifi_path=$wifi_policy
		[[ $wifi_kind != restart ]] || wifi_path=$wifi_restart
		[[ ! -e $wifi_path ]] || cp -p -- "$wifi_path" "$wifi_state/original.$wifi_kind"
		cp "$wifi_work/$wifi_kind" "$wifi_state/installed.$wifi_kind"
	done
	id -nG "$wifi_user" | tr ' ' '\n' >"$wifi_work/groups"
	if grep -Fxq "$wifi_group" "$wifi_work/groups"; then touch "$wifi_state/user-was-member"; fi
	if getent group "$wifi_group" >/dev/null; then touch "$wifi_state/group-existed"; fi
	for wifi_endpoint in "$wifi_interface" "$wifi_p2p"; do
		wifi_old=$(wpa_value "$wifi_endpoint" get ctrl_interface)
		[[ $wifi_old != FAIL* && -n $wifi_old ]] || fail "Cannot snapshot control settings for $wifi_endpoint"
		printf '%s\n' "$wifi_old" >"$wifi_state/control.$wifi_endpoint"
	done
	stat -c '%u %g %a %n' "$wifi_control" "$wifi_control/$wifi_interface" "$wifi_control/$wifi_p2p" >"$wifi_state/socket-metadata"
	touch "$wifi_state/snapshot-complete"
	groupadd -f "$wifi_group"
	usermod -aG "$wifi_group" "$wifi_user"
	systemd-tmpfiles --create /usr/lib/tmpfiles.d/mrs-uav-wifi-direct.conf
	# The node adds its address after authentication, keeping WPS clients IP-free.
	printf '[Network]\n' >/run/systemd/network/10-mrs-wifi-direct.network.d/50-address.conf
	for wifi_endpoint in "$wifi_interface" "$wifi_p2p"; do
		[[ $(wpa_value "$wifi_endpoint" set ctrl_interface "DIR=$wifi_control GROUP=$wifi_group") == OK ]] || fail "Cannot set $wifi_endpoint socket group"
	done
	chgrp "$wifi_group" "$wifi_control" "$wifi_control/$wifi_interface" "$wifi_control/$wifi_p2p"
	chmod g+rx "$wifi_control"
	chmod g+rw "$wifi_control/$wifi_interface" "$wifi_control/$wifi_p2p"

	# A rename within the destination directory makes each config replacement atomic.
	for wifi_kind in unit network policy restart; do
		wifi_path=$wifi_unit
		[[ $wifi_kind != network ]] || wifi_path=$wifi_profile
		[[ $wifi_kind != policy ]] || wifi_path=$wifi_policy
		[[ $wifi_kind != restart ]] || wifi_path=$wifi_restart
		if [[ ! -d $(dirname -- "$wifi_path") ]]; then install -d -m 755 "$(dirname -- "$wifi_path")"; fi
		wifi_temp=$(mktemp "$wifi_path.XXXXXX")
		install -m 644 "$wifi_work/$wifi_kind" "$wifi_temp"
		mv -fT -- "$wifi_temp" "$wifi_path"
	done
	systemctl daemon-reload
	networkctl reload
	[[ $(systemctl show "$wifi_service" --property=MainPID --value) == "$wifi_pid" ]] || fail "Supplicant changed during setup; rerun after it is stable"
	for wifi_endpoint in "$wifi_interface" "$wifi_p2p"; do
		[[ $(runuser -u "$wifi_user" -- wpa_cli -p "$wifi_control" -i "$wifi_endpoint" ping) == PONG ]] || fail "$wifi_user cannot access $wifi_endpoint"
	done
	touch "$wifi_state/complete"
	wifi_success=true
	printf 'Wi-Fi permissions configured. Log out and log in to activate %s membership.\n' "$wifi_group"
)
setup_wifi
printf 'Wi-Fi Direct support installed. Reboot to use the updated Wi-Fi executable and kernel support.\n'
POSTINST
cat >"$wifi_root/DEBIAN/prerm" <<'PRERM'
#!/usr/bin/env bash
set -euo pipefail
restore_setup() (
	umask 077
	wifi_state=/var/lib/mrs-uav-wifi-direct/setup
	[[ -d $wifi_state && ! -L $wifi_state ]] || {
		printf 'No Wi-Fi permission setup to restore.\n'
		exit 0
	}
	[[ $(stat -c %u "$wifi_state") == 0 && $(stat -c %a "$wifi_state") == 700 ]] || {
		printf 'Unsafe setup state permissions.\n' >&2
		exit 1
	}
	mapfile -t wifi_settings <"$wifi_state/settings"
	[[ ${#wifi_settings[@]} -ge 6 ]] || {
		printf 'Incomplete setup snapshot; inspect %s.\n' "$wifi_state" >&2
		exit 1
	}
	wifi_user=${wifi_settings[0]} wifi_group=${wifi_settings[1]} wifi_interface=${wifi_settings[2]}
	wifi_service=${wifi_settings[3]} wifi_control=${wifi_settings[4]} wifi_p2p=${wifi_settings[5]}
	wifi_unit=/etc/systemd/system/$wifi_service.d/permissions.conf
	wifi_profile=/etc/systemd/network/10-mrs-wifi-direct.network
	wifi_policy=/etc/polkit-1/rules.d/50-mrs-uav-wifi-direct.rules
	wifi_restart=/etc/needrestart/conf.d/50-mrs-uav-wifi-direct.conf
	compgen -G "/sys/class/net/p2p-$wifi_interface-*" >/dev/null && {
		printf 'Stop the mrs_uav_wifi_direct node before removing its support package.\n' >&2
		exit 1
	}
	if [[ ! -f $wifi_state/snapshot-complete ]]; then
		# No privileged mutations occur until the complete snapshot marker is written.
		rm -rf -- "$wifi_state"
		printf 'Removed an unfinished snapshot; no settings had been changed.\n'
		exit 0
	fi

	# Protect administrator edits. A previous partial uninstall is also accepted:
	# each file may already match its original, or have been removed as intended.
	for wifi_kind in unit network policy restart; do
		wifi_path=$wifi_unit
		[[ $wifi_kind != network ]] || wifi_path=$wifi_profile
		[[ $wifi_kind != policy ]] || wifi_path=$wifi_policy
		[[ $wifi_kind != restart ]] || wifi_path=$wifi_restart
		if [[ -e $wifi_path ]] && ! cmp -s "$wifi_path" "$wifi_state/installed.$wifi_kind" && ! cmp -s "$wifi_path" "$wifi_state/original.$wifi_kind"; then
			printf 'Configuration changed since installation: %s. Inspect it before uninstalling.\n' "$wifi_path" >&2
			exit 1
		fi
	done
	printf 'Restore previous files and Wi-Fi permissions for %s on %s.\n' "$wifi_user" "$wifi_interface"
	for wifi_kind in unit network policy restart; do
		wifi_path=$wifi_unit
		[[ $wifi_kind != network ]] || wifi_path=$wifi_profile
		[[ $wifi_kind != policy ]] || wifi_path=$wifi_policy
		[[ $wifi_kind != restart ]] || wifi_path=$wifi_restart
		if [[ -f $wifi_state/original.$wifi_kind ]]; then
			if [[ ! -d $(dirname -- "$wifi_path") ]]; then install -d -m 755 "$(dirname -- "$wifi_path")"; fi
			wifi_temp=$(mktemp "$wifi_path.XXXXXX")
			cp -p -- "$wifi_state/original.$wifi_kind" "$wifi_temp"
			mv -fT -- "$wifi_temp" "$wifi_path"
		else
			rm -f -- "$wifi_path"
		fi
	done
	systemctl daemon-reload
	if [[ -d /run/systemd/network/10-mrs-wifi-direct.network.d && ! -L /run/systemd/network/10-mrs-wifi-direct.network.d ]]; then
		rm -f -- /run/systemd/network/10-mrs-wifi-direct.network.d/50-address.conf
		rmdir /run/systemd/network/10-mrs-wifi-direct.network.d 2>/dev/null || true
	fi
	if systemctl is-active --quiet systemd-networkd; then networkctl reload; fi
	for wifi_endpoint in "$wifi_interface" "$wifi_p2p"; do
		if [[ -S $wifi_control/$wifi_endpoint ]]; then
			wifi_original=$(cat "$wifi_state/control.$wifi_endpoint")
			[[ $(wpa_cli -p "$wifi_control" -i "$wifi_endpoint" set ctrl_interface "$wifi_original") == OK ]] || {
				printf 'Cannot restore %s control settings; snapshot retained.\n' "$wifi_endpoint" >&2
				exit 1
			}
		fi
	done
	while read -r wifi_uid wifi_gid wifi_mode wifi_path; do
		if [[ -e $wifi_path ]]; then
			chown --no-dereference "$wifi_uid:$wifi_gid" "$wifi_path"
			chmod "$wifi_mode" "$wifi_path"
		fi
	done <"$wifi_state/socket-metadata"
	if [[ ! -f $wifi_state/user-was-member ]] && getent passwd "$wifi_user" >/dev/null && getent group "$wifi_group" >/dev/null; then
		if [[ $(id -g "$wifi_user") != "$(getent group "$wifi_group" | cut -d: -f3)" ]]; then
			# Remove only this supplementary group, retaining any other memberships.
			if id -nG "$wifi_user" | tr ' ' '\n' | grep -Fxq "$wifi_group"; then gpasswd -d "$wifi_user" "$wifi_group"; fi
		fi
	fi
	if [[ ! -f $wifi_state/group-existed ]] && getent group "$wifi_group" >/dev/null; then
		wifi_group_record=$(getent group "$wifi_group")
		wifi_group_gid=$(cut -d: -f3 <<<"$wifi_group_record")
		# Keep a group if another administrator/user has started using it meanwhile.
		if [[ -z $(cut -d: -f4 <<<"$wifi_group_record") ]] && ! getent passwd | awk -F: -v gid="$wifi_group_gid" '$4 == gid { found=1 } END { exit !found }'; then
			groupdel "$wifi_group"
		fi
	fi
	rm -rf -- "$wifi_state"
	printf 'Previous settings restored. Log out/in to refresh group membership.\n'
)
case ${1:-} in
	remove | rollback-setup) restore_setup ;;
	upgrade) ;;
	*) exit 0 ;;
esac
[[ ${1:-} != rollback-setup ]] || exit 0
if [[ -d /var/lib/dkms/mrs-uav-wifi-direct-support/@VERSION@ ]]; then
	dkms remove -m mrs-uav-wifi-direct-support -v @VERSION@ --all
fi
PRERM
cat >"$wifi_root/DEBIAN/postrm" <<'POSTRM'
#!/usr/bin/env bash
set -euo pipefail
case ${1:-} in
	remove | abort-install)
		dpkg-divert --package mrs-uav-wifi-direct-support --remove --rename --divert /usr/sbin/wpa_supplicant.distrib /usr/sbin/wpa_supplicant
		;;
esac
POSTRM
sed -i "s/@VERSION@/$wifi_version/g" "$wifi_root/DEBIAN/"{postinst,prerm}
chmod 755 "$wifi_root/DEBIAN/"{preinst,postinst,prerm,postrm}

# Resolve library dependencies from the binaries included in this deb.
mkdir -p "$wifi_work/debian"
printf 'Source: %s\nSection: libs\nPriority: optional\nMaintainer: Vojtech Vrba <vrba.vojtech@fel.cvut.cz>\n\nPackage: %s\nArchitecture: any\nDescription: Wi-Fi Direct support\n' "$SUPPORT_PACKAGE" "$SUPPORT_PACKAGE" >"$wifi_work/debian/control"
wifi_depends=$(cd "$wifi_work" && dpkg-shlibdeps -O -e"$wifi_root/usr/lib/$wifi_multiarch/libmsquic.so.$MSQUIC_VERSION" -e"$wifi_root/usr/sbin/wpa_supplicant")
wifi_depends=${wifi_depends#shlibs:Depends=}
case $wifi_arch in
	amd64) wifi_headers='linux-headers-generic-hwe-24.04 | linux-headers-generic' ;;
	arm64) wifi_headers='linux-headers-raspi | linux-headers-generic' ;;
	*) wifi_headers=linux-headers-generic ;;
esac
cat >"$wifi_root/DEBIAN/control" <<CONTROL
Package: $SUPPORT_PACKAGE
Version: $wifi_version
Section: libs
Priority: optional
Architecture: $wifi_arch
Maintainer: Vojtech Vrba <vrba.vojtech@fel.cvut.cz>
Depends: $wifi_depends, dkms, dpkg-dev, make, gcc, binutils, patch, kmod,
 libelf-dev, mokutil, openssl, initramfs-tools, linux-firmware, wireless-regdb,
 iw, iproute2, systemd, polkitd, passwd,
 wpasupplicant, $wifi_headers
Provides: libmsquic (= $MSQUIC_VERSION)
Conflicts: libmsquic, libmsquic-debug
Replaces: libmsquic, libmsquic-debug
Homepage: https://github.com/ctu-mrs/mrs_uav_wifi_direct
Description: Wi-Fi Direct and QUIC support for MRS UAV System
CONTROL
printf 'libmsquic 2 %s (>= %s)\n' "$SUPPORT_PACKAGE" "$wifi_version" >"$wifi_root/DEBIAN/shlibs"
printf 'activate-noawait ldconfig\n' >"$wifi_root/DEBIAN/triggers"
cat >"$wifi_root/usr/lib/$wifi_multiarch/pkgconfig/msquic.pc" <<PC
prefix=/usr
libdir=\${prefix}/lib/$wifi_multiarch
includedir=\${prefix}/include/msquic

Name: MsQuic
Description: QUIC library for MRS UAV topic sharing
Version: $MSQUIC_VERSION
Libs: -L\${libdir} -lmsquic
Cflags: -I\${includedir}
PC
(cd "$wifi_root" && find usr -type f -print0 | sort -z | xargs -0 md5sum >DEBIAN/md5sums)
wifi_artifact=$wifi_output/${SUPPORT_PACKAGE}_${wifi_version}_${wifi_arch}.deb
dpkg-deb --build --root-owner-group "$wifi_root" "$wifi_artifact"
dpkg-deb --info "$wifi_artifact"

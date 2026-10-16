![ROS Package Build](https://github.com/ctu-mrs/mrs_uav_wifi_direct/actions/workflows/ros_package_build.yml/badge.svg)
![Generic Package Build](https://github.com/ctu-mrs/mrs_uav_wifi_direct/actions/workflows/generic_package_build.yml/badge.svg)

# MRS UAV Wi-Fi Direct

This package shares ROS 2 topics between UAVs over Wi-Fi. It can use the existing access point (AP) connection and create an additional [Wi-Fi Direct](https://en.wikipedia.org/wiki/Wi-Fi_Direct) (peer-to-peer) connection between the UAVs. The direct connection carries data without going through the AP, while maintaining concurrent AP connection.

The P2P network enables direct connection of multiple UAV Zenoh routers. Additionaly, topic messages may travel over implemented [QUIC](https://github.com/microsoft/msquic) protocol, which provides encryption and reliable delivery. Each topic has a separate stream, so packet loss on one topic can be recovered while other topics continue sending.

## Install

With the MRS PPA configured, install the ROS package on each UAV:

```bash
sudo apt install ros-jazzy-mrs-uav-wifi-direct
```

This also installs `mrs-uav-wifi-direct-support`, which contains the QUIC library and Wi-Fi support. DKMS builds supported kernel changes needed by the radio (on Intel NUC) and rebuilds them after kernel updates. The installing user is also granted access to Wi-Fi control and P2P address profile configuration. The support package installation takes effect after a reboot, to leave the running Wi-Fi connection in place.

Automatic setup uses the existing netplan `wpa_supplicant` service and `systemd-networkd`. Wi-Fi control access also lets the selected account change the AP connection, so use a trusted local account. The P2P addresses use subnet `192.168.255.0/24`.

## Quick Start

Copy the configuration to your home directory:

```bash
cp "$(ros2 pkg prefix mrs_uav_wifi_direct)/share/mrs_uav_wifi_direct/config/default.yaml" ~/mrs_uav_wifi_direct.yaml
```

The installed configuration has `uav_list: []`, so the node starts by publishing local radio and discovery information. Topic sharing and Wi-Fi Direct group management begin when the fleet list is filled. In your copy, fill the list with the full names of all participating UAVs, including the local one. UAV names consist of `uav` followed by a number from 1 to 99.

The default shared topic is odometry. Add other topics under `shared_topics` as needed:

```yaml
shared_topics:
  - topic: "/{hostname}/estimation_manager/odom_main"
    message_type: "nav_msgs/msg/Odometry"
    rate_hz: 10.0
    reliability: "best_effort"
```

A positive `rate_hz` sends the latest sample at up to that rate. Set it to zero, or leave it out, to send every new sample. The default `best_effort` subscription works with common sensor publishers. Choose `reliable` when the source publisher supports reliable delivery, especially for large images or arrays.

Start the node on each UAV (either directly or in a TMUX session):

```bash
ros2 launch mrs_uav_wifi_direct mrs_uav_wifi_direct.launch.py config_path:="$HOME/mrs_uav_wifi_direct.yaml"
```

## How the connections work

UAVs automatically choose a Wi-Fi Direct group owner. The owner hosts the group, and the other UAVs join as clients. Clients communicate through the owner. An existing working group is preferred; if its owner disappears, the reachable UAVs choose another eligible owner. Group size and UAV numbers help separate groups choose a common owner when they meet.

When an AP is connected, P2P discovery and the direct group use its frequency and channel width. With no AP connection, the UAVs search for peers and form a group on a permitted channel, preferring 5 GHz. When the AP returns or changes channel, the direct group is updated to match it.

Topic traffic prefers the direct connection and falls back to the AP when enabled. Both QUIC connections stay ready while their networks are available. For AP communication alone, set `wifi_direct.enabled: false`. To require direct delivery, set `network.ap_fallback: false`.

Addresses come from the host's network settings. Within each subnet, the difference between the address's last byte and the UAV number must be the same for every UAV. The node derives this mapping separately for the AP and direct interfaces.

If another network manager already creates the group, set `wifi_direct.existing_group_interface` to that interface. The node will use its assigned address and leave group management to that manager.

The P2P network also carries ordinary IP traffic. With `rmw_zenoh_cpp`, the UAVs' Zenoh routers can use this network to share ROS topics directly. Configure a router on each UAV with the other UAVs' P2P addresses. Independently, the `shared_topics` configuration section controls the separate QUIC bridge.

In your copy of router config (`zenoh_router.json5`), add to `connect` block the IPs of other UAVs on the P2P subnet (192.168.255.* by defeault). Set listening to `0.0.0.0` to allow establishing Zenoh connections on a P2P network which appears later (binding the listener to a P2P address or adding `#iface=p2p-wlan0-…` requires that interface to exist already). Communication begins when the local router and the peer connection are available.

## Discovery and status

Measurements come from the running driver and existing Wi-Fi discovery caches. Collection reads these without starting extra scans. Nearby UAVs appear when the host has recently observed their advertisements. Observations expire after `wifi_direct.peer_timeout_s`. Other fleets can appear in discovery, while only configured UAVs exchange topics. Measurement ages help distinguish a new publication from a new radio sample. The QUIC library provides latency information, including network queuing and processing delays.

## Certificates

The node creates its certificate and private key automatically in `~/.ros/mrs_uav_wifi_direct/<hostname>/`. It remembers each peer's first certificate and checks it on later connections. Make the first connection on a trusted network, and keep this directory between launches and upgrades.

If a UAV's certificate changes, verify the new identity before deleting its `peers/<peer>.sha256` file on the receiving UAV. This allows that receiver to remember the replacement. Keep each private key on its original UAV. Certificates last ten years. Fleets with an existing certificate authority can supply their own certificate, key, and CA file in `tls`.

## Remove

Stop the ROS node, then remove the packages:

```bash
sudo apt remove ros-jazzy-mrs-uav-wifi-direct mrs-uav-wifi-direct-support
```

The support package restores the saved permissions and network files, removes its DKMS modules, and restores the distribution Wi-Fi executable. It preserves group memberships that existed before installation and stops removal if saved files have since been edited. The current Wi-Fi process and loaded driver keep running until the next service start or reboot.

## Build manually from source

Install the build tools and Wi-Fi library headers:

```bash
sudo apt install build-essential binutils cmake git perl dpkg-dev pkg-config patch libyaml-cpp-dev libssl-dev libdbus-1-dev libnl-3-dev libnl-genl-3-dev libnl-route-3-dev libpcsclite-dev libreadline-dev

cd ~/workspace/src/mrs_uav_wifi_direct
bash .ci/build_package.sh /tmp/wifi-direct-packages
sudo apt install /tmp/wifi-direct-packages/mrs-uav-wifi-direct-support_*_$(dpkg --print-architecture).deb

cd ~/workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mrs_uav_wifi_direct
source install/setup.bash
```



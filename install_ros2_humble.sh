#!/bin/bash
set -e

echo "============================"
echo " Installing ROS 2 Humble"
echo "============================"

# Skip if already installed
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "[info] ROS 2 Humble already present at /opt/ros/humble. Skipping install."
    exit 0
fi

# -----------------------------
# 1. Locale configuration
# -----------------------------
echo "[1/7] Configuring locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

# -----------------------------
# 2. Enable required repositories
# -----------------------------
echo "[2/7] Enabling universe repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# -----------------------------
# 3. Add ROS 2 apt source
# -----------------------------
echo "[3/7] Adding ROS 2 apt source..."
sudo apt update
sudo apt install -y curl

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
"https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb

# -----------------------------
# 4. Update system
# -----------------------------
echo "[4/7] Updating package index..."
sudo apt update

echo "[5/7] Upgrading system..."
sudo apt upgrade -y

# -----------------------------
# 5. Install ROS Humble Desktop
# -----------------------------
echo "[6/7] Installing ROS Humble Desktop..."
sudo apt install -y ros-humble-desktop

# -----------------------------
# 6. Auto-source ROS environment
# -----------------------------
echo "[7/7] Adding ROS setup to ~/.bashrc ..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
fi

echo "======================================"
echo " ROS 2 Humble installation complete!"
echo " Open a new terminal to start using it."
echo "======================================"

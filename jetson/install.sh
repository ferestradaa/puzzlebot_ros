echo "Installing apriltag 3.2 from source"
cd /tmp
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout v3.2.0
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
sudo cmake --install build
echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/apriltag.conf
sudo ldconfig
cd /tmp && rm -rf apriltag


BT_VERSION="4.9.0"
BT_PREFIX="/opt/behaviortree"

sudo apt install -y libzmq3-dev libboost-dev
sudo apt install libzxingcore-dev
sudo apt install libzbar-dev

cd /tmp 
git clone --branch ${BT_VERSION} https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DBTCPP_EXAMPLES=OFF7
cmake --build build -j$(nproc)
sudo cmake --install build --prefix ${BT_PREFIX}
echo "export CMAKE_PREFIX_PATH=${BT_PREFIX}:\$CMAKE_PREFIX_PATH" >> ~/.bashrc
source ~/.bashrc


echo "Installing pip dependencies (Humble on Focal workaround)"
pip3 install xacro
pip3 install pupil-apriltags

echo "Removing unnecessary sttuff for clean cleaning RAM"
sudo systemctl set-default multi-user.target
sudo systemctl disable --now docker.service docker.socket containerd
sudo systemctl disable --now snapd.service snapd.socket

echo "Configuring udev rules"
sudo tee /etc/udev/rules.d/99-puzzlebot.rules > /dev/null << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="3aa010ac1bffec11b2a96d508ce70331", SYMLINK+="ttyHACKER"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyLIDAR"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Verifying devices..."
ls -la /dev/ttyHACKER /dev/ttyLIDAR 2>/dev/null || echo "WARNING: devices not found, connect hardware and re-run udevadm trigger"
echo "=== Installing pip dependencies (Humble on Focal workaround) ==="
pip3 install xacro

echo "=== Configuring udev rules ==="
sudo tee /etc/udev/rules.d/99-puzzlebot.rules > /dev/null << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="3aa010ac1bffec11b2a96d508ce70331", SYMLINK+="ttyHACKER"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttyLIDAR"
EOF

sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Verifying devices..."
ls -la /dev/ttyHACKER /dev/ttyLIDAR 2>/dev/null || echo "WARNING: devices not found, connect hardware and re-run udevadm trigger"
#!/bin/bash
SERVICE_NAME="env_sensors"
SCRIPT_PATH="/ProtoDevelopementpi/Main.py"

# 1. Add shebang if needed and make executable
if ! head -n1 "$SCRIPT_PATH" 2>/dev/null | grep -q '^#!'; then
  sed -i '1i #!/usr/bin/env python3' "$SCRIPT_PATH"
fi
chmod +x "$SCRIPT_PATH"

# 2. Create systemd service unit
sudo tee /etc/systemd/system/${SERVICE_NAME}.service >/dev/null <<EOF
[Unit]
Description=Environment Sensor Main Service
After=multi-user.target

[Service]
Type=simple
User=root
WorkingDirectory=$(dirname "$SCRIPT_PATH")
ExecStart=/usr/bin/env python3 $SCRIPT_PATH
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 3. Enable and start
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}.service
sudo systemctl restart ${SERVICE_NAME}.service

echo "✅ Service '$SERVICE_NAME' set up and running!"
echo "Check with: sudo systemctl status ${SERVICE_NAME}"


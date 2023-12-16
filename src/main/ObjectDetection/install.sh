echo "Creating the Object Detection systemd service..."

if service --status-all | grep -Fq 'objectdetection'; then
  systemctl stop objectdetection
  systemctl disable objectdetection
  rm /lib/systemd/system/objectdetection.service
  rm /etc/systemd/system/objectdetection.service
  systemctl daemon-reload
  systemctl reset-failed
fi

cat > /lib/systemd/system/objectdetection.service <<EOF
[Unit]
Description=Service that runs objectdetection
[Service]
WorkingDirectory=/home/orangepi/inference_with_lite
# Run objectdetection at "nice" -10, which is higher priority than standard
Nice=-10
# for non-uniform CPUs, like big.LITTLE, you want to select the big cores
# look up the right values for your CPU
# AllowCPUs=4-7
ExecStart=/usr/bin/python3 /home/orangepi/inference_with_lite/test.py
ExecStop=/bin/systemctl kill objectdetection
Type=simple
Restart=on-failure
RestartSec=1
[Install]
WantedBy=multi-user.target
EOF

cp /lib/systemd/system/objectdetection.service /etc/systemd/system/objectdetection.service
chmod 644 /etc/systemd/system/objectdetection.service
systemctl daemon-reload
systemctl enable objectdetection.service

echo "Created objectdetection systemd service."
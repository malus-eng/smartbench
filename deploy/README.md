# Deployment — boot-time auto-start

Brings the locker up automatically on power-on: the three drivers are loaded by
systemd, then the controller service starts once devices are ready.

## Install

1. Install and register the kernel modules:
```sh
   for d in hcsr04 hx711 sts3215; do
     cd ~/smartbench/drivers/$d
     sudo make -C /lib/modules/$(uname -r)/build M=$(pwd) modules_install
   done
   sudo depmod -a
```

2. Auto-load the modules at boot:
```sh
   sudo cp deploy/modules-load-smartbench.conf /etc/modules-load.d/smartbench.conf
```

3. Install and enable the controller service:
```sh
   sudo cp deploy/smartbench-locker.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable smartbench-locker.service
```

4. Reboot. The bench powers up into STANDBY with no host attached.

## Notes
- The service waits for `systemd-modules-load.service` and an extra few seconds
  so /dev nodes and the servo's sysfs interface are ready before it starts.
- Logs: `sudo journalctl -u smartbench-locker.service -f`
EOF


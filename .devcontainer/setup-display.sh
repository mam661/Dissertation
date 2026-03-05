#!/bin/bash
# setup-display.sh
# Configures X11 display forwarding depending on what's available.
# - WSLg (Windows 11 / WSL2): mounts X11 unix socket directly
# - VcXsrv fallback (Windows 10): uses TCP to host.docker.internal

BASHRC=/root/.bashrc

echo "" >> $BASHRC
echo "# --- Display setup (auto-configured) ---" >> $BASHRC

# Check if the WSLg X11 socket is available
if [ -S /tmp/.X11-unix/X0 ]; then
    echo "WSLg X11 socket detected, using DISPLAY=:0"
    echo 'export DISPLAY=:0' >> $BASHRC
    echo 'export WAYLAND_DISPLAY=wayland-0' >> $BASHRC
    echo 'export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH' >> $BASHRC
    echo 'export GALLIUM_DRIVER=d3d12' >> $BASHRC
else
    echo "WSLg not available, falling back to host.docker.internal (VcXsrv)"
    HOST_IP=$(cat /etc/hosts | grep host.docker.internal | awk '{print $1}')
    if [ -n "$HOST_IP" ]; then
        echo "export DISPLAY=${HOST_IP}:0.0" >> $BASHRC
    else
        echo "export DISPLAY=host.docker.internal:0.0" >> $BASHRC
    fi
    echo 'export LIBGL_ALWAYS_INDIRECT=1' >> $BASHRC
    echo 'export QT_X11_NO_MITSHM=1' >> $BASHRC
fi

echo "Display setup complete. Re-open terminal or run: source ~/.bashrc"
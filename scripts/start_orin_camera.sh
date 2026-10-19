#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/luna_network.sh"
HOST="${LUNA_ORIN_SSH:-nvidia@$LUNA_ORIN_IP}"
SSH_OPTS=(-o ConnectTimeout=5 -o BindInterface="$LUNA_LOCAL_IFACE"
          -o ControlMaster=auto -o ControlPath=/tmp/luna_ssh_mux_%C -o ControlPersist=60)
REMOTE_CONFIG=/home/nvidia/head_2dof_capt_luna_stereo_undistorted.yaml
REMOTE_START=/home/nvidia/head_2dof_capt_start_luna_stereo.sh
scp "${SSH_OPTS[@]}" \
    "$ROOT/configs/orin_6_6_7_100_20261014/luna_stereo_undistorted.yaml" \
    "$HOST:$REMOTE_CONFIG"
scp "${SSH_OPTS[@]}" \
    "$ROOT/configs/orin_6_6_7_100_20261014/start_luna_stereo.sh" \
    "$HOST:$REMOTE_START"
ssh "${SSH_OPTS[@]}" "$HOST" \
    "restart=false
     if pgrep -f '[r]os2 launch gmsl_quadcam quad_csi_quickview' >/dev/null; then
         pids=\$(pgrep -f '[r]os2 launch gmsl_quadcam quad_csi_quickview')
         for pid in \$pids; do
             active_config=\$(tr '\\0' '\\n' </proc/\$pid/cmdline | sed -n 's/^params_file:=//p')
             if [ -z \"\$active_config\" ] || ! cmp -s -- \"\$active_config\" '$REMOTE_CONFIG'; then
                 echo \"Camera backend configuration differs: \$active_config (expected $REMOTE_CONFIG). Stop the existing camera backend before capture.\" >&2
                 exit 1
             fi
             dds=\$(tr '\\0' '\\n' </proc/\$pid/environ | sed -n 's/^CYCLONEDDS_URI=//p')
             case \"\$dds\" in
                 *'NetworkInterface name=\"$LUNA_ORIN_IFACE\"'*'Peer address=\"$LUNA_LOCAL_IP\"'*) ;;
                 *) restart=true ;;
             esac
         done
         if \$restart; then
             echo 'Camera DDS network changed; restarting backend for $LUNA_ORIN_IFACE -> $LUNA_LOCAL_IP'
             kill -TERM \$pids
             for unused in \$(seq 1 50); do
                 pgrep -f '[r]os2 launch gmsl_quadcam quad_csi_quickview' >/dev/null || break
                 sleep .1
             done
         fi
     fi
     if ! pgrep -f '[r]os2 launch gmsl_quadcam quad_csi_quickview' >/dev/null; then
         nohup env LUNA_ORIN_IFACE=$LUNA_ORIN_IFACE LUNA_LOCAL_IP=$LUNA_LOCAL_IP LUNA_REMOTE_CAMERA_CONFIG=$REMOTE_CONFIG \
             bash $REMOTE_START >/home/nvidia/luna_stereo_capt.log 2>&1 </dev/null &
     fi"
echo "Orin stereo backend requested; waiting for fresh DDS frames locally"

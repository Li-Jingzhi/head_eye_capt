#!/usr/bin/env bash
# Source from the project scripts. Select the physically connected camera link.
if [[ "${LUNA_TRANSPORT:-auto}" == auto ]]; then
    if ip -4 -o addr show dev eno0 2>/dev/null | grep -q 'inet 6\.6\.7\.'; then
        LUNA_TRANSPORT=ethernet
    else
        LUNA_TRANSPORT=wifi
    fi
fi
case "$LUNA_TRANSPORT" in
    ethernet)
        LUNA_LOCAL_IFACE=${LUNA_LOCAL_IFACE:-eno0}
        LUNA_ORIN_IFACE=eth0
        LUNA_ORIN_IP=${LUNA_ORIN_IP:-6.6.7.100}
        ;;
    wifi)
        LUNA_LOCAL_IFACE=${LUNA_LOCAL_IFACE:-wlp0s20f3}
        LUNA_ORIN_IFACE=wlan0
        LUNA_ORIN_IP=${LUNA_ORIN_IP:-10.121.40.130}
        ;;
    *) echo "LUNA_TRANSPORT must be auto, ethernet or wifi" >&2; return 1 ;;
esac
LUNA_LOCAL_IP=$(ip -4 -o addr show dev "$LUNA_LOCAL_IFACE" | awk '{split($4,a,"/"); print a[1]; exit}')
if [[ ! "$LUNA_LOCAL_IP" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "No IPv4 address on $LUNA_LOCAL_IFACE" >&2
    return 1
fi
export LUNA_TRANSPORT LUNA_LOCAL_IFACE LUNA_ORIN_IFACE LUNA_ORIN_IP LUNA_LOCAL_IP
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$LUNA_LOCAL_IFACE\"/></Interfaces></General><Discovery><Peers><Peer address=\"$LUNA_ORIN_IP\"/></Peers><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>256</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"

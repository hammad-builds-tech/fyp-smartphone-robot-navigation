#!/bin/bash
# Passive monitor: log ARP/neighbors + any UDP packets hitting discovery port,
# so we can SEE whether the phone transmits on this LAN at all.
LOG=/tmp/phone_probe.log
: > "$LOG"
echo "monitor start $(date)" >> "$LOG"

# background ARP watcher (needs ip; logs any new neighbor)
( timeout 180 ip monitor neigh dev wlp111s0 2>/dev/null | while read -r line; do
    echo "ARP: $line" >> "$LOG"
  done ) &

# UDP listener on the discovery port is taken by the backend; instead sniff
# with tcpdump if present (any traffic from non-PC hosts)
if command -v tcpdump >/dev/null; then
  timeout 170 tcpdump -i wlp111s0 -n -c 200 \
    'udp port 51315 or arp' 2>>"$LOG" | while read -r line; do
      echo "PKT: $line" >> "$LOG"
    done
else
  echo "tcpdump not available" >> "$LOG"
fi
echo "monitor end $(date)" >> "$LOG"

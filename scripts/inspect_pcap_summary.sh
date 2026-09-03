#!/usr/bin/env bash
set -Eeuo pipefail

CAPTURE_ROOT="${1:?usage: inspect_pcap_summary.sh CAPTURE_ROOT}"
ANALYSIS="$CAPTURE_ROOT/analysis"
mkdir -p "$ANALYSIS"
: >"$ANALYSIS/pcap_summary.txt"
: >"$ANALYSIS/pcap_conversations.txt"
: >"$ANALYSIS/pcap_rtps_packets.txt"

shopt -s nullglob
pcaps=("$CAPTURE_ROOT"/pcap/*.pcap "$CAPTURE_ROOT"/pcap/*.pcapng)
((${#pcaps[@]} > 0)) || {
  printf 'ERROR: no PCAP files found\n' >&2
  exit 1
}

for pcap in "${pcaps[@]}"; do
  printf 'FILE=%s\n' "$pcap" >>"$ANALYSIS/pcap_summary.txt"
  all_count="$(tcpdump -nn -r "$pcap" 2>/dev/null | wc -l)"
  udp_count="$(tcpdump -nn -r "$pcap" udp 2>/dev/null | wc -l)"
  arp_count="$(tcpdump -nn -r "$pcap" arp 2>/dev/null | wc -l)"
  printf 'packet_count=%s\nudp_count=%s\narp_count=%s\n' \
    "$all_count" "$udp_count" "$arp_count" >>"$ANALYSIS/pcap_summary.txt"
  if command -v capinfos >/dev/null 2>&1; then
    capinfos "$pcap" >>"$ANALYSIS/pcap_summary.txt" 2>&1 || true
  else
    stat --printf='size_bytes=%s\n' "$pcap" >>"$ANALYSIS/pcap_summary.txt"
    tcpdump -nn -r "$pcap" 2>/dev/null | wc -l | \
      awk '{print "packet_lines=" $1}' >>"$ANALYSIS/pcap_summary.txt"
  fi
  tcpdump -nn -r "$pcap" udp 2>/dev/null | \
    awk '{print $3 " -> " $5}' | sed 's/:$//' | sort | uniq -c | sort -nr | \
    sed "s#^#$(basename "$pcap") #" >>"$ANALYSIS/pcap_conversations.txt" || true
  if command -v tshark >/dev/null 2>&1; then
    tshark -r "$pcap" -Y rtps -T fields -e frame.number -e ip.src -e ip.dst \
      >>"$ANALYSIS/pcap_rtps_packets.txt" 2>/dev/null || true
  else
    tcpdump -nn -vv -r "$pcap" 'udp' 2>/dev/null | head -n 200 \
      >>"$ANALYSIS/pcap_rtps_packets.txt" || true
  fi
done

printf 'PCAP_ANALYSIS=PASS files=%s tshark=%s capinfos=%s\n' \
  "${#pcaps[@]}" "$(command -v tshark || printf unavailable)" \
  "$(command -v capinfos || printf unavailable)"

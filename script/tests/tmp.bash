while true; do
  python3 robstride_edulite05_lab.py scan --start 0x00 --end 0xFF
  ip -details -statistics link show can0 | grep -A1 -E 'can state|re-started|RX:|TX:'
  sleep 1
done

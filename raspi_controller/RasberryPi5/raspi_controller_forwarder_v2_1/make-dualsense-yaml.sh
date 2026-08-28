#!/usr/bin/env bash
set -Eeuo pipefail

OUT="${1:-dualsense.yaml}"
UINPUT_BIN="$(command -v uinput || true)"
if [[ -z "$UINPUT_BIN" ]]; then
  echo "uinput from Interception Tools is not installed." >&2
  exit 127
fi

find_gamepad() {
  local sys event name dev yaml
  for sys in /sys/class/input/event*; do
    [[ -r "$sys/device/name" ]] || continue
    event="$(basename "$sys")"
    dev="/dev/input/$event"
    [[ -r "$dev" ]] || continue
    name="$(cat "$sys/device/name" 2>/dev/null || true)"
    case "$name" in
      *DualSense*|*Wireless\ Controller*)
        yaml="$($UINPUT_BIN -p -d "$dev" 2>/dev/null || true)"
        if grep -q 'BTN_SOUTH' <<<"$yaml" && grep -q 'ABS_X' <<<"$yaml"; then
          echo "$dev"
          return 0
        fi
        ;;
    esac
  done
  return 1
}

DEV="$(find_gamepad || true)"
if [[ -z "$DEV" ]]; then
  echo "DualSense gamepad event node was not found. Connect the controller and retry." >&2
  exit 1
fi

echo "Using $DEV" >&2
"$UINPUT_BIN" -p -d "$DEV" | sed -E 's/^NAME:.*/NAME: Remote DualSense/' > "$OUT"
echo "Created $OUT" >&2
echo "Copy this file to the RDK as /etc/controller/dualsense.yaml" >&2

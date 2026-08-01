#!/bin/bash
# =======================================================
# DualSense Bluetooth ペアリングスクリプト
#
# 仕様:
#   1) DualSense の MAC アドレス候補をスキャンして表示
#   2) 使いたい MAC アドレスを手入力
#   3) pair / trust / connect を実行
#
# メモ:
#   - 実行前に DualSense をペアリングモードにしてください
#   - 実行方法: bash pair_dualsense.sh
# =======================================================

set -u

SCAN_SEC="${SCAN_SEC:-10}"
ADDR_FILE="$(dirname "$0")/.dualsense_addr"
MAC_REGEX='^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$'

print_header() {
    echo ""
    echo "========================================"
    echo "  DualSense Bluetooth ペアリング"
    echo "========================================"
}

ensure_bluetooth_tools() {
    echo ""
    echo "[1/4] Bluetooth パッケージを確認中..."

    if ! command -v bluetoothctl >/dev/null 2>&1; then
        echo "       -> bluez が未インストールです。インストールします..."
        sudo apt update
        sudo apt install -y bluez bluez-tools
        sudo systemctl enable bluetooth
        sudo systemctl start bluetooth
        sleep 2
        echo "       OK Bluetooth インストール完了"
    else
        echo "       OK bluez はインストール済みです"
    fi

    if ! sudo systemctl is-active --quiet bluetooth; then
        sudo systemctl start bluetooth
        sleep 2
    fi
    echo "       OK Bluetooth サービス動作中"
}

ensure_expect() {
    echo ""
    echo "[2/4] expect を確認中..."

    if ! command -v expect >/dev/null 2>&1; then
        echo "       -> expect をインストールします..."
        sudo apt install -y expect
        echo "       OK expect インストール完了"
    else
        echo "       OK expect はインストール済みです"
    fi
}

show_pairing_instructions() {
    echo ""
    echo "[3/4] DualSense をペアリングモードにしてください"
    echo ""
    echo "【手順】"
    echo "  1. DualSense の電源を完全に切る"
    echo "     （PSボタンを10秒長押し -> ライトが消えるまで）"
    echo ""
    echo "  2. PS ボタン + SHARE ボタンを同時に約3秒長押し"
    echo "     -> ライトバーが素早く点滅 = ペアリングモード"
    echo ""
    echo "  3. RDK X5と DualSense を 1m 以内に近づける"
    echo ""
    read -r -p "  準備ができたら Enter を押してください..."
}

scan_dualsense_candidates() {
    local scan_output=""
    local devices_output=""
    local info_output=""
    local addr=""

    bluetoothctl power on >/dev/null 2>&1 || true
    bluetoothctl agent NoInputNoOutput >/dev/null 2>&1 || true
    bluetoothctl default-agent >/dev/null 2>&1 || true
    bluetoothctl pairable on >/dev/null 2>&1 || true
    bluetoothctl discoverable on >/dev/null 2>&1 || true

    scan_output="$(timeout --foreground "${SCAN_SEC}s" bluetoothctl scan on 2>&1 || true)"
    bluetoothctl scan off >/dev/null 2>&1 || true

    devices_output="$(bluetoothctl devices 2>&1 || true)"

    while IFS= read -r addr; do
        [ -z "$addr" ] && continue
        info_output="${info_output}$(bluetoothctl info "$addr" 2>&1 || true)"$'\n'
    done <<EOF
$(printf '%s\n' "$devices_output" | sed -nE 's/^Device (([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}).*/\1/p')
EOF

    printf '%s\n%s\n%s\n' "$scan_output" "$devices_output" "$info_output"
}

strip_ansi() {
    local esc
    esc="$(printf '\033')"
    printf '%s\n' "$1" | sed -E "s/\r//g; s/${esc}\[[0-9;?]*[ -/]*[@-~]//g"
}

normalize_mac() {
    printf '%s\n' "$1" | tr '[:lower:]' '[:upper:]'
}

extract_candidates() {
    # 名称が出ない環境では bluetoothctl info の複数行ブロックからも拾う
    local line
    local current_mac=""
    local seen=""

    remember_candidate() {
        local mac="$1"
        [ -z "$mac" ] && return 0
        if ! printf '%s\n' "$seen" | grep -Fxq "$mac"; then
            printf '%s\n' "$mac"
            seen="${seen}${seen:+$'\n'}${mac}"
        fi
    }

    while IFS= read -r line; do
        local found_mac=""
        found_mac="$(printf '%s\n' "$line" | sed -nE 's/^Device (([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}).*/\1/p' | tr '[:lower:]' '[:upper:]')"
        if [ -n "$found_mac" ]; then
            current_mac="$found_mac"
            case "$line" in
                *"DualSense Wireless Controller"*|*"Wireless Controller"*)
                    remember_candidate "$current_mac"
                    ;;
            esac
            continue
        fi

        case "$line" in
            *"Icon: input-gaming"*|*"Class: 0x00002508"*)
                remember_candidate "$current_mac"
                ;;
        esac
    done <<EOF
$(strip_ansi "$1")
EOF

    if [ -f "$ADDR_FILE" ]; then
        remember_candidate "$(normalize_mac "$(cat "$ADDR_FILE")")"
    fi
}

ask_mac_address() {
    local candidates="$1"
    local answer=""
    local line=""
    local -a candidate_list=()

    echo ""
    echo "  スキャン結果（候補MACアドレス）:"
    if [ -n "$candidates" ]; then
        mapfile -t candidate_list <<< "$candidates"
        local idx=1
        for line in "${candidate_list[@]}"; do
            [ -z "$line" ] && continue
            printf '    %d) %s\n' "$idx" "$line"
            idx=$((idx + 1))
        done

        echo ""
        for line in "${candidate_list[@]}"; do
            [ -z "$line" ] && continue
            while true; do
                if ! read -r -p "  候補 ${line} を使いますか? [y/n]: " answer; then
                    echo ""
                    echo "  [失敗] 入力を読み取れませんでした。"
                    return 1
                fi
                answer="$(printf '%s' "$answer" | tr '[:upper:]' '[:lower:]')"
                case "$answer" in
                    y|yes)
                        DS_ADDR="$(normalize_mac "$line")"
                        echo ""
                        echo "  候補MACを選択しました: ${DS_ADDR}"
                        return 0
                        ;;
                    n|no)
                        break
                        ;;
                    *)
                        echo "  y か n で入力してください。"
                        ;;
                esac
            done
        done
    else
        echo "    候補を検出できませんでした"
    fi

    echo ""
    echo "  使いたい MAC アドレスを手入力してください（例: AA:BB:CC:DD:EE:FF）"
    read -r -p "  MACアドレス: " DS_ADDR
    DS_ADDR="$(normalize_mac "$DS_ADDR")"

    if ! printf '%s\n' "$DS_ADDR" | grep -Eq "$MAC_REGEX"; then
        echo ""
        echo "  [失敗] MACアドレス形式が不正です。"
        echo "  形式: AA:BB:CC:DD:EE:FF"
        return 1
    fi

    return 0
}

pair_trust_connect() {
    expect -c "
        set timeout 60
        set target_addr [string toupper \"${DS_ADDR}\"]
        set prompt_re {([^\n]*#\s*)}
        match_max 100000

        proc wait_prompt {prompt_re} {
            expect {
                -re \$prompt_re { return 0 }
                timeout { exit 1 }
                eof { exit 1 }
            }
        }

        spawn bluetoothctl
        wait_prompt \$prompt_re

        send -- \"power on\r\"
        wait_prompt \$prompt_re
        send -- \"agent NoInputNoOutput\r\"
        wait_prompt \$prompt_re
        send -- \"default-agent\r\"
        wait_prompt \$prompt_re
        send -- \"pairable on\r\"
        wait_prompt \$prompt_re
        send -- \"discoverable on\r\"
        wait_prompt \$prompt_re
        send -- \"scan on\r\"

        expect {
            -re \"Device ${DS_ADDR}\" {}
            -re {Discovery started|\[NEW\]|\[CHG\]} { exp_continue }
            -re {[^\n]*#\s*} { exp_continue }
            timeout {
                send_user \"\\n\\[WARN\\] 入力した MAC アドレス ${DS_ADDR} がスキャン中に見つかりませんでした。\\n\"
                exit 1
            }
            eof { exit 1 }
        }

        # スキャンを止めてからペアリングする方が not available を回避しやすい
        send -- \"scan off\r\"
        expect {
            -re {Discovery stopped|[^\n]*#\s*} {}
            timeout { wait_prompt \$prompt_re }
            eof { exit 1 }
        }

        # 別ホストで再ペアリングした後に古いボンド情報が残ると失敗しやすい。
        # 先に対象アドレスの接続/ボンドを掃除してから再ペアリングする。
        send -- \"disconnect ${DS_ADDR}\r\"
        expect {
            -re {Successful disconnected|Device .* not connected|Failed to disconnect: .*} {}
            -re {[^\n]*#\s*} {}
            timeout { wait_prompt \$prompt_re }
            eof { exit 1 }
        }

        send -- \"remove ${DS_ADDR}\r\"
        expect {
            -re {Device has been removed|Device .* not available|Failed to remove: org.bluez.Error.DoesNotExist|Failed to remove: .*} {}
            -re {[^\n]*#\s*} {}
            timeout { wait_prompt \$prompt_re }
            eof { exit 1 }
        }

        # remove 後はデバイスオブジェクトが消えるため、再スキャンで再検出してから pair する
        send -- \"scan on\r\"
        expect {
            -re \"Device ${DS_ADDR}\" {}
            -re {Discovery started|\[NEW\]|\[CHG\]} { exp_continue }
            -re {[^\n]*#\s*} { exp_continue }
            timeout {
                send_user \"\\n\\[WARN\\] remove 後に MAC ${DS_ADDR} を再検出できませんでした。\\n\"
                exit 1
            }
            eof { exit 1 }
        }
        send -- \"scan off\r\"
        expect {
            -re {Discovery stopped|[^\n]*#\s*} {}
            timeout { wait_prompt \$prompt_re }
            eof { exit 1 }
        }

        set paired 0
        for {set i 1} {\$i <= 3 && \$paired == 0} {incr i} {
            send -- \"pair ${DS_ADDR}\r\"
            expect {
                -re {Confirm passkey.*yes/no} { send -- \"yes\r\"; exp_continue }
                -re {Authorize service.*yes/no} { send -- \"yes\r\"; exp_continue }
                -re {Request confirmation} { send -- \"yes\r\"; exp_continue }
                -re {Accept pairing.*yes/no} { send -- \"yes\r\"; exp_continue }
                -re {Pairing successful|Device .* already paired|Failed to pair: org.bluez.Error.AlreadyExists} { set paired 1 }
                -re {Device .* not available|Failed to pair: org.bluez.Error.NotAvailable|Failed to pair: org.freedesktop.DBus.Error.UnknownObject} {
                    if {\$i < 3} {
                        send_user \"\\n\\[WARN\\] Device object unavailable。再スキャンして再試行します (\$i/3)...\\n\"
                        send -- \"scan on\r\"
                        expect {
                            -re \"Device ${DS_ADDR}\" {}
                            -re {Discovery started|\[NEW\]|\[CHG\]} { exp_continue }
                            timeout {}
                        }
                        send -- \"scan off\r\"
                        expect {
                            -re {Discovery stopped|[^\n]*#\s*} {}
                            timeout {}
                        }
                    } else {
                        exit 1
                    }
                }
                timeout { if {\$i >= 3} { exit 1 } }
                eof { exit 1 }
                -re {Failed to pair: .*} { exit 1 }
            }
            wait_prompt \$prompt_re
        }

        if {\$paired == 0} {
            exit 1
        }

        send -- \"trust ${DS_ADDR}\r\"
        expect {
            -re {trust succeeded|Changing .* succeeded|Device .* is already trusted} {}
            timeout { exit 1 }
            eof { exit 1 }
            -re {Failed to trust: .*} { exit 1 }
        }
        wait_prompt \$prompt_re

        send -- \"connect ${DS_ADDR}\r\"
        expect {
            -re {Connection successful|Device .* already connected|Failed to connect: org.bluez.Error.AlreadyConnected} {}
            timeout { exit 1 }
            eof { exit 1 }
            -re {Failed to connect: .*} { exit 1 }
        }
        wait_prompt \$prompt_re

        send -- \"exit\r\"
    " 2>&1
}

main() {
    local scan_log=""
    local candidates=""
    local pair_log=""

    print_header
    ensure_bluetooth_tools
    ensure_expect
    show_pairing_instructions

    echo ""
    echo "[4/4] DualSense 候補をスキャン中...（最大 ${SCAN_SEC}秒）"
    scan_log="$(scan_dualsense_candidates)"
    candidates="$(extract_candidates "$scan_log")"

    if ! ask_mac_address "$candidates"; then
        echo ""
        echo "  再度このスクリプトを実行してください。"
        exit 1
    fi

    echo ""
    echo "  入力された MAC アドレス: ${DS_ADDR}"
    echo "  ペアリングと接続を実行します..."

    pair_log="$(pair_trust_connect)"
    if [ $? -ne 0 ]; then
        echo ""
        echo "  [失敗] ペアリングまたは接続に失敗しました。"
        echo ""
        echo "  ---- 確認事項 ----"
        echo "  ・DualSense がまだペアリングモードで点滅しているか"
        echo "  ・入力した MAC アドレスが正しいか"
        echo "  ・必要なら bluetoothctl remove ${DS_ADDR} 後に再試行"
        echo ""
        echo "  ---- 重要ログ ----"
        printf '%s\n' "$pair_log" | grep -Ei 'not available|failed to|timed out|error' | tail -10 || true
        echo ""
        echo "  ---- デバッグログ（末尾） ----"
        printf '%s\n' "$pair_log" | tail -30
        exit 1
    fi

    printf '%s\n' "${DS_ADDR}" > "${ADDR_FILE}"

    echo ""
    echo "  OK ペアリング完了"
    echo "  OK 信頼済みに登録"
    echo "  OK 接続完了"
    echo "  OK 接続先アドレスを保存: ${ADDR_FILE}"
    echo ""
    echo "========================================"
    echo "  OK ペアリング完了"
    echo "  次回以降は PS ボタンで自動接続できます。"
    echo "========================================"
}

main

#!/bin/bash

target="RoDEP-5Ghz"
connection=$(nmcli -t -f active,ssid dev wifi | awk -F: '$1 == "yes" {print substr($0, 5)}')

if [ "$connection" = "$target" ]; then
    echo "接続中: $connection"
else
    echo "$target に切り替えます"
    sudo nmcli connection up "$target"
fi

# 9マスターゲットグリッド状態
# 0: まだ倒していない (STANDING)
# 1: 狙っている (TARGET)
# 2: 倒したもの / 倒れ判定 (FALLEN)

uint8 STANDING = 0
uint8 TARGET = 1
uint8 FALLEN = 2

uint8[9] states

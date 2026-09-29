# driving_robot - 追加アイデアメモ

## このドキュメントの位置づけ

このドキュメントは壁打ち・ブレストの成果物で、正式な仕様書ではありません。

## 案-モデルの適合
現在のモデルは以下になっている
```
features:
  h0_s: 0.5
  h1_s: 1.0        # レジーム判定（要求加速度 = dv_h1 / h1_s）に使うので use_h1 は false 不可
  h2_s: 2.0
  h3_s: 3.0
  use_h0: false
  use_h1: true
  use_h2: true
  use_h3: true
  p1_s: 0.5
  p2_s: 1.0
  use_p1: true
  use_p2: true
  past_as_delta: false
  use_v0_sq: true
  use_dv1_x_v0: true
```
h0_s ~ h3_sについて、固定値で`true` or `false`で使うか使わないかをユーザーが判断\
これを固定値ではなく、0.1~3.0(0.1刻み)で自動で計算するようにしてもっとも成績(MAE,RMSSE,R^2)数値をモデルとして保存する\
自動計算には`true``flase`も使う。使わないモデルの方が成績が良ければそちらを使う

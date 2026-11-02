# プロトコル・単位・検証状態の根拠

## 証拠の区別

| 区分 | 今回記録する内容 | そこからは断定しないこと |
| --- | --- | --- |
| ユーザーの過去の実測報告 | 「ポート2」、4 Mbps/8N1、17/16バイト、ID 0/1/2、mode 0/1、温度26〜28℃相当、エラー0、数値変化、純正マザーボードなしの取得・回転、過去の追従失敗 | 今回の実機再現、OSのデバイス名、型式/ファームウェア全般への適用 |
| 今回コードで再検証 | 提示TX/RX各5件のCRC一致、signed値/位置候補の取り出し、公式SDKの生成/解析結果との比較、仮想通信の異常処理 | 純正側の単位、実際の回転・停止、ゲイン安定性 |
| 開発用仕様を適用した仮説 | 純正のフィールド配置、rad/rad/s/N·m換算、低位3bitエラー、減速比6.33、ゲイン換算 | M8018固有の正式仕様、トルクが実測値か推定値か、補助フィールドの意味 |
| 実機検証が必要 | 軸・正方向・ゼロ・倍率、速度換算、トルク源/倍率、減速比、ゲイン、許容周期、終了動作、温度/エラーの意味 | CRCだけでこれらが確定すること |

5組の返信の温度候補はすべて28、エラー候補はすべて0です。26〜28℃の報告範囲を新たに再現したわけではありません。位置変化などは保存フレームの差として確認しただけです。

## 公式参照と固定版

一次資料は [unitreerobotics/unitree_actuator_sdk](https://github.com/unitreerobotics/unitree_actuator_sdk)、固定コミット `5b79a42d81cd69adac1367db79efdb972a898fd1`。既存サブモジュールを更新・変更せず使用しました。

- [GO-M8010-6プロトコル定義](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/include/unitreeMotor/include/motor_msg_GO-M8010-6.h)：16–20行mode/id、26–34行指令、40–49行返信、60–80行フレーム構造。これが今回参照した公式の公開プロトコル定義です。同梱に完全なプロトコルPDFはなく、未入手のマニュアルを参照済みとは扱いません。
- [公式CRC実装](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/include/crc/crc_ccitt.h)：反転多項式0x8408と表、右シフトによる更新、最終XORなし。
- [SDK公開API](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/include/unitreeMotor/unitreeMotor.h)：MotorCmd / MotorData、共通構造体。ここにある `gyro` / `acc` / `LW` / `Acc` / `footForce` の存在は、GO-M8010-6や純正からそれらの実測値が取得できる証拠ではありません。本ドライバーはIMU等を公開せず、補助値はrawとビット候補のまま保持します。
- [公式README](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/README.md)：指令はローター側、出力軸ゲインはr²で割る。A1/B1専用補正26.07や100をGO系へ流用しません。
- [GO-M8010-6公式例](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/example/example_goM8010_6_motor.cpp)：機種選択、速度と減速比の関係。公式例はモーターを動かすため実行していません。
- [公式Linux64ライブラリ](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/lib/libUnitreeMotorSDK_Linux64.so)：SHA-256 `d51bc5f10cd757458ff4c34fa3bf6ecb0bb28c6c331de335aaaa527c68850c68`。この固定版では `MotorCmd::modify_data` / `MotorData::extract_data` のソースは非同梱です。`scripts/official_vectors.cpp` はSerialPortを生成せず、この2関数と `queryGearRatio` だけを呼びます。変換式の切り捨ては当該バイナリの命令と独立出力でも確認しました。公式実装のソースが公開されていると誤記しません。

既存のopenbiped版（`76443343d2f4872fb9dbeda0f9c18b386d6b536b`）の `src/unitreeMotor.cpp` は純正CRC仮説の二次資料です。公式SDKと同一視せず、新ドライバーの正しさの独立根拠には使っていません。同版は丸め処理を行い、今回呼び出した公式バイナリの切り捨てとは異なります。既存のID変更研究資料は履歴として保持していますが、新ドライバーにはその操作を実装していません。

## バイト・ビット配置

全多バイト整数はリトルエンディアン。C/C++ビットフィールドをPythonの構造体として模倣せず、明示的にマスクします。

| オフセット | TX（17バイト） | RX（16バイト） |
| --- | --- | --- |
| 0..1 | fe ee | fd ee（提示返信と独立ベクトルで確認） |
| 2 | ID=下位4bit、mode=bit4..6、bit7予約 | 同配置 |
| 3..4 | torque int16 | torque候補 int16 |
| 5..6 | speed int16 | speed候補 int16 |
| 7..10 | position int32 | position候補 int32 |
| 11 | kpの下位バイト | temperature候補 int8 |
| 11..12 | kp uint16 | 12..13は補助raw uint16 |
| 13..14 | kd uint16 | 12..13: error候補=raw & 7、補助bit3..14、予約bit15 |
| 15..16 / 14..15 | TX CRC16 | RX CRC16 |

公式の `force :12` の配置は公式RXデコーダーへ合成入力を渡して確認し、bit3..14の値3456と一致しました。しかし値の配置確認は、GO-M8010-6に実際の足先力センサーがあることや、純正返信のこの領域が足先力であることを証明しません。`auxiliary_raw` と候補ビットを保持し、足先力とは命名しません。

例：最初のID0 RXの `38 24 00 00` は位置候補9272。mode1 RXの `f8 ff` は速度候補−8。最終RXの `13 00` はトルク候補19。提示TXの `80 02` / `40 00` はゲインraw640/64です。

送信可能IDは0..14の明示指定のみ。ID15ブロードキャスト、未知mode、mode2のエンコーダ校正、mode7、ID変更、設定・フラッシュ操作は拒否します。オフラインのdecodeは未知値をそのまま表示し、送信には利用しません。

## CRCの再検証

| パラメーター | 開発用 | 純正用候補（提示フレームで検証） |
| --- | --- | --- |
| 正規多項式 / 反転多項式 | 0x1021 / 0x8408 | 同じ |
| 処理 | byte XOR後、LSBから右シフトを8回 | 同じ |
| 初期値 | 0x0000 | 0x2CBB |
| 最終XOR | 0x0000 | 0x0000 |
| TX対象 | ヘッダーを含むbytes 0..14 | 同じ、5/5一致 |
| RX対象 | ヘッダーを含むbytes 0..13 | 同じ、5/5一致 |
| CRC格納 | little endian | little endian |

`tx_crc` と `rx_crc` は別の設定オブジェクトです。同じ方式と決め打ちした証明ではなく、別々の入力列に対して再計算し比較しました。純正用で初期値0を使うと全10件で不一致になります。

計算結果：TX `C344, C6E9, C81E, 01F9, 74AB`、RX `1E7F, 5F49, 3D17, 5C4B, F7D6`。各16進値は整数表記で、ワイヤ上では下位バイトから格納します。全10件のヘッダー・データ・CRCを含む1320箇所の1ビット反転をそれぞれ試し、すべて拒否しました（ヘッダー変更はヘッダー検査、それ以外はCRC検査）。記録は `validation/report.json`。

有限個の例との一致は全ファームウェア・全フレームに対する仕様確定ではありません。CRC一致は意味・単位・鮮度・実際の制御受理を証明しません。

## 変換式と公式独立期待値

公式バイナリの入力はfloat32。`q` / `dq` / `kp` / `kd` は入力をfloat32化した後、以下を倍精度で計算し**ゼロ方向へ切り捨て**ます。τ×256はfloat32演算です。`2π` の代わりに公式の `6.2832` を使用します。高精度πに置換すると境界で生成フレームが変わります。

| 値 | SDKローター入力→TX raw | RX raw→SDKローター換算 |
| --- | --- | --- |
| 位置 q [rad] | trunc(float32(q) / 6.2832 × 32768) | raw × 6.2832 / 32768 |
| 速度 dq [rad/s] | trunc(float32(dq) / 6.2832 × 256) | raw × 6.2832 / 256 |
| トルク τ [N·m] | trunc(float32(τ × 256)) | raw / 256 |
| kp, kd | trunc(float32(gain) / 25.6 × 32768) | 返信に該当ゲインなし |

プロトコルヘッダーには速度q7・rad/s等のコメントがありますが、その記載だけからraw/128と判断していません。公開API・公式の生成/解析結果と照合しています。float32 RXの丸め差はPythonでは約1e−7以下の差を許容して比較します。Pythonの換算値はdoubleで保持します。

SDK公開APIのクランプ範囲（固定バイナリで確認）はローター側で位置±411774、速度±804、トルク±127.99、kp/kd 0..25.599です。本実装は不正値・範囲超過をクランプせず拒否し、さらにユーザー設定の運用範囲と整数パック範囲を検査します。この数値は純正の性能・使用可能範囲を示しません。

`tests/fixtures/official_vectors.json` と `scripts/verify_official.py` は、Pythonエンコーダーで作った期待値ではありません。公式共有ライブラリを直接呼び出した4つのTXと1つの合成RX解析を固定しています。特にq=6.2832（float32）→raw32767、q=−0.1→raw−521、dq=0.1→raw4が丸め実装との違いを検出します。RX合成ベクトルはID2/mode1、q=1.77788794、dq=−0.1963499933、τ=−1、temp28、error5。実機から取得したベクトルではありません。

純正に上記換算を適用するのは仮説です。標準のAPIは `axis="rotor"`、減速比換算なしです。出力軸を選ぶ場合の理想換算は `q_out=q_rot/r`、`dq_out=dq_rot/r`、`τ_out=τ_rot*r`、`kp_rot=kp_out/r²`、`kd_rot=kd_out/r²`。**r=6.33は公式GO-M8010-6のqueryGearRatioの値であり、純正M8018系では未検証**です。効率・摩擦・荷重でトルク関係も変わります。

実測角度校正は選択した軸で `angle = slope*raw + intercept` を最小二乗し、`direction=sign(slope)`、`angle_rad_per_count=abs(slope)`、`zero_raw=−intercept/slope` とします。往復差・残差・別方向の傾きを報告します。測定span外の外挿、バックラッシュ、ラップ、外れ値、温度/負荷依存を自動解決しません。校正係数を設定すると位置の順変換/逆変換の両方に使用します。速度・トルク係数は別設定で、自動的に校正済みにはしません。

## 検証範囲と残る確認

`python -m unittest discover -s tests -v` と `python scripts/offline_validation.py` は実機未接続で実行します。提示データの独立期待値、公式SDK出力、仮想受信と異常処理を別々に確認します。仮想プラントは指令通りに動くという合成モデルなので、制御の物理的な成立・安定性の根拠にはなりません。

まだ確認していないのは、実機の電気的接続・変換器の挙動、Windowsでの4 Mbps、M8018型式/ファームウェア別適合、正方向・ゼロ・軸・減速比・換算係数、温度/エラー/補助の意味、トルク源、ゲイン、タイミングと追従性、終了指令の機械的挙動です。過去のID0追従失敗は解決済みとは扱いません。

# Go2 Air Pythonドライバー（実験用）

**今回完了したのはコード実装とオフライン検証です。実機への送信・回転確認はしていません。** 純正M8018系の倍率・減速比・ゲイン・終了時の挙動は未確定です。[根拠と変換式](protocol-evidence.md)を先に確認してください。

## Windowsでの導入

Python 3.10以降とGitを使用します。PowerShellでは仮想環境のactivateは不要です。

```powershell
git clone --recurse-submodules https://github.com/88keisuke/go2_motor_research.git
cd go2_motor_research
# PRのブランチを使う場合
git switch feature/go2-air-python-driver
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[serial]"
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m go2driver --help
.\.venv\Scripts\python.exe -m go2driver ports
```

以降の `python` はこの仮想環境のPythonに読み替えてください。`go2motor` も同じCLIです。オフライン解析は標準ライブラリだけで動き、pyserialをimportしません。`ports` はpyserialを使った列挙のみで、ポートを開きません。通常起動・import・テスト・dry-runではデバイスを開きません。

「ポート2」は過去の環境の呼称であり、COM2とは断定していません。`ports` の結果で確認した `COM7` など、任意のOSデバイスパスを `--device` に指定します。Linuxなら `/dev/ttyUSB0` などです。変換器・ドライバーが4 Mbps、8N1、半二重RS-485の送受切替に対応することを別途確認してください。RTS/CTS、XON/XOFF、DSR/DTRフロー制御は無効です。RTSによる方向制御が必要な変換器は現実装の対象外です。Windowsで4 Mbpsが利用できるかは実機未検証です。

## オフラインで最初に行う操作

```powershell
python -m go2driver decode "fd ee 10 00 00 f8 ff 37 24 00 00 1c 00 6c 4b 5c"
python -m go2driver decode "fe ee 00 00 00 00 00 00 00 00 00 00 00 00 00 44 c3" --direction tx
python -m go2driver replay validation/supplied-captures.jsonl
python -m go2driver replay validation/virtual-multi.jsonl
python -m go2driver status --ids 0 1 2 --dry-run
python -m go2driver move --id 0 --target 0.1 --duration 2 --period 0.02 --config examples/config.json --dry-run
python -m go2driver velocity --id 0 --target 0.05 --duration 2 --config examples/config.json --dry-run
python -m go2driver torque --id 0 --target 0.02 --duration 2 --config examples/config.json --dry-run
python -m go2driver multi --targets examples/targets.json --duration 2 --config examples/config.json --dry-run
```

`--send` を省略すると常にdry-runです。送信予定フレームを16進表示します。`--preview-start` は表示用の仮の開始角度（既定0）で、実機制御には使用しません。位置は選択した軸の**絶対角度rad**、速度はrad/s、トルクはN·mです。`examples/targets.json` は小さな数値の書式例であり、その角度が機械的に許容されることを示しません。dry-runは実際の送受信時間を予測しません。

上位APIの最小例（この例は通信しません）：

```python
from go2driver import Command, Units, PROFILES
from go2driver.protocol import encode
cmd = Command(motor_id=0, position=0.1, velocity=0.0, torque=0.0, kp=0.1, kd=0.01)
print(encode(cmd.raw(Units(axis="rotor")), PROFILES["stock"]).hex(" "))
```

実機APIは `SerialTransport(device, allow_send=True)` → `MotorBus` → `MotorController.run({ID: target}, kind="position"/"velocity"/"torque")` の順です。`MotorBus.exchange` は低水準の1要求APIで、CRC・整数範囲・モード・IDを検証します。位置等の運用範囲・軌道・異常監視・終了処理は必ず `MotorController` を通します。複数スレッドのバス呼出しはロックで直列化しますが、複数コントローラーの同時運転はサポートしません。

## 状態取得と実機制御の入口

状態取得もモード0要求の**送信**であり、受信専用の観測ではありません。モード0の機械的な挙動は未検証です。次は使用者が実機・電源・固定・可動範囲を確認した後に手動で実行する書式例です。この開発作業では実行していません。

```powershell
python -m go2driver status --ids 0 1 2 --device COM7 --send --log capture-stock.jsonl --config examples/config.json
```

表示にはraw、換算値、軸、未校正表示、仮説の根拠、温度候補・エラー候補・補助raw値が含まれます。ステータス異常で継続しません。低位3bitは開発用配置からのエラー候補で、純正での意味は未確認です。補助値を足先力とは表示しません。

運動制御は、設定の `termination.verified` が `true` で、`termination.evidence` に**このモーター・ファームウェア・変換器で終了指令を確認した記録**がある場合に限り可能です。配布する設定は `false` です。単に送信を通すために変更せず、確認した事実を記入してください。既存返信はモード0の応答を示しますが、安全な停止・保持・無励磁を証明しません。終了方式はモード0、全制御値0に限定し、未知モードやID変更を試すコードは含めません。

使用者が校正・範囲・終了動作を確認した設定を `my-config.json` として保存した後の書式例：

```powershell
python -m go2driver move --id 0 --target 0.1 --duration 3 --period 0.02 --kp 0.1 --kd 0.01 --device COM7 --send --config my-config.json --log move-001.jsonl
python -m go2driver multi --targets examples/targets.json --duration 3 --period 0.02 --device COM7 --send --config my-config.json --log multi-001.jsonl
python -m go2driver velocity --id 0 --target 0.05 --duration 2 --ramp 0.5 --device COM7 --send --config my-config.json --log speed-001.jsonl
python -m go2driver torque --id 0 --target 0.02 --duration 2 --ramp 0.5 --device COM7 --send --config my-config.json --log torque-001.jsonl
```

速度制御はkp=0と設定kd、トルク制御はkp=kd=0でランプを生成します。純正側でその通りの特性になるかは未検証です。速度・トルク制御中も位置範囲を監視します。各設定値は選択軸の単位で、ゲインも出力軸指定なら理想減速機のr²換算を使います。開発用と同じ応答・安定性になるとは仮定しません。

設定は `units` / `limits` を全IDの既定値とし、`motors.{ID}.units` / `.limits` で個別上書きできます。`direction` は±1、`zero_raw` は生エンコーダ値、`gear_ratio` は任意の正値です。`gear_ratio_verified` は根拠の状態表示で、校正証明ではありません。位置・速度・トルクの実測係数は `angle_rad_per_count` / `velocity_rad_s_per_count` / `torque_nm_per_count` を選択軸の係数として設定します。角度係数には `angle_calibration_evidence` が必要です。係数の符号は `direction` にまとめます。

範囲の既定値と例の値は定格・安全保証値ではありません。未校正なら物理量による追従監視自体が誤る可能性があります。用途に適した値と機械的な保護が必要です。

位置軌道は全対象の現在位置を取得してから、端点の速度・加速度が0の5次曲線を生成します。要求周期ではなく送信時の経過時間を使い、ピーク速度（1.875×移動角/時間）も検証します。順次送受信が遅れた場合は実時間に追従して次の予定時刻に進み、遅れた指令をまとめて送ることはしません。最後に目標位置・速度0の指令を送り、終了指令を試みます。位置保持の継続は保証しません。

通信断、受信タイムアウト、連続CRC異常（既定3）、モード/IDの問題、温度、エラー候補、追従不足、速度・トルク・位置の超過、Ctrl+Cで中断します。ID違いは保存して対象返信を期限まで待ちます。運動指令の自動再送はしません。中断時は設定で確認済みとされたモード0を**全対象IDそれぞれ**に1回試し、成功・失敗・返信を保存します。通信断では届かないことがあります。`acknowledged` は対応するCRC正常返信の取得を意味し、`mechanical_stop_proven` は常にfalseです。OS強制終了、電源断、シリアルドライバーの停止では終了処理を実行できません。

## 次に行う角度校正

1. 軸（ローター/出力軸）を決め、モーターIDと固定条件を記録します。自動回転は使わず、既知の角度に置いたときの位置rawを取得します。モード0での状態取得にも上記の留意点があります。
2. 同じIDについてゼロ・複数の既知角度を往路/復路で繰り返し測り、`examples/angle_samples.json` の形式に記録します。例の値は合成データです。角度はrad、`leg` は `outbound` / `return`、同一角度は同じ数値で記入します。
3. 次を実行し、傾き・ゼロ・符号、各測定残差、RMS/最大残差、同一角度の往復差、往路/復路別傾きを確認します。自動で合格判定はしません。温度・荷重・バックラッシュ・角度の折返しを変えた再測定も必要です。

```powershell
python -m go2driver calibrate measured-angles-id0.json --axis output --output calibration-id0.json
python -m go2driver calibrate examples/angle_samples.json --axis output --output demo-calibration.json
python -m go2driver speed-check examples/speed_samples.json --calibration demo-calibration.json
```

4. 校正レポートの `units` を該当IDの `motors.{ID}.units` に反映し、根拠を記録します。他のIDへ無条件に流用しません。角度の校正から減速比・トルク・ゲインの正しさは導けません。
5. 同じIDの時刻・位置raw・速度rawを `examples/speed_samples.json` 形式で保存し、`speed-check` に渡します。JSONLの `state.timestamp`、`state.raw.position`、`state.raw.speed` が対応項目です。ID別に分離してください。位置差分速度と速度フィールド両端平均の差、dt、ノイズ見積りを表示します。`--min-dt` / `--max-dt` から外れた間隔は除外、`--raw-noise` で位置ノイズの標準偏差相当（count）を指定できます。速度係数は自動確定しません。

## ログ、複数台の時刻差、開発用との比較

JSONLはUTF-8、1行1イベント、schema=1です。既存ファイルを上書きしません。

- `session`：プロファイル、全IDの換算設定、タイムアウト、CRC制限。
- `tx`：全送信バイト、ID、上位指令、ホスト送信開始時刻。
- `rx_chunk`：破損・ノイズ・部分受信も含む全受信バイトと受信時刻。
- `rx` / `state`：CRC正常フレームとraw・換算値・検証状態。
- `transaction`：ID、成否、往復遅延、受信時刻/失敗理由。
- `tracking` / `cycle`：追従誤差、各IDの送信時刻差、周期超過。
- `crc_error` / `id_mismatch` / `abort` / `termination`：異常と終了指令の結果。

時刻はUTCの壁時計とホスト高分解能単調時計（perf_counter）です。WindowsのPython 3.10/3.12でも短いID間時刻差を測定できる時計を使い、sessionに時計名と分解能を保存します。RX時刻はシリアルread完了時で、UART最終ビットや内部サンプリング時刻ではありません。TXはwrite呼出し前で、USB転送・変換器のバッファ遅延を含みます。プロトコルにはシーケンス番号がなく、同ID・同modeの遅延返信を完全には識別できません。各要求前に古い入力と部分フレームを破棄します。ホスト側タイムアウトは設定値+1回のread時間程度の遅れを持ちます。

`replay` は生TX/RXを再解析します。`rx_chunk` があるログではそちらを使い二重計上しません。各IDの実測返信間隔・往復遅延の平均/標準偏差/最小/最大、要求失敗数、サイクル内ID間時刻差を表示します。集計には初期・終了の状態取得も含むので、運動中だけを調べる場合は `transaction.command_mode=1` で絞れます。欠落はタイムアウト等で対応返信を取得できなかった要求数で、物理パケット損失の確定値ではありません。

```powershell
python -m go2driver replay capture-stock.jsonl
python -m go2driver replay capture-stock.jsonl --profile development
python -m go2driver compare capture-stock.jsonl capture-development.jsonl
```

開発用では `--profile development` を明示して別ログを取得します。compareは各プロファイルのCRCとraw分布・タイミングを並べます。入力JSONL以外の過去のテキストログは、元時刻と生フレームを保持した上で上記イベント形式に変換してください。比較条件（軸、負荷、角度、指令、ゲイン、変換器、ファームウェア）は手動で記録します。分布の一致は物理単位・特性の一致を証明しません。

過去の40°/40°/60°・10秒の試行の追従不足は、今回解決したと扱いません。約16msごとの順次送受信なら各IDは約48ms間隔ですが、内部制御周期とは別です。角度校正→速度フィールド比較→送信値/ゲインの確認→ID別実測周期・順次遅延の評価の順で切り分けます。追従閾値を緩めるだけの対処は実装していません。

## 構成と再現可能な検証

`protocol.py` は純粋なCRC・パッキング・ストリーム解析、`transport.py` はシリアル/仮想通信、`model.py` は単位と検証状態、`driver.py` は要求/返信、`control.py` は範囲・軌道・異常処理です。`calibration.py` と `logging.py` はオフライン処理、`cli.py` が入口です。将来のマイコン移植ではprotocolと測定した換算式から始められます。Cビットフィールドの配置には依存せず明示的にマスク/シフトします。

```powershell
python -m unittest discover -s tests -v
python scripts/offline_validation.py
python scripts/validate_sources.py
```

Linux x86_64とg++があれば `python3 scripts/verify_official.py` で固定した公式共有ライブラリのシリアライザー/デコーダーを再実行し、独立ベクトルと照合できます。SerialPortを生成しません。Windowsの通常テストは保存済みの独立ベクトルを使います。

`validation/report.json`、`unittest.txt`、`official-sdk.txt`、`supplied-captures.jsonl`、`virtual-multi.jsonl` を保存しています。後者は理想追従する**合成の仮想通信**であり、実機の成功ログではありません。オフライン検証の成功を、実機の回転・停止・性能の確認と混同しないでください。

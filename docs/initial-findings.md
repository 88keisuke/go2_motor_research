# ID変更に関する初期調査

公開ソースの静的確認です。実機に接続せず、モーター操作プログラムも実行していません。

## 1. Go2向けの直接候補：open SDK

固定コミット：`76443343d2f4872fb9dbeda0f9c18b386d6b536b`。

入口は [`example/gochangeid.cpp`](https://github.com/openbiped/unitree_actuator_open_sdk/blob/76443343d2f4872fb9dbeda0f9c18b386d6b536b/example/gochangeid.cpp#L20) の `main` です。[CMakeの46–48行](https://github.com/openbiped/unitree_actuator_open_sdk/blob/76443343d2f4872fb9dbeda0f9c18b386d6b536b/CMakeLists.txt#L46) が `GOVERSION=2` を指定して `go2_changeid` を生成します。対応する分岐は `MotorType::GO_2` を選びます。

| 段階 | ソースで確認した処理 | 根拠（gochangeid.cpp） |
| --- | --- | --- |
| ID引数 | `atoi` で旧ID・新IDを読む。範囲チェックなし | 25–27行 |
| 変更フレーム生成 | 4バイト：`FB 01 ((newid & 0xF) << 4) \| (oldid & 0xF) BB` | 28–33行 |
| ブートローダー移行の試行 | `GO_2`、`mode=7`、`id=15`、ゲイン・位置・速度・トルク0で `SerialPort::sendRecv` を呼び、2秒待つ | 39–57行 |
| ID変更送信 | `SerialPort::send` で4バイト送信。200 µs待って4バイト受信を試み、2秒待つ | 59–65行 |
| 復帰の試行 | ID 0～14それぞれに `FB 04 id BB`、200 µs後に `FB 03 id BB`、500 µs待機 | 67–85行 |
| 結果表示 | `SUCCESS` を表示するが、新IDでの問い合わせは行わない | 87–88行 |

旧ID0→新ID1なら変更フレームの構成は `FB 01 10 BB` になります。これはコードの読み取り例であり、実機への送信手順ではありません。

### 通信・パケット生成

- 通信方式はRS-485シリアル。[`SerialPort.h` 48–56行](https://github.com/openbiped/unitree_actuator_open_sdk/blob/76443343d2f4872fb9dbeda0f9c18b386d6b536b/include/serialPort/SerialPort.h#L48) の既定値は4,000,000 baud、8N1、フロー制御なし、受信タイムアウト20,000 µsです。`gochangeid.cpp` はポート名だけ渡すのでこの既定値を使います。
- [`SerialPort::sendRecv` 319行以降](https://github.com/openbiped/unitree_actuator_open_sdk/blob/76443343d2f4872fb9dbeda0f9c18b386d6b536b/src/SerialPort.cpp#L319) は `MotorCmd::modify_data` でフレームを生成し、Go2では17バイトを送信、16バイトの応答を期待します。
- [`MotorCmd::modify_data` のGO_2分岐](https://github.com/openbiped/unitree_actuator_open_sdk/blob/76443343d2f4872fb9dbeda0f9c18b386d6b536b/src/unitreeMotor.cpp#L78) は `FE EE` ヘッダー、4ビットID、3ビットmode、12バイトの制御値、CRC16の17バイトを構築します。CRCは反転多項式 `0x8408`、初期値 `0x2CBB`、末尾CRCを除いた15バイトに適用されます。GO-M8010-6分岐の初期値0とは異なります。
- [`RIS_Mode_t`](https://github.com/openbiped/unitree_actuator_open_sdk/blob/76443343d2f4872fb9dbeda0f9c18b386d6b536b/include/unitreeMotor/include/motor_msg_GO-M8010-6.h#L16) のコメントではID 0～14、15は応答なしのブロードキャストです。modeのコメントは0～2と3の予約を説明するのみで、7の意味は正式に説明していません。mode7をブートローダー移行と呼ぶ根拠はこのツールのコメントと使い方です。
- ID変更・復帰の4バイトコマンドには、このコードではCRCを付けていません。受信フレームの仕様は不明です。

### 重要な未確認事項

`sendRecv` の戻り値とID変更後の `recv` の結果・内容を無視しています。送信したバイト数の確認だけでは、モーターがコマンドを受理したことは証明できません。ブロードキャストへの無応答と失敗を区別していません。

新旧IDはマスクされるため、範囲外の入力も別の4ビット値として送られます。新ID15も入力上は拒否しません。全モーター向けの移行とID 0～14への復帰試行はバス上の他のモーターに影響する可能性があります。

純正Go2の適合ファームウェア版、ACK、設定保存領域・保存時点、電源再投入後のID維持は、この実装だけでは分かりません。上流READMEはGo2対応を表明していますが、本研究では実機確認していません。

## 2. 公式SDK：別方式とバイナリの調査対象

固定コミット：`5b79a42d81cd69adac1367db79efdb972a898fd1`。

[`example/changeID.cpp`](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/example/changeID.cpp#L17) は `MotorType::B1` を設定し、コメントでA1/B1を対象としています。ID `0xBB` のブロードキャストでmode10を送り、100 ms後にmode11（`MotorPulsator`）へ移行、手動でモーターを回してID 0/1/2を選び、mode0で保存する方式です。Go2用4ビットID／mode7／4バイトコマンドとは別に扱います。

[`motor_tools/README.md`](https://github.com/unitreerobotics/unitree_actuator_sdk/blob/5b79a42d81cd69adac1367db79efdb972a898fd1/motor_tools/README.md#L1) はRS-485経由の内部設定ツール `changeid` を説明しています。通常モードでは設定を変更できないという記述があり、ブートローダーモードとの関係を調査する根拠になります。ただし対応機種と具体的な変更フレームは記述されていません。

`motor_tools/Unitree_MotorTools_v1.2.4_{x86_64,arm64,arm32}_Linux/` には実行ファイルがあります。x86_64版 `changeid` はstripped ELFで、ツールのソースはこのディレクトリにはありません。今回はファイル形式のみ確認し、実行・逆アセンブルはしていません。次段階の静的比較対象です。

## 3. Go2解析資料：ブートローダー背景

固定コミット：`4e446688f96e96fa124b337d30c7fd18d2ce1c2a`。

[`docs/bootloader.md` 38–56行](https://github.com/thomasfla/go2_motor_analysis/blob/4e446688f96e96fa124b337d30c7fd18d2ce1c2a/docs/bootloader.md#L38) は純正アプリケーションからの未文書化フレームによる移行と、リセット直後の `0x18` による別経路を記述しています。前者のフレーム形式と応答は未記録と明記されています。

`tools/enter_bootloader_0x18.py` は実機へ送信するツールなので実行しません。カスタムファームウェアの通信・設定を純正ファームウェアのID変更実装と同一視しません。資料は機種・ブート経路の背景情報として利用します。

## 現時点の結論

Go2向けID変更を意図した公開コードの候補は特定できました。純正Go2で受理される条件と永続化は未確認です。次は公式 `changeid` の通信構築部分を静的に比較し、ACK・保存・復帰条件について公開資料と照合します。ファームウェア更新はID変更の前提とせず、調査範囲から除外します。

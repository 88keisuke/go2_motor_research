# 保存したオフライン検証

2026-11-03（Asia/Shanghai）、Linux x86_64 / Python 3.12で実行。実機への送信は0回です。

- `unittest.txt`：23テスト、全件成功。提示10フレームの1320箇所の1ビット反転、部分受信/連結/ノイズ/ID不一致/タイムアウト、異常中断と終了返信欠落を含みます。
- `report.json`：CRCのパラメーター、TX/RXごとの計算結果・解析値、テスト結果、仮想複数台運転の記録。
- `official-sdk.txt`：固定した公式Linux64ライブラリを直接呼び出した4 TX + 1 RX独立ベクトルとの照合結果。SerialPortを生成しません。
- `supplied-captures.jsonl`：ユーザー提示の過去5 TX + 5 RX。時刻不明のため時刻を付加していません。
- `official-development.jsonl`：公式SDKの合成入力/出力ベクトル。実機の送受信ログや時系列の要求応答ではありません。
- `virtual-multi.jsonl`：理想追従する合成プラントとの通信。ノイズと分割受信を含みます。時刻はこの仮想通信を実行したホストの測定値で、実機の遅延・制御周期を示しません。

追加の確認：wheelビルド、隔離した `/tmp` 以下へのインストール、`go2motor --help` 起動、CLIの解析/校正/速度比較/dry-run/再解析、3サブモジュールの固定コミット・ライセンス・未変更状態の検証が成功しています。

再実行は `python scripts/offline_validation.py`（保存ログを再生成）、`python scripts/verify_official.py`（Linux x86_64 + g++ + 公式サブモジュール）、`python scripts/validate_sources.py`。JSONLは上記の合成/過去資料のみで、実機の成功証拠ではありません。

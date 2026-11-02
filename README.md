# Go2 Motor Research / Python Driver

Unitree Go2に標準搭載される純正モーターについて、**元のファームウェアを維持したままIDを変更する処理と条件を特定する**研究リポジトリです。

現在は資料取得・静的解析・Pythonドライバー実装・オフライン検証の段階です。今回、実機への送信は行っていません。ID書き換え、ブートローダー移行、ファームウェア書き込みは新しいドライバーの対象外です。以下の検証手順はシリアルポートを開きません。

## Go2 Air純正モーター用Pythonドライバー

[導入・CLI・校正手順（Windows中心）](docs/python-driver.md) / [公式根拠・変換式・未検証事項](docs/protocol-evidence.md)。状態取得、単体/複数台の位置軌道、速度・トルク指令、raw/換算値、JSONL保存・再解析、校正を実装しました。既定はdry-runで、明示的な `--send` がなければデバイスを開きません。

```sh
python -m unittest discover -s tests -v
python -m go2driver status --ids 0 1 2 --dry-run
python -m go2driver replay validation/supplied-captures.jsonl
```

純正側のCRCは提示TX/RX各5フレームに一致しています。物理単位・減速比6.33・ゲイン・モード0の機械的挙動は実機未検証です。実機制御は使用環境で終了指令を確認した根拠の設定が必要です。検証記録は `validation/`、提示フレームと公式SDKの独立ベクトルは `tests/fixtures/` にあります。

## 最初に読む資料

- [解析計画](docs/research-plan.md)：優先順位、調査項目、成果物、完了条件。
- [初期調査結果](docs/initial-findings.md)：ID変更候補、通信フレーム、確認済み事項と未確認事項。
- [参照元・ライセンス一覧](docs/sources.md)：取得元、固定コミット、原本の管理方法。
- [機械可読の取得記録](sources.json)：コミットとライセンスファイルのSHA-256。

## 原本の管理

指定された3リポジトリは `references/` 以下のGitサブモジュールとして、取得時のコミットに固定しています。原本のソース、ライセンス、著作権表示、Git履歴を保持し、解析メモはこのリポジトリの `docs/` に分離します。サブモジュール内のファイルは変更しません。

```sh
git clone --recurse-submodules https://github.com/88keisuke/go2_motor_research.git
cd go2_motor_research
python3 scripts/validate_sources.py
```

既存チェックアウトで未取得の参照がある場合は `git submodule update --init --recursive` を使います。固定版を維持するため、通常のセットアップで `--remote` を使いません。

## クラウド環境のオフライン検証

作業ディレクトリは `/workspace/go2_motor_research` です。既存の隔離環境・チェックアウトを使い、追加のGit worktreeは作りません。

```sh
bash scripts/setup_analysis.sh
```

Git、Python 3、g++（C++17）、makeを使用します。CMake 3.31.6を `/workspace/.go2-research-tools` の仮想環境に導入し、open SDKのコアだけを `build/open-sdk/` にビルドします。モーター操作例とPythonバインディングは無効です。続いて、シリアル通信を含まない送信フレーム生成チェックを実行します。常駐サービスや認証情報は不要です。

このチェックの成功はソース取得とフレーム生成の再現性を示します。純正モーターがID変更コマンドを受理すること、IDが不揮発領域に保存されること、再起動後も維持されることは未検証です。

## 初期調査で見つかった重要な候補

`references/unitree_actuator_open_sdk/example/gochangeid.cpp` に、`GOVERSION=2` でGo2を選ぶ `go2_changeid` の実装があります。コードはモード7への移行後に `FB 01 <新IDと旧IDを詰めた1バイト> BB` を送ります。ただし受信内容と変更結果を確認せず `SUCCESS` を表示します。現時点では**解析候補**であり、実機で確認済みの変更方法とは扱いません。

公式SDKの `example/changeID.cpp` はA1/B1向けの回転操作によるID設定で、Go2向け候補とは区別して調査します。詳細と固定コミットへの根拠リンクは初期調査結果に記録しています。

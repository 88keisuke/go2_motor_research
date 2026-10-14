# 参照元とライセンス

以下の3件をGitサブモジュールとして取得しました。URLは追跡パラメーターを除いた正規のGitHub URLです。取得時点のコミットを親リポジトリのgitlinkと `sources.json` に記録し、ブランチの最新状態には自動追従しません。

| 参照元 | 配置 | 固定コミット | 原本LICENSE |
| --- | --- | --- | --- |
| [openbiped/unitree_actuator_open_sdk](https://github.com/openbiped/unitree_actuator_open_sdk) | `references/unitree_actuator_open_sdk` | `76443343d2f4872fb9dbeda0f9c18b386d6b536b` | [BSD-3-Clause](../references/unitree_actuator_open_sdk/LICENSE) |
| [thomasfla/go2_motor_analysis](https://github.com/thomasfla/go2_motor_analysis) | `references/go2_motor_analysis` | `4e446688f96e96fa124b337d30c7fd18d2ce1c2a` | [MIT](../references/go2_motor_analysis/LICENSE) |
| [unitreerobotics/unitree_actuator_sdk](https://github.com/unitreerobotics/unitree_actuator_sdk) | `references/unitree_actuator_sdk` | `5b79a42d81cd69adac1367db79efdb972a898fd1` | [BSD-3-Clause](../references/unitree_actuator_sdk/LICENSE) |

ソース、LICENSE、著作権表示、同梱の第三者資料とそのライセンスを原本のまま保持します。各リポジトリのルートLICENSEだけで、同梱の全第三者コードの条件を上書きするものではありません。引用・再配布時は該当する元ファイルの条件も確認します。本研究リポジトリ全体のライセンスは今回新設していません。

更新する場合は対象のコミット、変更理由、LICENSEの差分、解析への影響を確認し、gitlink・`sources.json`・根拠リンクを同時に更新します。原本への変更を解析メモとして保存せず、`docs/` に記録します。

## 取得と検証

```sh
git submodule update --init --recursive
python3 scripts/validate_sources.py
```

検証は固定コミット、origin URL、gitlink、ライセンスのSHA-256、未変更の作業ツリーを確認します。実機通信は行いません。

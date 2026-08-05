# ControlLearning

RDS（RoboDragonsSystem）の制御学習，シミュレーション演習，および講義解説Webアプリケーションです．

## ディレクトリ構成とコンテンツ

- `slide/index.html`: RDS 車輪速度制御モデル（`wheel.m`）の解説スライドWebアプリケーションです．全デバイス（スマホ・タブレット・PC）に完全対応したレスポンシブデザイン，SVG 制御ブロックダイアグラム，MathJax 3 による LaTeX 数式表示，モータ状態空間モデル $\dot{x} = Ax + Bu$，$y = Cx + d$，PID/FF制御，60Hz LPF 速度推定の解説と，ブラウザ上で動くリアルタイム車輪速度制御シミュレータが含まれています．
- `question/wheel.m`: RDS 車輪制御モデルの課題用穴あきスクリプト（時間軸 $t \in [0, 1.0]$ s 厳密固定，`SHOW_ANIMATION` 独立ウィンドウ車輪回転描画機能付き）です．
- `answer/wheel.m`: 模範解答スクリプト（時間軸 $t \in [0, 1.0]$ s 厳密固定，`SHOW_ANIMATION` 独立ウィンドウ車輪回転アニメーション描画機能付き）です．

---

## 🌐 Webスライドへのアクセス方法

1. **GitHub Pages (24時間常時Web公開)**:
   GitHub Pages を有効化することで，URL [資料](`https://robodragons.github.io/ControlLearning/slide/index.html`)経由でスマホ等どのデバイスからでも常時アクセス可能になります．
2. **ローカルファイルで直接開く**:
   - [slide/index.html を開く](file:///c:/Users/shuu0/RoboDragons/ControlLearning/slide/index.html)
3. **ローカルWebサーバー起動 (同一Wi-Fiのスマホ等からアクセス)**:
   ```bash
   python -m http.server 8080 --directory c:\Users\shuu0\RoboDragons\ControlLearning\slide
   ```

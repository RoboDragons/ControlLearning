# ControlLearning

RDS（RoboDragonsSystem）の制御学習，シミュレーション演習，および講義解説Webアプリケーションです．

## ディレクトリ構成とコンテンツ

- `slide/index.html`: RDS 車輪速度制御モデル（`wheel.m`）の解説スライドWebアプリケーションです．全デバイス（スマホ・タブレット・PC）に完全対応したレスポンシブデザイン，SVG 制御ブロックダイアグラム，MathJax 3 による LaTeX 数式表示，モータ状態空間モデル $\dot{x} = Ax + Bu$，$y = Cx + d$，PID/FF制御，60Hz LPF 速度推定の解説と，ブラウザ上で動くリアルタイム車輪速度制御シミュレータが含まれています．
- `question/`: 演習用穴あきスクリプトディレクトリ（ステップ別に単体実行・確認可能）
  - `task1_plant_model.m`: 【課題1】モータ連続・離散時間状態空間モデルの導出と開ループステップ応答確認
  - `task2_pid_control.m`: 【課題2】偏差計算，PID制御器の実装，PWM飽和リミット処理
  - `task3_output_equation.m`: 【課題3】出力方程式 $y = Cx + d$ とエンコーダパルス量子化ノイズの検証
  - `task4_velocity_estimation.m`: 【課題4】後退差分と60Hz 2次バターワースLPFによる速度推定
  - `wheel.m`: 【総合演習】全要素を統合した車輪速度制御シミュレーション（独立ウィンドウ回転アニメーション付き）
- `answer/`: 各課題および総合シミュレーションの模範解答スクリプトディレクトリ
  - `task1_plant_model.m`: 課題1 模範解答
  - `task2_pid_control.m`: 課題2 模範解答
  - `task3_output_equation.m`: 課題3 模範解答
  - `task4_velocity_estimation.m`: 課題4 模範解答
  - `wheel.m`: 総合シミュレーション 模範解答

---

## 🌐 Webスライドへのアクセス方法

1. **GitHub Pages (24時間常時Web公開)**:
   GitHub Pages を有効化することで，[資料](https://robodragons.github.io/ControlLearning/slide/index.html)経由でスマホ等どのデバイスからでも常時アクセス可能になります．
2. **ローカルファイルで直接開く**:
   - [slide/index.html を開く](file:///c:/Users/shuu0/RoboDragons/ControlLearning/slide/index.html)
3. **ローカルWebサーバー起動 (同一Wi-Fiのスマホ等からアクセス)**:
   ```bash
   python -m http.server 8080 --directory c:\Users\shuu0\RoboDragons\ControlLearning\slide
   ```

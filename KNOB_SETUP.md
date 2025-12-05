# ノブスプライト画像の埋め込み手順

## 必要なもの
- knob.png (64x64ピクセル × 64フレーム、縦並び)

## 手順

### 1. 画像を配置
`knob.png` を `Resources/` フォルダに配置してください：
```
plugin-cloud-grain/
  └── Resources/
      └── knob.png  ← ここに配置
```

### 2. BinaryData ファイルを生成
以下のコマンドを実行してください：

```bash
python3 generate_binary_data.py
```

このスクリプトが以下を自動生成します：
- `BinaryData.h`
- `BinaryData.cpp`

### 3. プロジェクトをビルド
通常通りプロジェクトをビルドしてください。画像データはバイナリに埋め込まれます。

## 注意事項
- 画像はビルド時にバイナリに埋め込まれるため、実行時にファイルが不要になります
- 画像を変更した場合は、再度 `generate_binary_data.py` を実行してください
- `BinaryData.h` と `BinaryData.cpp` は自動生成されるため、手動で編集しないでください

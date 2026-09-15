---
name: development-guidelines
description: チーム全体で統一された開発プロセスとコーディング規約を確立するための包括的なガイドとテンプレート。開発ガイドライン作成時、コード実装時に使用する。
allowed-tools: Read, Write, Edit
---

# 開発ガイドラインスキル

チーム開発に必要な2つの要素をカバーします:
1. 実装時のコーディング規約 (implementation-guide.md)
2. 開発プロセスの標準化 (process-guide.md)

## 前提条件

開発ガイドライン作成を開始する前に、以下を確認してください:

### 推奨ドキュメント

1. `docs/architecture.md` (アーキテクチャ設計書) - 技術スタックの確認
2. `docs/repository-structure.md` (リポジトリ構造) - ディレクトリ構造の確認

開発ガイドラインは、プロジェクトの技術スタックとディレクトリ構造に
基づいた具体的なコーディング規約と開発プロセスを定義します。

## 既存ドキュメントの優先順位

**重要**: `docs/development-guidelines.md` に既存の開発ガイドラインがある場合、
以下の優先順位に従ってください:

1. **既存の開発ガイドライン (`docs/development-guidelines.md`)** - 最優先
   - プロジェクト固有の規約とプロセスが記載されている
   - このスキルのガイドより優先する

2. **このスキルのガイド** - 参考資料
   - ./guides/implementation.md: 汎用的なコーディング規約
   - ./guides/process.md: 汎用的な開発プロセス
   - 既存ガイドラインがない場合、または補足として使用

**新規作成時**: このスキルのガイドとテンプレートを参照
**更新時**: 既存ガイドラインの構造と内容を維持しながら更新

## 出力先

作成した開発ガイドラインは以下に保存してください:

```
docs/development-guidelines.md
```

## ⚠️ 本プロジェクトでの注意

**このスキルの `./guides/` は汎用テンプレート由来で、コード例はTypeScript/JavaScript、ブランチ戦略はGit Flow前提で書かれています。**

本プロジェクトは **Python 3.13 + FastAPI + PostgreSQL 15（Raspberry Pi）** です。
**コード実装時の規約は `docs/development-guidelines.md`（Python規約・pytest/ruff/mypy・main+featureブランチ）を正としてください。**
このスキルのガイドは、`docs/development-guidelines.md` を新規作成・改訂する際の「構成の参考」としてのみ使用します。

## クイックリファレンス

### コード実装時
**必ず `docs/development-guidelines.md` を参照**（./guides/implementation.md はTS例の汎用参考資料であり、実装時には使わない）

### 開発プロセスの参照／策定時
ガイドライン文書の構成参考: ./guides/process.md

含まれる内容（汎用例）:
- 基本原則（具体例の重要性、理由説明）
- Git運用ルール（汎用例はGit Flow。本プロジェクトは main + feature/fix ブランチ）
- コミットメッセージとPRプロセス
- テスト戦略（ピラミッドとカバレッジ）
- コードレビューのプロセス
- 品質自動化

### テンプレート
開発ガイドライン作成時: ./template.md


## 使用シーン別ガイド

### 新規開発時
1. `docs/development-guidelines.md` で命名規則・コーディング規約・ブランチ戦略を確認
2. テストを先に書く（TDD）

### コードレビュー時
- `docs/development-guidelines.md` のレビュー観点・規約に照らして確認

### テスト設計時
- `docs/development-guidelines.md` の「テスト戦略」（ピラミッド、カバレッジ、pytestパターン）

### リリース準備時
- `docs/development-guidelines.md` の「Git運用ルール」
- コミットメッセージが Conventional Commits に従っているか確認

## チェックリスト

- [ ] コーディング規約が具体例付きで定義されている
- [ ] 命名規則が明確である（言語別・プロジェクト固有）
- [ ] エラーハンドリングの方針が定義されている
- [ ] ブランチ戦略が決まっている（本プロジェクト: main + feature/fix ブランチ）
- [ ] コミットメッセージ規約が明確である（Conventional Commits）
- [ ] PRテンプレートが用意されている
- [ ] テストの種類とカバレッジ目標が設定されている
- [ ] コードレビュープロセスが定義されている
- [ ] 品質チェック手順が定義されている（pytest / ruff / mypy をコミット前に実行）
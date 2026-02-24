# GitHub Copilot 接入第三方 API（MiniMax）

本项目已预置 `OAICopilot` 工作区配置，你只需要补充 API Key。

## 已完成配置

- 工作区设置：`.vscode/settings.json`
  - `oaicopilot.baseUrl = https://api.minimaxi.com/v1`
  - 预置模型：`MiniMax-M2.5`
- 扩展推荐：`.vscode/extensions.json`
  - `github.copilot`
  - `github.copilot-chat`
  - `johnny-zhao.oai-compatible-copilot`

## 你需要做的事（等你拿到 API Key 后）

1. 安装并启用上述扩展。
2. 打开命令面板（`Ctrl+Shift+P`）。
3. 执行：`OAICopilot: Set OAI Compatible API Key`。
4. Provider 选择 `minimax`（如果提示）。
5. 粘贴你的 API Key。
6. 打开 Copilot Chat，模型选择器 -> `Manage Models...` -> `OAI Compatible`，勾选 `MiniMax-M2.5`。

## 可选（多模型/多供应商）

如果你后续需要同时接入多个 API，可执行：

- `OAICopilot: Set OAI Compatible Multi-Provider API Key`

并在 `oaicopilot.models` 中继续添加模型条目。

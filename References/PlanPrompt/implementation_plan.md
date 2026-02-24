# Claude Code 会话跟踪与分支管理架构（最终演进版）

> 本方案深度契合 Claude Code 的“代理循环”工作流及上下文管理机制，遵循“以更少的外设实现效果”原则，将 Git 分支视作会话的载体，而非对话的镜像。

---

## 一、 系统架构理念（基于官方文档）

1. **会话本位论：** Claude Code 的上下文边界是会话（Session）。会话状态存放在磁盘，与 Git 本质解耦。
2. **代价敏感性：** 任何引入 LLM 的工具调用都会占用 `200k Token` 的珍贵空间。
3. **分层职责模型：**
   * **Hooks（后台钩子）：** 负责静默的底层数据安全性打底（如自动 `checkpoint`，拦截危险写入）。
   * **Skills（用户技能）：** 负责传达明确的宏观意图（如合并、切换主干），AI 作为中介解析自然语言。
   * **Scripts（直通脚本）：** 针对纯读取与状态展示（如查看挂载关系），旁路 AI，通过原生命令行直接反馈，极速且 0 Token 消耗。

---

## 二、 核心业务逻辑更新

### 1. 分支模型的演进及层级

摒弃“对话级分支”的碎片化设想。确立**“平等的平行宇宙”**模型：

* **受保护主干（Protected Mains）**
  * 地位对等，不可被 AI 直接修改。
  * 例如：`main`, `v1-stable`, `legacy-support`。
  * 配置于轻量级文件 `~/.claude/session-git-config.json` 中。
* **工作流分支（Working Branches）**
  * AI 发挥的舞台。
  * **挂载关系：** 一个工作分支上承载着多个物理会话窗口的所有历史更改。

### 2. 用户批注的响应与落地

* **批注响应 A：多路径探索的任意性 (`/fork-explore` 重构)**
  * 不再局限于严格的“两条路径”，而是支持依据实际情况分化出任意数量的子分支。表现形式为脱离当前执行绪，创建平行的逻辑分支。
* **批注响应 B： `/goto` 强化与引导拦截 (`askquestion` 返回机制)**
  * 支持 `commit_id` 与已知分支名（如 `main`）的全跳跃能力。
  * 若目标处于脱离头指针（Detached HEAD）的危险地带，通过脚本主动向 stdout 输出标准的自然语言提示词，触发 Claude Code 内置的回问机制，引导用户做出抉择。
* **批注响应 C： 保护机制强化 (`/delete` 防护)**
  * 销毁当前开发分支前，必须弹出确权（需用户显式输入 Y/n 或在提示下输入分支全名比对）。
* **批注响应 D：并联基座建立 (`/new-main` 与 `/branch-main-from`)**
  * 更名明确化：从既有主干或某个特定的旧有提交点，抽离并标记出一个与 `main` 平起平坐的新受保护主干分支。
* **批注响应 E：0 Token 极速状态感知 (`/branch-status`)**
  * 化繁为简。废弃依赖 AI 整理状态的重资产方案，转为纯原生 bash 脚本（类似 `git log --graph` 及挂载数据联表查询），直接输出在终端视野，不再进入代理循环的消耗链路。
* **批注响应 F：同步与实效性 (实名动态挂载表)**
  * 由 `SessionStart` 和 `UserPromptSubmit` 双峰挂钩。只要由于用户编辑在 `.claude` 内产生了重命名指令流，Hook 将即刻截获并覆盖 `session-git-map.json` 中的称谓缓存。

---

## 三、 实施方案详解：结构与指引

### 1. 数据中枢

* **`~/.claude/session-git-config.json`**：存受保护的分支正则列表。
* **`~/.claude/session-git-map.json`**：维护“会话到分支”的多对一热链关系。

```json
{
  "<session_hash>": {
    "branch": "feat/payment-gateway",
    "project": "/home/dev/foo-project",
    "title": "支付网关开发(第一阶段)",
    "last_active": "2026-02-24T14:23:01Z"
  }
}
```

### 2. 后台基建：Hooks 网络

严格遵循官方 Hook 生命周期的分段介入。

| Hook 类型 | 接管行为与设计思想 |
| :--- | :--- |
| **`SessionStart`** | **(上下文精简注入 + 标题刷新)** matcher: `*` 或 `startup|resume`。只抓取当前被置入的分支系。如果陷入了 Detached HEAD，注入一句`[WARNING: HEAD Detached]`。这是维持 Token 经济学最重要的防线。 |
| **`UserPromptSubmit`** | **(快照基准线)** matcher: `*`。拦截用户的每次输入。当检测到工作区存在手工编写的“脏数据”时，触发 `checkpoint: user-manual <timestamp>`。 |
| **`PreToolUse`** | **(红线防越)** matcher: `Write|Edit|MultiEdit`。核验当前执行上下文的分支名是否处于受保护名词阵列之下。是则`exit 2` 阻断 AI 改写现实；非则针对潜在变更做先期 `git add` 暂存准备。 |
| **`PostToolUse`** | **(增量存盘)** matcher: `Write|Edit|MultiEdit`。打下`checkpoint: <tool_name> <path>` 记录工具修改快照。 |
| **`Stop`** | **(收尾清算)** matcher: `*`。清理临时操作状态，处理纯问答空转分支的快照。 |

### 3. 用户武器库：Skills 与 指令群

官方机制规定，每个斜杠命令应当拥有自己的物理目录和独立的 `SKILL.md`，并在文件中向 Claude 提供指导。我们将它们部署在 `~/.claude/skills/` 之中：

* `.claude/skills/merge/SKILL.md` (提供 `/merge`)
* `.claude/skills/delete/SKILL.md` (提供 `/delete <branch_name>`)
* `.claude/skills/new-branch/SKILL.md` (提供 `/new-branch <name>`)
* `.claude/skills/new-main/SKILL.md` (提供 `/new-main <name>`)
* `.claude/skills/branch-main-from/SKILL.md` (提供 `/branch-main-from <commit-id> <name>`)
* `.claude/skills/fork-explore/SKILL.md` (提供 `/fork-explore [<path-1> <path-2> ...]`)
* `.claude/skills/goto/SKILL.md` (提供 `/goto <target>`)
* `.claude/skills/branch-status/SKILL.md` (提供 `/branch-status`，使用 `disable-model-invocation: true` 降低模型调用的强制消耗，但在说明里引导执行脚本)

这里每个 `SKILL.md` 将通过提示词，指示 Claude 去调用封装好的 bash 脚本，或者直接执行必要操作。

---

## 结论

这份蓝图不再是将 Git 当作玩具去适配闲聊框，而是深刻利用大模型的理解能力，以 Git 的严谨逻辑为骨架构建出的专业级工作流代理系统。

*等待确认：如果没有重大分歧，我将正式按此结构开始生成具体的 Hook 脚本和外挂技能清单代码。*

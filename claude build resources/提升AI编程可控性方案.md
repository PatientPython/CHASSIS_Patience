# 分支管理

原则：
所有的AI编辑都需要挂载在working branch上面，在与AI开启对话后就需要保证自己在working branch上。会使用hooks自动进行git commit来做checkpoint

[这个可以用户输入指令hooks自动按需注入提示词实现。使用脚本如果当前处于默认主分支输出后面的指令，如果不处于默认主分支只注入简短的当前分支信息，不注入额外信息]

如果当前 Git 指针位于默认主分支上，且我启动了一次会话，你必须使用内置的 AskUserQuestion 依次向我提出两个问题：

“基于当前git指针位置，创建新git worktree还是work branch？”；

在我做出选择后，选项A显示上一个问题的回答，选项B”提供类似 “Please type the name of the new branch or worktree below” 的选项用于输入分支名。对于work branch在命名时在前面自动加上“work/”前缀。

Example 1：
AI:
"当前你处于默认主分支，基于当前git指针位置，创建新git worktree还是work branch？
选项A: NEW git worktree 用于归档版本或并行开发等
选项B: NEW git work branch 用于提升AI编程可控性"

User:
"NEW git work branch 用于提升AI编程可控性"

AI:
"选项A: 你将要创建 NEW git work branch
选项B: Please type the name of the new branch or worktree below"

User:
"gimbal-control"

AI:
"创建NEW git work branch，名称为'work/gimbal-control'..."

Example 2:
AI:
"当前你处于受保护分支，基于当前git指针位置，创建新git worktree还是work branch？
选项A: NEW git worktree 用于归档版本或并行开发等
选项B: NEW git work branch 用于提升AI编程可控性"

User:
"NEW git worktree 用于归档版本或并行开发等"

AI:
"选项A: 你将要创建 NEW git worktree
选项B: Please type the name of the new branch or worktree below"

User:
"stable-gimbal"

AI:
"创建NEW git worktree，名称为'stable-gimbal'..."

# 利用git commit 自动 checkpoint

在这些位置自动git commit：

1. 每次用户输入提示词 'UserPromptSubmit:When you submit a prompt, before Claude processes it' 时，自动提交当前未提交的更改，保证没有未提交的更改。
提交签名格式：“CPST:用户提示词的前十个字”（注意不是字符，有汉字一个汉字算一个字）
2. 每次对话停止时提交 'Stop:When Claude finishes responding'
提交签名格式：“CPED:AI回答的前十个字”（注意不是字符，有汉字一个汉字算一个字）
3. 每次任务结束时提交 'TaskCompleted:When a task is being marked as completed'
提交签名格式：“TASK:任务名称的前十个字”（注意不是字符，有汉字一个汉字算一个字）任务名称使用'TaskGet'内置工具自动读取对应名称

原有的.claude\hooks\hook_pre_tool.sh删除

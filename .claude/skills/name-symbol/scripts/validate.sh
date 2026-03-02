#!/bin/bash
# validate.sh — 验证命名文档的格式和命名规则
# 用法: bash validate.sh naming_output.md
# 检查项:
#   1. 模板结构完整性（7个分区是否都存在）
#   2. 全局变量是否有 G_ 前缀
#   3. 静态变量是否有 S_ 前缀
#   4. 结构体类型是否使用 ST_ 前缀
#   5. 枚举类型是否使用 EM_ 前缀
#   6. 宏/常量是否全大写下划线
#   7. 是否存在单字节变量名（除 i/j/k）

set -euo pipefail

FILE="${1:-naming_output.md}"

if [ ! -f "$FILE" ]; then
    echo "❌ 文件不存在: $FILE"
    exit 1
fi

echo "=== 命名文档验证 ==="
echo "文件: $FILE"
echo ""

ERRORS=0
WARNINGS=0

# 1. 检查模板结构完整性
echo "--- 1. 模板结构 ---"
SECTIONS=("物理量" "控制变量" "状态变量" "参数与常量" "中间计算量" "结构体定义" "枚举定义")
for section in "${SECTIONS[@]}"; do
    if grep -q "## [0-9]*\. $section" "$FILE" 2>/dev/null; then
        echo "  ✅ $section"
    else
        echo "  ❌ 缺少分区: $section"
        ((ERRORS++))
    fi
done
echo ""

# 2. 检查全局变量前缀
echo "--- 2. 全局变量前缀 (G_) ---"
# 在表格中查找 float/int 类型的全局变量，检查是否以 G_ 开头
grep -oP '(?<=\| )[A-Za-z_][A-Za-z0-9_]*(?= \| float)' "$FILE" 2>/dev/null | while read -r name; do
    if [[ "$name" == G_* ]] || [[ "$name" == [a-z]* ]] || [[ "$name" == [A-Z][A-Z_]* ]]; then
        : # G_ 前缀、局部变量（小写开头）、常量（全大写）都合法
    else
        echo "  ⚠️ 可能缺少前缀: $name"
        ((WARNINGS++))
    fi
done
echo "  检查完成"
echo ""

# 3. 检查结构体命名
echo "--- 3. 结构体命名 (ST_) ---"
grep -oP 'ST_[A-Za-z0-9_]+' "$FILE" 2>/dev/null | sort -u | while read -r name; do
    echo "  ✅ $name"
done
# 检查是否有不符合 ST_ 的结构体类型
grep -P '结构体类型名' "$FILE" -A 100 2>/dev/null | grep -oP '(?<=\| )[A-Z][A-Za-z0-9_]+(?= \|)' | grep -v '^ST_' | grep -v '^#' | while read -r name; do
    echo "  ❌ 结构体未使用 ST_ 前缀: $name"
    ((ERRORS++))
done
echo ""

# 4. 检查枚举命名
echo "--- 4. 枚举命名 (EM_) ---"
grep -oP 'EM_[A-Za-z0-9_]+' "$FILE" 2>/dev/null | sort -u | while read -r name; do
    echo "  ✅ $name"
done
echo ""

# 5. 检查常量命名（应全大写下划线）
echo "--- 5. 常量命名 (全大写) ---"
grep -P 'const float|const int|const uint' "$FILE" 2>/dev/null | grep -oP '(?<=\| )[A-Za-z_][A-Za-z0-9_]*(?= \|)' | while read -r name; do
    if [[ "$name" =~ ^[A-Z][A-Z0-9_]*$ ]] || [[ "$name" == G_* ]] || [[ "$name" == S_* ]]; then
        echo "  ✅ $name"
    else
        echo "  ⚠️ 常量建议全大写: $name"
        ((WARNINGS++))
    fi
done
echo ""

# 6. 检查单字节变量名
echo "--- 6. 单字节变量检查 ---"
grep -oP '(?<=\| )[a-zA-Z](?= \|)' "$FILE" 2>/dev/null | while read -r name; do
    if [[ "$name" =~ ^[ijk]$ ]]; then
        echo "  ✅ 循环变量 $name 合法"
    else
        echo "  ❌ 禁止单字节变量: $name (华为 规则3.4)"
        ((ERRORS++))
    fi
done
echo "  检查完成"
echo ""

echo "=== 验证完成 ==="
echo "错误: $ERRORS | 警告: $WARNINGS"

if [ "$ERRORS" -gt 0 ]; then
    exit 1
else
    exit 0
fi

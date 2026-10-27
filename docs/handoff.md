# 当前交接记录

更新日期：2026-10-27。本文记录当前实现与证据，长期约定见 `decisions.md`。

## 当前目标与授权

用户明确批准“ADC 平均、换算与浮点低通方案”：新增 5 个文件、修改 9 个文件。
范围为两个独立计算模块、主机数值测试和配套规范文档；温度、DMA 接入、环路调度及 Keil 接入延后。
没有修改生成代码、配置、模板或 Git 状态的授权。

## 当前实现

- `User/Algorithm/filter.c/.h`：带步长整数平均，以 uint32_t 累加并四舍五入；
  头文件 static inline 浮点低通，历史由调用方保存。
- `User/Bsp/adc_convert.c/.h`：五通道枚举 V_CHASSIS、V_BAT、I_BAT、I_CHASSIS、V_CAP；
  每通道独立比例和零点宏；switch 选择参数后调用通用内联计算。
- 数据链为多次采样 → 整数平均 → `adc_raw` → 换算 → `adc_convert` → 可选浮点低通。
  输出数组由调用方持有，转换返回传入地址，无内部静态输出、状态码或错误哨兵。
- 参数按用户提供原理图第 3～5 页及 TI INA240 手册记录标称值，电流零点保留 2047.5 计数；
  未完成实板标定。原理图来源、器件号及参考关系见 `hardware.md`。
- `User/README.md` 提供完整采样块的平均与换算示例及可选低通示例；
  首次低通以当前换算值初始化，未指定实际系数。
- 新模块尚未加入 Keil 构建，也未从 main 或中断调用。当前 main 仍以 4/1 元素循环 DMA
  观察原始计数；不能直接用于 10 次平均示例所需的 40/10 元素完整块。
- 当前 HRTIM 主计数器重复计数为 9，但初始化的 InterruptRequests 为 HRTIM_MASTER_IT_NONE；
  已有中断入口不等于 25 kHz 闭环已运行。实际采集、调度和 40 μs 执行预算待后续设计验证。

## 文件与基线

本轮新增：

- `User/Algorithm/filter.h`、`User/Algorithm/filter.c`
- `User/Bsp/adc_convert.h`、`User/Bsp/adc_convert.c`
- `tools/test_adc_feedback.py`

本轮修改：

- `AGENTS.md`、`README.md`、`User/README.md`
- `docs/coding_style.md`、`docs/decisions.md`、`docs/architecture.md`
- `docs/hardware.md`、`docs/testing.md`、本文

上一阶段开始时已有 .ioc、Core 中断/main/HRTIM 和 Keil uvoptx/uvprojx 改动。
续接时 Keil 两项配置不再显示改动，新增 `MDK-ARM/.vscode/keil-assistant.log` 改动，
Git 索引指纹也与上一阶段记录不同；已向用户报告，保留续接时状态，不恢复或覆盖。
续接时的 118 个范围外受跟踪文件重新记录完整字节指纹，后续检查一致。
该证据仅证明续接阶段范围外文件未变，不将其冒充跨阶段的完整基线证明。
不对被忽略的全部构建产物作未变声明。

续接 Git 索引 SHA-256：
`731f2d50529b6c5ae0026be70eb47628d8bf83b3a205d7906cab1062d53e0d5c`。
续接检查中索引字节一致；未执行暂存、提交、推送或分支操作。
当前 Git 状态包含用户原有改动，不能把全工作区 diff 都视为本轮新增内容。

## 本轮验证

以下命令于 2026-10-27 续接阶段执行，未安装工具。

| 检查命令 | 实际结果与边界 |
| --- | --- |
| `python -X utf8 -B tools/check_project.py` | 退出码 0，21 个受管文件文本检查通过；不覆盖 Core，不等于 C/H 机械格式通过 |
| `python -X utf8 -B -m unittest discover -s tools -p test_check_project.py -v` | 退出码 0，19 项通过，使用系统临时目录 |
| `python -X utf8 -B tools/check_project.py --format` | 退出码 1，NOT_RUN：缺少 clang-format，未执行、未安装 |
| `python -X utf8 -B -m unittest discover -s tools -p test_adc_feedback.py -v` | 退出码 0，11 项全部通过，各在 GCC -O0、-O2 下运行，两个编译单元链接通过 |
| `gcc -std=c99 -Wall -Wextra -Werror -pedantic -fsyntax-only templates/module_template.c` | 退出码 0，模板语法检查通过，不生成目标文件 |
| 范围外及 Git 索引检查 | 续接基线中的 118 个范围外受跟踪文件及索引字节一致 |
| 独立只读审查 | 未发现需修复缺陷；核对实现、测试与文档，不代替实际测试或硬件审查 |
| 固件编译与上板验证 | 未执行；不能据主机测试声明 ARMCC 兼容性、采样时序或闭环性能通过 |

ADC 测试临时 C 文件和可执行文件仅生成于系统临时目录，结束清理；`-B` 禁止字节码缓存。
数值覆盖舍入、最大累加值、较大步长、通道映射、零点符号、返回地址、低通阶跃及完整处理链。
本轮自维护文件采用 UTF-8 无 BOM、LF，不对生成文件做格式化或转码。

## 下一步

1. 用户人工审阅差异并决定是否提交。
2. 经单独清单批准后添加两个 C 文件与头文件路径到 Keil，执行目标固件编译。
3. 设计批量 DMA 缓冲区和处理调度，明确完整块就绪、两路触发相位、块配对、覆盖检测及处理期限；
   不在主计数器事件中未经确认地读取尚未完成的最后一次采样。
4. 上板核对电压、电流方向与标定，测量实际计算耗时，再确定低通系数及环路反馈方案。
5. 温度转换待确认 NTC 型号、R25、B 值/阻温表及实际连接后另行实现。

# 当前交接记录

更新日期：2026-10-27。本文记录当前实现及本轮证据；长期约定见 `decisions.md`。

## 当前目标与授权

用户已批准模块重命名、函数实现迁入 C 文件、Keil 编译链登记及规范同步的逐文件清单。
新约定：编写或重命名自维护 C/H 必须同步登记 Keil；大部分情况下仅做静态检查，无需编写
测试脚本；所有函数实现均放在 .c。新增或执行专项测试须由用户特别说明。

## 当前实现

- 原 ADC 转换文件已重命名为 `User/Bsp/myadc.c/.h`，数组接口为 `Myadc_Convert()`，
  标量接口为 `Myadc_ConvertValue()`。头文件保护宏和全部现行引用已同步。
- `Myadc_ConvertValue()` 与 `Filter_LowPass()` 在头文件声明、在对应 .c 普通定义，
  具有可跨文件调用的外部链接；不在头文件保留实现。私有辅助函数可在 .c 使用 static inline。
  是否实际展开调用由编译器优化决定，不保证跨编译单元内联。
- 整数平均、四舍五入、五通道枚举、参数宏、偏置和转换公式保持原有行为。
  数据链仍为采样块 → 平均 → `adc_raw` → 换算 → `adc_convert` → 可选浮点低通。
  数组及每通道历史由调用方持有，首次低通用当前换算值初始化。
- `MDK-ARM/2026superpower.uvprojx` 新增 Application/User/Bsp 与
  Application/User/Algorithm 分组，登记 myadc、filter 的 C/H。
  C 文件为类型 1，H 文件为类型 5；增加两个头文件目录，未设置排除编译。
- 构建登记已经完成，但没有增加 main 或中断调用，也未执行 Keil 构建。
  现有 DMA 仍为 4/1 元素观察缓冲区，不提供 10 次采样的完整 40/10 元素块。
  DMA 批量采集、环路调度、标定、温度和上板时序继续待接入。
- `tools/test_adc_feedback.py` 是既有专项测试，本轮仅同步文件名、函数名和跨文件调用说明，
  未新增脚本、未运行测试；不将上一版数值测试通过作为当前版已验证证据。

## 本轮文件与保护边界

按清单重命名两个 ADC 文件，修改 filter.c/.h、Keil .uvprojx、既有 ADC 测试，
以及 AGENTS、README、User/README、coding_style、ai_workflow、decisions、architecture、
hardware、testing 和本文。规范及示例统一使用 Myadc 名称、C 文件实现和默认静态检查。
未修改 .ioc、Core、Drivers、启动/RTE、uvoptx、模板或 Git 配置。

实施前在内存记录 132 个已有文件及 Git 索引的 SHA-256；不创建备份文件。
116 个范围外文件中，115 个保持与基线一致；
`MDK-ARM/.vscode/c_cpp_properties.json` 在工作期间出现两条新增头文件路径，
对应 User/Bsp 和 User/Algorithm。agent 未写入此文件，已向用户报告，保留变化；
可能由编辑器随 Keil 工程同步，来源未作进一步确认，不将其当作手工获批修改。

Git 索引实施前指纹：
`ae2a2e7105a267e8e63e19cd8018726429ca1c57e227e1b8a0a02314efe383d8`。
核对时索引字节一致。原有改动保留，未自行暂存、提交、推送或改变分支。

## 本轮验证

| 检查 | 实际结果与边界 |
| --- | --- |
| `python -X utf8 -B tools/check_project.py` | 退出码 0，21 个受管文件文本检查通过 |
| `python -X utf8 -B tools/check_project.py --format` | 退出码 1，NOT_RUN：缺少 clang-format，未安装 |
| `gcc -std=c99 -Wall -Wextra -Werror -pedantic -I User/Bsp -I User/Algorithm -fsyntax-only User/Bsp/myadc.c User/Algorithm/filter.c` | 退出码 0，模块 C99 语法检查通过，不产生目标文件 |
| Keil XML 静态核对 | 两个分组、四个条目的类型、唯一性、路径及包含目录正确，源文件没有局部排除编译 |
| Keil 配置边界核对 | 从当前文本移除两个新增分组和两个包含目录后，SHA-256 与实施前完全相同；其余内容逐字节保留 |
| 现有测试静态核对 | Python AST 解析通过，文件路径和函数引用已同步；未运行数值测试 |
| 旧名及实现位置检索 | 现行源码、示例和规范已替换旧文件名及函数名，公共头文件无函数实现 |
| 独立只读审查 | 未发现需修复的功能、声明链接或 Keil 登记缺陷；不代替目标编译及上板证据 |
| 固件编译、数值测试、检查工具单元测试、上板验证 | 本轮未执行，以静态检查为主；不声称链接、ARMCC 兼容性或实时性能通过 |

自维护改动为 UTF-8 无 BOM、LF；没有整文件格式化生成源码。
Keil 登记不等于固件已编译，也不等于运行时反馈链已经启动。

## 后续入口

1. 用户人工审阅差异并决定是否提交。
2. 需要固件编译时，先确认 Keil 工具和产物位置，再按获批范围构建；避免自动安装或改变编译选项。
3. 单独设计完整采样块就绪、两路 ADC 块配对、覆盖检测与控制调度，再接入运行流程。
4. 实板核对参数、方向及标定，测量计算时间，再选择低通系数并验证环路。
5. 温度换算待明确 NTC 型号和阻温参数后另行实现。

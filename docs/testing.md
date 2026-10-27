# 检查与验证说明

## 1. 自维护规范检查

工具需要 Python 3.10 或更新版本，仅使用标准库，不安装依赖。
从工程根目录执行；也可以从其他目录调用脚本，默认根目录仍是脚本所在工程。
命令不修改工程文件或 Git 索引；`-B` 禁止导入时生成字节码缓存。

```powershell
# 默认检查全部受管文件，不要求文件已被 Git 跟踪。
python -X utf8 -B tools/check_project.py

# 工作区变更、暂存变更、未跟踪且未被 Git 忽略的受管文件的并集。
python -X utf8 -B tools/check_project.py --changed

# 行为测试只在系统临时目录创建样例并在退出时清理，不暂存或提交 Git。
python -X utf8 -B -m unittest discover -s tools -p test_check_project.py -v

# 额外检查 C/H 机械排版，不写回文件。
python -X utf8 -B tools/check_project.py --format
```

`--changed --format` 可组合；`--root PATH` 可显式选择检查根目录，含空格的路径要加引号。
日常交付默认执行全量检查；局部检查通过不代表全量通过。

### 受管范围

- 根目录 `AGENTS.md`、`README.md`、`.editorconfig`、`.clang-format`。
- `docs/`、`User/`、`templates/`、`tools/` 中扩展名为
  `.md`、`.c`、`.h`、`.py`、`.json`、`.yaml`、`.yml`、`.toml`、`.txt` 的文件。
- 排除任意层级的 `Core`、`Drivers`、`MDK-ARM`、`references`、`build`、`Debug`、`Release`、
  `Objects`、`Listings`、`.git`、`.cache`、`__pycache__` 目录，目录排除不区分大小写。
- `.ioc` 不在允许的扩展名内；C/H 前 4096 字节中含 `USER CODE BEGIN` 或
  `STMicroelectronics` 标记时也不纳入；不跟随符号链接或可识别的目录链接。

允许列表和生成标记仅减少误纳入，不能作为生成身份的最终鉴定。新文件仍由人和 agent 确认来源。
尚未添加新类型到受管范围时，不能把该类型当作已经检查；扩展范围也要事先批准。

### 检查内容和输出

- 严格 UTF-8 解码，拒绝 BOM，不自动猜测或转换 GB2312 等旧编码。
- 检查行末空白、最终 LF 和 CR 字符；空的受管文件不视为合格模板。
- 仅对 C/H 检查 Tab；不使用 C 缩进规则误判 Markdown 内容。
- 输出 `相对路径:行:列:错误代码:原因`。UTF-8 解码错误的列按字节计，其他列按字符计。
- `--format` 显式使用项目 `.clang-format`，逐个受管 C/H 调用
  `clang-format --dry-run --Werror`；不使用 `-i`，不执行自动修复。
- 未指定 `--format` 时输出 `text only; C/H format NOT_RUN`，只代表文本检查通过。
- `NO_FILES` 表示没有选中文件，不是整个工程验证通过。

| 退出码 | 含义 |
| --- | --- |
| 0 | 所选文本检查通过；若输出 `NO_FILES`，表示没有检查目标 |
| 1 | 检出文本或格式问题；若同时缺工具，会同时报告未执行项 |
| 2 | Git/路径/工具错误，或格式工具缺失导致请求的检查未执行 |

命名含义、注释真实程度、接口契约、并发、单位及硬件参数需要人工审阅。
脚本不是完整 C 语法分析器，也不替代编译器或上板验证。

## 2. 工具测试覆盖

使用临时文件验证中文 UTF-8、GB2312、BOM、Tab、行末空白、最终换行、CRLF、中文与空格路径、
保护目录和生成标记排除、空选择、检查前后字节不变、退出码、Git 读取错误与格式工具缺失。
Git 清单和不可控的外部格式工具以边界模拟提供输入；测试不在实际仓库暂存或提交。
`--changed` 另外在真实工作区执行，检查与本轮新增文件的对应关系。

## 3. ADC 主机数值测试

```powershell
python -X utf8 -B -m unittest discover -s tools -p test_adc_feedback.py -v
```

需要现有 GCC 和 Python 标准库；缺少 GCC 时测试显示 `NOT_RUN` 并跳过，不能报告数值测试通过，
不自动安装。测试在系统临时目录创建两个 C 编译单元及可执行文件，退出时清理，
不在仓库写入构建产物；使用 `-B` 禁止 Python 字节码缓存。

11 项测试分别在 `-O0`、`-O2` 下链接真实 filter、adc_convert 模块并运行，
编译采用 C99、`-Wall -Wextra -Werror -pedantic`：

- 单次、10 次、不同次数的连续及步长 4 平均，四舍五入、输入不变和通道隔离。
- 65535 个最大 uint16_t 样本的累加与舍入，以及跨越 16 位索引范围的步长。
- 五通道映射、满量程、零点附近、电流正负号、通用比例和偏置、输出地址。
- 低通 alpha 为 0、1 及中间值，上升与下降阶跃，各通道历史独立。
- 完整平均 → 换算 → 初始化历史 → 浮点低通调用链。
- 两个编译单元共同包含头文件，确认 static inline 不造成重复定义。

测试使用独立预期值和浮点容差，不以重复生产公式作为唯一判据。
接口前置条件之外的零次数、零步长、空指针或非法计数不属于受支持输入，不要求错误码。
主机数值验证不包含 DMA 一致性、两路采样对齐、目标 ARM 内联和 40 μs 执行预算。

## 4. 编译证据

当前 Keil 工程记录 ARMCC 5.06 update 6（build 750）。本轮不修改构建配置，不将模板加入固件。
有本机 GCC 时，可进行不产生目标文件的模板 C99 语法检查：

```powershell
gcc -std=c99 -Wall -Wextra -Werror -pedantic -fsyntax-only templates/module_template.c
```

该检查不能证明 ARMCC 兼容性、链接成功、实时性能或固件可用。
固件构建前应单独确认 Keil 工具位置、目标配置及产物目录；运行生成产物的构建需先获批。
模板或文本检查不能代替固件构建。本轮不宣称 Keil 编译或硬件测试通过。

## 5. 硬件验证与记录

具体硬件测试方案需结合已确认的板卡、参数和模块实现，再审阅批准。
记录板卡/固件版本、配置、输入条件、测量方式、预期与实测结果，不把旧工程记录搬作当前证据。
未验证的 ADC 换算、PWM 波形、保护阈值和闭环性能保持“待验证”。

交付时给出实际命令、退出码、日期、测试数量、未执行原因及剩余问题，汇总到获批的
`handoff.md` 更新中。本轮既有文件保护以执行前后的完整内容指纹比较验证，Git 索引单独比较。

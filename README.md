# NGCC ASIC 排行榜 · NGCC ASIC Leaderboard

新一代商用密码算法征集活动（NGCC）候选算法的 ASIC 实现排行榜：在统一工艺与工作频率下比较各实现的运算吞吐量和面积，并公开各实现的 RTL。

网页：<https://chipweaver.github.io/ngcc-asic-leaderboard/>

> chipweaver 独立评估，非 ICCS 官方结果。

[English](#english)

## 收录内容

| 类别 | 目录 | 实现 |
|---|---|---|
| 数字签名 | — | 暂无结果 |
| 密钥封装 | [`kem/`](kem/) | Aigis-Enc+、HARE、NEV、QUBE、ZEN（性能优化版） |
| 密钥交换 | — | 暂无结果 |
| 密码杂凑 | — | 暂无结果 |

## 评估规则

- 指标定义参照商用密码标准研究院《新一代商用密码算法 ASIC 实现自评估指引》（2026 年 6 月）。
- 实现版本：性能优化版。工艺：90 nm。工作频率：160 MHz，各实现在该频率下均通过静态时序分析。
- 运算吞吐量：单位时间内完成的运算次数（ops/s），按平均时钟周期数计算。每个类别如何计为"一次运算"，见对应网页的"评估方法"。
- 面积：综合后逻辑面积与 SRAM 面积之和，以工艺库二输入与非门（NAND2）的面积归一化，单位 kGE。吞面比 = 运算吞吐量 ÷ 面积。
- 按 NGCC 规定的经典安全强度等级（128、256、512 比特，以及可选的 384 比特）分组，仅在同一等级内比较。
- 与指引的差异：工作频率固定为 160 MHz，未进行频率扫描；SRAM 面积由 CACTI 90 nm 模型估算，未使用工艺库存储宏单元。因此数值仅适用于本榜内部比较。

## 目录结构

```
<类别>/                     例如 kem/
  category.json             该类别的核心操作、一次运算的组成、尺寸字段和网页文字
  <实现>/
    result.json             该实现的最新结果（历史版本见 git 记录）
    rtl/
      rtl-files.json        RTL 文件清单（编译顺序）和 include 目录
      src/                  SystemVerilog 源码
site/                       网页模板与字体
build.py                    校验数据并生成网页
```

## RTL 使用说明

- 源码为可综合的 SystemVerilog，文件按 `rtl/rtl-files.json` 中的顺序编译。
- SRAM 模块在仿真时为行为模型；定义 `SYNTHESIS` 后作为黑盒存储宏，综合时需要使用者提供对应工艺的存储宏。

## 本地生成网页

```bash
python3 build.py          # 输出到 dist/，打开 dist/kem/index.html
```

只需 Python 3 标准库。数据有误时构建会失败并列出问题。

## 反馈

发现数据或实现有误，请提交 [Issue](https://github.com/chipweaver/ngcc-asic-leaderboard/issues)。

## 许可

- 代码（RTL、`build.py`、网页模板）：[MIT](LICENSE)。
- 数据与文字（`*/category.json`、`*/*/result.json`）：[CC BY 4.0](LICENSE-DATA)，引用时请注明出处。
- 字体（`site/fonts/`）：SIL Open Font License 1.1，见该目录下的许可证文件。
- 各算法的设计归其提交者所有；本仓库只包含 chipweaver 的硬件实现。

---

## English

ASIC implementation leaderboard for the candidates in the Next-generation Commercial Cryptographic Algorithms Program (NGCC). Implementations are compared on throughput and area under a common process and clock frequency, and their RTL is published here.

Site: <https://chipweaver.github.io/ngcc-asic-leaderboard/>

> Independent evaluation by chipweaver; not an official ICCS result.

**Contents.** Key Encapsulation (`kem/`): Aigis-Enc+, HARE, NEV, QUBE and ZEN, performance-optimized versions. Digital Signature, Key Exchange and Cryptographic Hash have no results yet.

**Rules.** Metric definitions follow the ICCS ASIC self-evaluation guide for next-generation commercial cryptographic algorithms (新一代商用密码算法 ASIC 实现自评估指引, June 2026). Performance-optimized versions, 90 nm process, 160 MHz clock; every implementation passes static timing analysis at that frequency. Throughput is operations per second from mean clock cycles; what counts as one operation is defined per category on its page. Area is post-synthesis logic area plus SRAM area, normalized to NAND2 gate equivalents (kGE); throughput / area = throughput ÷ area. Results are grouped by the NGCC classical security strength levels (128, 256 and 512 bits, plus the optional 384 bits) and compared only within a level. Differences from the guide: the clock is fixed at 160 MHz with no frequency sweep, and SRAM area is a CACTI 90 nm estimate rather than a foundry memory macro, so the numbers are comparable only within this leaderboard.

**Layout.** `<category>/category.json` defines a category; `<category>/<implementation>/result.json` holds the latest result (history is in git); `<category>/<implementation>/rtl/` holds the SystemVerilog sources, compiled in the order of `rtl-files.json`. SRAM modules are behavioural models in simulation and black-box memory macros when `SYNTHESIS` is defined; synthesis needs memory macros for the target process.

**Build.** `python3 build.py` (standard library only) validates the data and writes the site to `dist/`.

**Feedback.** Please open an [issue](https://github.com/chipweaver/ngcc-asic-leaderboard/issues) for errors in the data or the implementations.

**License.** Code (RTL, `build.py`, site template): [MIT](LICENSE). Data and text (`*/category.json`, `*/*/result.json`): [CC BY 4.0](LICENSE-DATA). Fonts (`site/fonts/`): SIL Open Font License 1.1. The algorithm designs belong to their submitters; this repository contains chipweaver's hardware implementations only.

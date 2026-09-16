# SRT 仿真项目（srtfangzhen）

> TODO：一句话说清这个项目仿真的是什么、要回答什么问题

## 项目简介

TODO：背景（SRT 立项背景）、研究目标、仿真对象、预期产出。

## 目录结构

```
srtfangzhen/
├─ docs/       文档：方案、公式推导、参考文献、会议记录
├─ models/     仿真模型：Simulink / COMSOL 等工程文件
├─ scripts/    脚本：参数扫描、数据处理、画图
├─ data/       输入数据：实测数据、材料参数、边界条件
└─ results/    输出结果：图片、导出的数据表
```

## 环境依赖

| 工具 | 版本 | 用途 |
|---|---|---|
| TODO: MATLAB / Simulink | TODO | TODO |
| TODO: COMSOL Multiphysics | TODO | TODO |
| TODO: Python | TODO | TODO |

> 写清楚版本号很重要——半年后换电脑重装，你不会记得当时用的是哪个版本。

## 快速开始

```bash
git clone https://github.com/xxxr2007/srtfangzhen.git
cd srtfangzhen
```

TODO：跑通第一个仿真的步骤（打开哪个模型、运行哪个脚本、预期看到什么）。

## 开发规范

- `main` 保持"能跑"的状态，大改动开 `feature/xxx` 分支
- 提交信息写清楚**改了什么、为什么改**，一行写不下就空一行写正文
- 仿真输出（图片、大数据表）**不要**无脑提交，先看 `.gitignore`
- 模型文件改了记得把**参数设置**也记录在 docs 里，二进制模型没法看 diff

## 开发日志

| 日期 | 内容 |
|---|---|
| 2026-09-16 | 仓库初始化 |

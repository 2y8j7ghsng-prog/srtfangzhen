# git-practice

练手用的仓库，随便折腾，搞坏了删掉重来就行。

## Git 速查表

| 场景 | 命令 |
|---|---|
| 看当前状态（最常用，迷路了先敲它） | `git status` |
| 收集所有改动 | `git add .` |
| 只收集某个文件 | `git add 文件名` |
| 存一个快照 | `git commit -m "改了什么"` |
| 推到 GitHub | `git push` |
| 从 GitHub 拉最新 | `git pull` |
| 看提交历史 | `git log --oneline` |
| 看远程地址对不对 | `git remote -v` |
| 改了但想撤销（危险操作） | `git restore 文件名` |
| 回到某个旧版本 | `git checkout 提交哈希` |

## 一次完整的改动流程

```bash
git status          # 1. 先看看改了啥
git add .           # 2. 收集
git commit -m "..." # 3. 存档
git push            # 4. 上传
```

## 什么时候该 commit

改完一个完整的小功能、或者修好一个 bug 就存一次。
改了三行还没验证过就先别存——历史里全是垃圾提交，以后想回退都找不到。

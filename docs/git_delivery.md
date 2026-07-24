# Git 交付状态

任务文档要求最终以公开 Git 仓库交付源代码、提交历史和复现说明。当前工作区位于 `main` 分支，源码、README、测试、实验记录、报告和演示视频均位于同一工程目录。

公开交付仓库为：<https://github.com/VMnicht/drone_sim>。

复核远端状态可执行：

```bash
git remote -v
git fetch origin main
git log --oneline --decorate -n 10
```

公开前应再次执行 `colcon test-result --all --verbose`、`python3 scripts/verify_experiments.py --quiet`，并确认未提交密钥、账号信息或不应公开的本地数据。

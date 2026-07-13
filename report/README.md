# 报告生成

最终 PDF 位于 `output/pdf/drone_sim_report.pdf`。

在 Codex Windows 运行时中执行：

```powershell
python scripts/generate_report.py --root .
```

报告中的实验表格和曲线来自 `artifacts/experiments/`，因此修改控制参数后应先重新运行三套实验，再重新生成报告。


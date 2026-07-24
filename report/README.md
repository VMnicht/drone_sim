# 报告生成

最终 PDF 位于 `output/pdf/drone_sim_report.pdf`。

在 Codex Windows 运行时中执行：

```powershell
python scripts/generate_report.py --root .
```

报告中的实验表格和曲线来自 `artifacts/experiments/` 的 11 个正式场景，模型、控制、接口、场景和验收数字来自 `src/drone_bringup/config/*.yaml`。修改参数后应先执行 `./start_sim.sh batch`，再重新生成报告。Python 环境需包含 `PyYAML`、`reportlab`、`Pillow` 和 `pypdf`；生成后用 `scripts/render_pdf.py` 渲染 9 页做视觉复核。论文标题与各级标题使用黑色黑体，正文、摘要、图注和页眉页脚使用黑色宋体。

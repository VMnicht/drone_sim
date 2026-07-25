# 报告生成与交付

正式交付文件为：

- `output/docx/drone_sim_academic_report.docx`
- `output/pdf/drone_sim_academic_report.pdf`

报告采用 A4 论文版式。中文标题为黑体黑色加粗，中文正文为宋体黑色，英文与数字使用 Times New Roman。系统架构图和控制框图由程序绘制，实验图来自 `artifacts/experiments/`，没有使用生成式图片。

重新生成 DOCX：

```powershell
python scripts/generate_academic_report_docx.py
```

脚本依赖 `python-docx` 与 `Pillow`。生成后使用 Microsoft Word 或 WPS 打开 DOCX，并导出为 `output/pdf/drone_sim_academic_report.pdf`。可用下列命令把 PDF 渲染为逐页图片，检查标题孤行、图注分页、表格越界和字体替换：

```powershell
python scripts/render_pdf.py output/pdf/drone_sim_academic_report.pdf tmp/report_render --scale 1.8
```

报告中的表格和曲线来自 11 个正式场景的 JSON、CSV 与 PNG。模型、控制、接口和阈值来自 `src/drone_bringup/config/`。若修改了模型参数，应先执行 `./start_sim.sh batch` 重新生成实验材料，再生成报告，避免正文数字与仓库证据不一致。

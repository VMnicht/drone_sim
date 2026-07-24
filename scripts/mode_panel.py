#!/usr/bin/env python3
"""Local Web panel for launching modes and editing allowlisted YAML configs."""

import argparse
import json
import mimetypes
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlparse

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "src" / "drone_bringup" / "config"
DEFAULT_CONFIG = CONFIG_DIR / "mode_panel.yaml"

MODE_CATALOG = {
    "hover": {"name": "单机悬停", "description": "交互式起飞悬停，可打开 RViz2"},
    "experiment": {"name": "正式场景", "description": "运行选定场景并保存独立结果"},
    "multi": {"name": "三机模式", "description": "三机独立命名空间与安全间距监测"},
    "ground-station": {"name": "Web 地面站", "description": "单机仿真与实时控制页面"},
    "batch": {"name": "批量评测", "description": "顺序运行全部场景并执行阈值验收"},
}

SHOWCASE_SECTIONS = [
    {
        "id": "core",
        "name": "核心必做功能",
        "description": "对应任务文档第二、四章：从电机 RPM 到安全路径和 RViz2。",
        "items": [
            {"id": "dynamics", "name": "六自由度动力学", "requirement": "四电机 RPM、一阶响应、推力/力矩、刚体积分、TF/Odom/IMU/Path", "action": {"type": "launch", "mode": "hover", "scenario": "hover", "rviz": True}, "view": "hover"},
            {"id": "controller", "name": "模型控制器与 Mixer", "requirement": "目标点到期望姿态、总推力/三轴力矩和四路 RPM，含多级限幅", "action": {"type": "launch", "mode": "experiment", "scenario": "target", "rviz": True}, "view": "target"},
            {"id": "map", "name": "五障碍物地图", "requirement": "可复现几何地图、固定坐标系、障碍物尺寸和安全膨胀", "action": {"type": "launch", "mode": "experiment", "scenario": "five_obstacles", "rviz": True}, "view": "five_obstacles"},
            {"id": "planning", "name": "3D A* 与安全跟踪", "requirement": "碰撞检测、安全距离、规划路径、单调路径进度和终点锁定", "action": {"type": "launch", "mode": "experiment", "scenario": "perception_replan", "rviz": True}, "view": "perception_replan"},
            {"id": "rviz", "name": "RViz2 综合可视化", "requirement": "机体、目标、历史轨迹、障碍物、点云、体素、规划路径和局部目标", "action": {"type": "launch", "mode": "experiment", "scenario": "perception_replan", "rviz": True}, "view": "perception_replan"},
        ],
    },
    {
        "id": "acceptance",
        "name": "最低验收与稳定性展示",
        "description": "对应任务文档第五章；每张卡可启动真实 ROS2 场景并查看实测曲线。",
        "items": [
            {"id": "demo_hover", "name": "悬停实验", "requirement": "从地面起飞并稳定在 (0,0,1.5)", "action": {"type": "launch", "mode": "experiment", "scenario": "hover", "rviz": True}, "view": "hover"},
            {"id": "demo_target", "name": "目标点实验", "requirement": "飞向 (2,1,1.5) 并悬停", "action": {"type": "launch", "mode": "experiment", "scenario": "target", "rviz": True}, "view": "target"},
            {"id": "demo_square", "name": "多目标点实验", "requirement": "顺序完成正方形 5 个航点", "action": {"type": "launch", "mode": "experiment", "scenario": "square", "rviz": True}, "view": "square"},
            {"id": "demo_five", "name": "静态五障碍避障", "requirement": "障碍位于起终点之间，实际净距大于 0.30 m", "action": {"type": "launch", "mode": "experiment", "scenario": "five_obstacles", "rviz": True}, "view": "five_obstacles"},
            {"id": "demo_narrow", "name": "狭窄通道绕行", "requirement": "展示明显绕行、规划路径与实际轨迹", "action": {"type": "launch", "mode": "experiment", "scenario": "narrow_passage", "rviz": True}, "view": "narrow_passage"},
            {"id": "demo_stability", "name": "稳定性曲线", "requirement": "位置误差、RPM、姿态、轨迹、最小障碍物距离和到达指标", "action": {"type": "view", "scenario": "perception_replan"}, "view": "perception_replan"},
        ],
    },
    {
        "id": "bonus",
        "name": "全部加分项与创新扩展",
        "description": "任务文档第六章 10 个方向均有独立入口。",
        "items": [
            {"id": "yaml", "name": "全量 YAML 参数", "requirement": "质量、惯量、电机系数、控制增益及全部运行参数", "action": {"type": "config", "name": "model.yaml"}},
            {"id": "wind", "name": "风扰恢复", "requirement": "3.04 N 阵风、误差峰值与恢复时间", "action": {"type": "launch", "mode": "experiment", "scenario": "wind_gust", "rviz": True}, "view": "wind_gust"},
            {"id": "noise", "name": "传感器噪声/IMU/GPS", "requirement": "白噪声、偏置、随机游走、协方差、延迟和丢包", "action": {"type": "launch", "mode": "experiment", "scenario": "sensor_noise", "rviz": True}, "view": "sensor_noise"},
            {"id": "pointcloud", "name": "点云、体素与局部感知", "requirement": "量程/FOV/遮挡/噪声/丢点和体素持久化", "action": {"type": "launch", "mode": "experiment", "scenario": "perception_replan", "rviz": True}, "view": "perception_replan"},
            {"id": "multi", "name": "三无人机", "requirement": "独立 namespace/TF/传感器和最小安全间距监测", "action": {"type": "launch", "mode": "multi", "scenario": "hover", "rviz": True}},
            {"id": "circle", "name": "圆轨迹", "requirement": "解析位置、速度和加速度前馈", "action": {"type": "launch", "mode": "experiment", "scenario": "circle", "rviz": True}, "view": "circle"},
            {"id": "figure", "name": "八字轨迹", "requirement": "Gerono 八字连续参考", "action": {"type": "launch", "mode": "experiment", "scenario": "figure_eight", "rviz": True}, "view": "figure_eight"},
            {"id": "evaluation", "name": "自动评测", "requirement": "11 场景批处理、阈值检查、CSV/JSON/PNG 和 rosbag", "action": {"type": "launch", "mode": "batch", "scenario": "hover", "rviz": False}},
            {"id": "station", "name": "Web 地面站", "requirement": "实时状态/曲线、目标输入、reset、扰动与故障控制", "action": {"type": "launch", "mode": "ground-station", "scenario": "hover", "rviz": True}},
            {"id": "comparison", "name": "参考项目对比", "requirement": "pengyu_sim/MARSIM 架构、模型、感知和适用边界", "action": {"type": "file", "name": "comparison"}},
            {"id": "fault", "name": "电机故障注入", "requirement": "效率/上限与命令丢包/延迟/冻结，展示恢复", "action": {"type": "launch", "mode": "experiment", "scenario": "fault_motor", "rviz": True}, "view": "fault_motor"},
            {"id": "sweep", "name": "回放与参数扫描", "requirement": "固定种子回放和 3x3 增益/风力扫描热图", "action": {"type": "file", "name": "sweep"}},
        ],
    },
    {
        "id": "delivery",
        "name": "交付与可追溯材料",
        "description": "任务文档第二、七、八章要求的文档和材料。",
        "items": [
            {"id": "task", "name": "任务文档", "requirement": "原始验收要求", "action": {"type": "file", "name": "task"}},
            {"id": "readme", "name": "完整 README", "requirement": "架构、构建、启动、参数、场景与限制", "action": {"type": "file", "name": "readme"}},
            {"id": "report", "name": "学术论文式 PDF 报告", "requirement": "方法、公式、实验、失败案例、比较、AI 使用和反思", "action": {"type": "file", "name": "report"}},
            {"id": "video", "name": "演示视频", "requirement": "1 到 3 分钟录屏材料", "action": {"type": "file", "name": "video"}},
            {"id": "git", "name": "公开 Git 交付状态", "requirement": "源代码、提交历史、复现说明与远端发布边界", "action": {"type": "file", "name": "git"}},
            {"id": "ai", "name": "AI 使用记录", "requirement": "工具、关键交互、错误、人工修正和验证", "action": {"type": "file", "name": "ai"}},
            {"id": "audit", "name": "完成性审计", "requirement": "逐项证据、指标和复核命令", "action": {"type": "file", "name": "audit"}},
        ],
    },
]

PUBLISHED_FILES = {
    "task": ("任务文档", Path("任务文档.md")),
    "readme": ("README", Path("README.md")),
    "ai": ("AI 使用记录", Path("ai_usage.md")),
    "audit": ("完成性审计", Path("docs/completion_audit.md")),
    "comparison": ("参考项目对比", Path("docs/reference_comparison.md")),
    "report": ("学术报告", Path("output/pdf/drone_sim_report.pdf")),
    "video": ("演示视频", Path("output/video/drone_demo.mp4")),
    "git": ("Git 交付状态", Path("docs/git_delivery.md")),
    "sweep": ("参数扫描热图", Path("artifacts/parameter_sweep/heatmap.png")),
}

RESULT_ARTIFACTS = [
    ("总览", "experiment_summary.png"),
    ("三维轨迹", "trajectory_3d.png"),
    ("位置跟踪", "position_tracking.png"),
    ("位置误差", "position_error.png"),
    ("电机 RPM", "motor_rpm.png"),
    ("姿态", "attitude.png"),
    ("环境与净距", "environment_metrics.png"),
]


HTML = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Drone Simulation Mode Panel</title><style>
:root{color-scheme:dark;--bg:#07111f;--card:#101f35;--line:#263b59;--blue:#4e9bff;--green:#55dda0;--orange:#ffb45f;--red:#ff6b7c;--text:#edf5ff;--muted:#9db0c9}
*{box-sizing:border-box}body{margin:0;font:14px Inter,system-ui,sans-serif;background:radial-gradient(circle at 15% 0,#16365a 0,transparent 34%),var(--bg);color:var(--text)}
header{padding:22px 28px;border-bottom:1px solid var(--line);background:#081525dd;position:sticky;top:0;z-index:2}.title{display:flex;align-items:center;gap:13px}.dot{width:13px;height:13px;border-radius:50%;background:var(--orange);box-shadow:0 0 15px currentColor}.dot.running{background:var(--green)}h1{font-size:22px;margin:0}header p{color:var(--muted);margin:5px 0 0}
main{max-width:1450px;margin:auto;padding:20px;display:grid;grid-template-columns:1.2fr .8fr;gap:16px}.card{background:linear-gradient(145deg,#12243d,#0d1a2d);border:1px solid var(--line);border-radius:14px;padding:17px;box-shadow:0 12px 30px #0004}.wide{grid-column:1/-1}h2{font-size:16px;margin:0 0 13px}.modes,.scenarios{display:grid;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));gap:9px}.mode,.scenario{border:1px solid var(--line);background:#0a1729;padding:12px;border-radius:10px;cursor:pointer;transition:.15s}.mode:hover,.scenario:hover,.selected{border-color:var(--blue);background:#102b4d;transform:translateY(-1px)}.mode b,.scenario b{display:block;margin-bottom:4px}.mode span,.scenario span{color:var(--muted);font-size:12px}
.toolbar{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:14px}button,select,input{border:1px solid #395579;background:#0a1729;color:var(--text);border-radius:8px;padding:9px 12px}button{cursor:pointer;background:#2568bd;border-color:#3e83db;font-weight:650}button.stop{background:#8d3041;border-color:#cf5268}button.secondary{background:#152840}.toggle{display:flex;align-items:center;gap:6px;color:var(--muted)}
.kv{display:grid;grid-template-columns:auto 1fr;gap:8px 12px}.kv span:nth-child(odd){color:var(--muted)}.ok{color:var(--green)}.warn{color:var(--orange)}.error{color:var(--red)}pre{margin:0;white-space:pre-wrap;word-break:break-word;background:#06101d;border:1px solid var(--line);padding:13px;border-radius:9px;max-height:350px;overflow:auto;color:#b9d6ff}
.configbar{display:flex;gap:8px;margin-bottom:10px}.configbar select{flex:1}textarea{width:100%;height:430px;background:#06101d;color:#dcecff;border:1px solid var(--line);border-radius:9px;padding:13px;font:13px ui-monospace,monospace;resize:vertical}.hint{color:var(--muted);font-size:12px;margin-top:8px}table{width:100%;border-collapse:collapse}th,td{padding:8px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}.pill{display:inline-block;padding:3px 7px;border-radius:99px;background:#173456;color:#9dc9ff}
@media(max-width:900px){main{grid-template-columns:1fr}.wide{grid-column:auto}}
</style></head><body><header><div class="title"><i id="dot" class="dot"></i><div><h1>无人机仿真模式管理 Panel</h1><p>免 source · 模式启停 · RViz2 · 日志 · YAML 调参</p></div></div></header><main>
<section class="card"><h2>运行模式</h2><div id="modes" class="modes"></div><div id="scenarioBlock"><h2 style="margin-top:16px">实验场景</h2><div id="scenarios" class="scenarios"></div></div><div class="toolbar"><label class="toggle"><input id="rviz" type="checkbox" checked>打开 RViz2</label><label>时长 <input id="duration" type="number" min="1" max="3600" placeholder="YAML 默认" style="width:105px"></label><button onclick="startMode()">启动模式</button><button class="stop" onclick="stopMode()">停止当前</button><button class="secondary" onclick="openGroundStation()">打开地面站</button></div><p id="message" class="hint"></p></section>
<section class="card"><h2>运行状态</h2><div id="status" class="kv"></div><h2 style="margin-top:18px">实时日志</h2><pre id="log">尚未启动</pre></section>
<section class="card wide"><h2>YAML 参数编辑</h2><div class="configbar"><select id="configSelect" onchange="loadConfig()"></select><button class="secondary" onclick="loadConfig()">重新加载</button><button onclick="saveConfig()">校验并保存</button></div><textarea id="editor" spellcheck="false"></textarea><div class="hint">只允许编辑 config 目录内的 YAML；保存时自动备份并执行语法、参数覆盖和跨配置安全校验，失败自动回滚。控制器、动力学、传感器、地图、规划和场景参数均可在此修改。</div></section>
<section class="card wide"><h2>最近实验结果</h2><table><thead><tr><th>场景</th><th>来源</th><th>最终误差/m</th><th>稳态误差/m</th><th>最小净距/m</th><th>状态</th></tr></thead><tbody id="results"></tbody></table></section>
</main><script>
let catalog={},selectedMode='hover',selectedScenario='hover';
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function api(path,body){const options=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};const r=await fetch(path,options),j=await r.json();if(!r.ok)throw Error(j.message||r.statusText);return j}
function selectMode(name){selectedMode=name;document.querySelectorAll('.mode').forEach(e=>e.classList.toggle('selected',e.dataset.name===name));scenarioBlock.style.display=name==='experiment'?'block':'none'}
function selectScenario(name){selectedScenario=name;document.querySelectorAll('.scenario').forEach(e=>e.classList.toggle('selected',e.dataset.name===name))}
async function init(){catalog=await api('/api/catalog');modes.innerHTML=Object.entries(catalog.modes).map(([k,v])=>`<div class="mode" data-name="${k}" onclick="selectMode('${k}')"><b>${esc(v.name)}</b><span>${esc(v.description)}</span></div>`).join('');scenarios.innerHTML=catalog.scenarios.map(v=>`<div class="scenario" data-name="${v.id}" onclick="selectScenario('${v.id}')"><b>${esc(v.name)}</b><span>${esc(v.description)}</span></div>`).join('');configSelect.innerHTML=catalog.configs.map(v=>`<option>${esc(v)}</option>`).join('');selectMode('hover');selectScenario('hover');await loadConfig();refresh()}
async function startMode(){try{message.textContent='正在启动…';const d=duration.value?Number(duration.value):null;const j=await api('/api/start',{mode:selectedMode,scenario:selectedScenario,rviz:rviz.checked,duration:d});message.textContent=j.message}catch(e){message.textContent=e.message}}
async function stopMode(){try{message.textContent=(await api('/api/stop',{})).message}catch(e){message.textContent=e.message}}
function openGroundStation(){window.open(catalog.ground_station_url,'_blank','noopener')}
async function loadConfig(){try{const j=await api('/api/config?name='+encodeURIComponent(configSelect.value));editor.value=j.content;message.textContent='已加载 '+j.name}catch(e){message.textContent=e.message}}
async function saveConfig(){try{const j=await api('/api/config',{name:configSelect.value,content:editor.value});message.textContent=j.message}catch(e){message.textContent=e.message}}
function num(v){return v==null?'—':Number(v).toFixed(4)}
async function refresh(){try{const s=await api('/api/status');dot.classList.toggle('running',s.running);status.innerHTML=`<span>状态</span><b class="${s.running?'ok':s.exit_code&&s.exit_code!==0?'error':'warn'}">${s.running?'运行中':'空闲'}</b><span>模式</span><b>${esc(s.mode||'—')}</b><span>场景</span><b>${esc(s.scenario||'—')}</b><span>PID</span><b>${s.pid||'—'}</b><span>运行时间</span><b>${s.elapsed_seconds.toFixed(1)} s</b><span>退出码</span><b>${s.exit_code??'—'}</b><span>结果目录</span><b>${esc(s.output_dir||'—')}</b>`;log.textContent=s.log_tail||'暂无日志';log.scrollTop=log.scrollHeight;const rs=await api('/api/results');results.innerHTML=rs.map(v=>`<tr><td>${esc(v.scenario)}</td><td>${esc(v.source)}</td><td>${num(v.final_position_error_m)}</td><td>${num(v.steady_state_error_m)}</td><td>${num(v.minimum_obstacle_clearance_m)}</td><td><span class="pill">已记录</span></td></tr>`).join('')}catch(e){message.textContent='Panel 连接失败：'+e.message}setTimeout(refresh,700)}
init();
</script></body></html>"""


# Task-oriented dashboard.  The legacy HTML above is intentionally retained as a
# compact fallback/reference while the handler publishes this richer acceptance UI.
HTML_V2 = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>无人机仿真任务展示 Panel</title><style>
:root{color-scheme:dark;--bg:#07111f;--card:#101f35;--line:#29415f;--blue:#5aa7ff;--green:#55dda0;--orange:#ffb45f;--red:#ff7282;--text:#edf5ff;--muted:#a9bad0}
*{box-sizing:border-box}body{margin:0;font:14px Inter,"Microsoft YaHei",system-ui,sans-serif;background:radial-gradient(circle at 15% 0,#16365a 0,transparent 32%),var(--bg);color:var(--text)}
header{padding:20px 28px;border-bottom:1px solid var(--line);background:#081525ee;position:sticky;top:0;z-index:3}.title{display:flex;align-items:center;gap:13px}.dot{width:13px;height:13px;border-radius:50%;background:var(--orange);box-shadow:0 0 15px currentColor}.dot.running{background:var(--green)}h1{font-size:22px;margin:0}header p{color:var(--muted);margin:5px 0 0}
main{max-width:1500px;margin:auto;padding:20px;display:grid;grid-template-columns:1.15fr .85fr;gap:16px}.card{background:linear-gradient(145deg,#12243d,#0d1a2d);border:1px solid var(--line);border-radius:14px;padding:17px;box-shadow:0 12px 30px #0004}.wide{grid-column:1/-1}h2{font-size:17px;margin:0 0 8px}.lead{color:var(--muted);margin:0 0 15px}.section-title{display:flex;align-items:end;justify-content:space-between;gap:12px;margin:20px 0 10px}.section-title:first-child{margin-top:0}.section-title span{color:var(--muted);font-size:12px}
.feature-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(235px,1fr));gap:10px}.feature{border:1px solid var(--line);background:#091728;border-radius:11px;padding:13px;display:flex;flex-direction:column;min-height:148px}.feature b{font-size:15px}.feature p{color:var(--muted);line-height:1.55;margin:7px 0 12px}.feature-actions{display:flex;gap:7px;flex-wrap:wrap;margin-top:auto}
button,select,input{border:1px solid #3d5d83;background:#0a1729;color:var(--text);border-radius:8px;padding:9px 12px}button{cursor:pointer;background:#2568bd;border-color:#4389dc;font-weight:650}button:hover{filter:brightness(1.14)}button.stop{background:#8d3041;border-color:#cf5268}button.secondary{background:#152840}button.tiny{font-size:12px;padding:7px 9px}button:disabled{opacity:.45;cursor:not-allowed}
.architecture{display:grid;grid-template-columns:repeat(7,minmax(110px,1fr));align-items:center;gap:8px;margin-top:14px}.node{border:1px solid #45688f;background:#0a1729;border-radius:10px;padding:13px 8px;text-align:center;min-height:72px;display:flex;align-items:center;justify-content:center;line-height:1.35}.arrow{text-align:center;color:var(--blue);font-size:22px}.topic-flow{margin-top:10px;border:1px dashed #45688f;border-radius:9px;padding:10px;color:#b9d6ff;text-align:center;line-height:1.55}.decoupled{color:var(--green)}
.modes,.scenarios{display:grid;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));gap:9px}.mode,.scenario{border:1px solid var(--line);background:#0a1729;padding:12px;border-radius:10px;cursor:pointer;transition:.15s}.mode:hover,.scenario:hover,.selected{border-color:var(--blue);background:#102b4d;transform:translateY(-1px)}.mode b,.scenario b{display:block;margin-bottom:4px}.mode span,.scenario span{color:var(--muted);font-size:12px}.toolbar{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:14px}.toggle{display:flex;align-items:center;gap:6px;color:var(--muted)}
.kv{display:grid;grid-template-columns:auto 1fr;gap:8px 12px}.kv span:nth-child(odd){color:var(--muted)}.ok{color:var(--green)}.warn{color:var(--orange)}.error{color:var(--red)}pre{margin:0;white-space:pre-wrap;word-break:break-word;background:#06101d;border:1px solid var(--line);padding:13px;border-radius:9px;max-height:350px;overflow:auto;color:#b9d6ff}.hint{color:var(--muted);font-size:12px;margin-top:8px}
.configbar{display:flex;gap:8px;margin-bottom:10px}.configbar select{flex:1}textarea{width:100%;height:390px;background:#06101d;color:#dcecff;border:1px solid var(--line);border-radius:9px;padding:13px;font:13px ui-monospace,monospace;resize:vertical}
table{width:100%;border-collapse:collapse}th,td{padding:9px;border-bottom:1px solid var(--line);text-align:right}th:first-child,td:first-child{text-align:left}tbody tr{cursor:pointer}tbody tr:hover{background:#102b4d}.pill{display:inline-block;padding:3px 7px;border-radius:99px;background:#173456;color:#9dc9ff}
.result-head{display:flex;gap:10px;align-items:center;justify-content:space-between;flex-wrap:wrap}.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:9px;margin:14px 0}.kpi{background:#091728;border:1px solid var(--line);border-radius:10px;padding:11px}.kpi span{display:block;color:var(--muted);font-size:12px}.kpi b{display:block;font-size:18px;margin-top:5px;word-break:break-word}.gallery{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:12px}.figure{background:#fff;border-radius:10px;padding:8px;color:#172338}.figure img{width:100%;display:block;border-radius:5px}.figure b{display:block;padding:6px 4px 2px}.coverage{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.coverage strong{font-size:20px;color:var(--green)}
@media(max-width:1100px){.architecture{grid-template-columns:1fr}.arrow{transform:rotate(90deg)}}@media(max-width:900px){main{grid-template-columns:1fr}.wide{grid-column:auto}.gallery{grid-template-columns:1fr}}
</style></head><body>
<header><div class="title"><i id="dot" class="dot"></i><div><h1>无人机仿真任务展示 Panel</h1><p>任务逐项实机展示 · RViz2 联动 · 全指标可视化 · 配置与交付物入口</p></div></div></header><main>
<section class="card wide"><div class="coverage"><strong id="coverageCount">—</strong><div><h2>任务要求一键展示矩阵</h2><p class="lead">每项均对应任务文档的验收点；“实机展示”启动真实 ROS2 节点，“查看实测图”读取正式实验结果。</p></div></div><div id="showcase"></div></section>
<section class="card wide"><h2>系统架构与数据流</h2><p class="lead"><span class="decoupled">控制算法、动力学模型和轨迹算法为纯 C++ 核心库，与 ROS2 解耦</span>；地图、消息适配、TF 与 RViz2 位于 ROS2 层。</p><div class="architecture"><div class="node">目标点 / 航点 / 解析轨迹</div><div class="arrow">→</div><div class="node">位置—姿态级联控制<br>纯算法</div><div class="arrow">→</div><div class="node">Mixer + 约束<br>纯算法</div><div class="arrow">→</div><div class="node">6-DOF 动力学<br>纯模型</div></div><div class="topic-flow">ROS2 适配层：/target_pose → /motor_rpm_command → /odom · /imu · /tf · /path &nbsp; | &nbsp; 地图/感知：障碍物 → 点云 → 体素 → 3D A* → /planned_path · /local_goal</div></section>
<section class="card"><h2>手动运行模式</h2><div id="modes" class="modes"></div><div id="scenarioBlock"><h2 style="margin-top:16px">正式实验场景</h2><div id="scenarios" class="scenarios"></div></div><div class="toolbar"><label class="toggle"><input id="rviz" type="checkbox" checked>打开 RViz2</label><label>时长 <input id="duration" type="number" min="1" max="3600" placeholder="YAML 默认" style="width:105px"></label><button onclick="startMode()">启动模式</button><button class="stop" onclick="stopMode()">停止当前</button><button class="secondary" onclick="openGroundStation()">打开地面站</button></div><p id="message" class="hint"></p></section>
<section class="card"><h2>运行状态</h2><div id="status" class="kv"></div><h2 style="margin-top:18px">实时日志</h2><pre id="log">尚未启动</pre></section>
<section class="card wide" id="resultsCard"><div class="result-head"><div><h2>实验指标与曲线可视化</h2><p class="lead">覆盖误差、到达、超调、净距、路径、姿态、RPM、扰动与噪声指标。</p></div><select id="resultSelect" onchange="loadResult(this.value)"></select></div><div id="kpis" class="kpis"></div><div id="gallery" class="gallery"></div></section>
<section class="card wide" id="configCard"><h2>YAML 参数编辑</h2><div class="configbar"><select id="configSelect" onchange="loadConfig()"></select><button class="secondary" onclick="loadConfig()">重新加载</button><button onclick="saveConfig()">校验并保存</button></div><textarea id="editor" spellcheck="false"></textarea><div class="hint">只允许编辑 config 目录内的 YAML；保存时自动备份并执行语法、参数覆盖和跨配置安全校验，失败自动回滚。</div></section>
<section class="card wide"><h2>最近实验索引</h2><table><thead><tr><th>场景</th><th>来源</th><th>最终误差 m</th><th>稳态误差 m</th><th>最小净距 m</th><th>操作</th></tr></thead><tbody id="results"></tbody></table></section>
</main><script>
let catalog={},selectedMode='hover',selectedScenario='hover';
const $=id=>document.getElementById(id),esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function api(path,body){const options=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};const r=await fetch(path,options);let j;try{j=await r.json()}catch{j={message:r.statusText}}if(!r.ok)throw Error(j.message||r.statusText);return j}
function selectMode(name){selectedMode=name;document.querySelectorAll('.mode').forEach(e=>e.classList.toggle('selected',e.dataset.name===name));$('scenarioBlock').style.display=name==='experiment'?'block':'none'}
function selectScenario(name){selectedScenario=name;document.querySelectorAll('.scenario').forEach(e=>e.classList.toggle('selected',e.dataset.name===name))}
function allFeatures(){return (catalog.showcase_sections||[]).flatMap(s=>s.items)}
function renderShowcase(){const sections=catalog.showcase_sections||[];$('coverageCount').textContent=`${sections.reduce((n,s)=>n+s.items.length,0)} 项入口`;$('showcase').innerHTML=sections.map(s=>`<div class="section-title"><div><h2>${esc(s.name)}</h2><span>${esc(s.description)}</span></div><span>${s.items.length} 项</span></div><div class="feature-grid">${s.items.map(v=>`<article class="feature"><b>${esc(v.name)}</b><p>${esc(v.requirement)}</p><div class="feature-actions"><button class="tiny" onclick="runFeature('${v.id}')">${v.action.type==='launch'?'实机展示':v.action.type==='view'?'查看实测图':v.action.type==='config'?'打开参数':'打开材料'}</button>${v.view&&v.action.type!=='view'?`<button class="tiny secondary" onclick="loadResult('${v.view}',true)">查看实测图</button>`:''}</div></article>`).join('')}</div>`).join('')}
async function runFeature(id){const item=allFeatures().find(v=>v.id===id);if(!item)return;const a=item.action;try{if(a.type==='launch'){selectedMode=a.mode;selectedScenario=a.scenario||'hover';selectMode(selectedMode);selectScenario(selectedScenario);$('rviz').checked=a.rviz!==false;$('message').textContent=`正在启动：${item.name}…`;const j=await api('/api/start',{mode:a.mode,scenario:a.scenario||'hover',rviz:a.rviz!==false,duration:null});$('message').textContent=`${item.name}：${j.message}`;window.scrollTo({top:$('status').getBoundingClientRect().top+window.scrollY-100,behavior:'smooth'})}else if(a.type==='view'){await loadResult(a.scenario,true)}else if(a.type==='config'){$('configSelect').value=a.name;await loadConfig();$('configCard').scrollIntoView({behavior:'smooth'})}else if(a.type==='file'){window.open('/file?name='+encodeURIComponent(a.name),'_blank','noopener')}}catch(e){$('message').textContent=`${item.name}：${e.message}`}}
async function init(){catalog=await api('/api/catalog');$('modes').innerHTML=Object.entries(catalog.modes).map(([k,v])=>`<div class="mode" data-name="${k}" onclick="selectMode('${k}')"><b>${esc(v.name)}</b><span>${esc(v.description)}</span></div>`).join('');$('scenarios').innerHTML=catalog.scenarios.map(v=>`<div class="scenario" data-name="${v.id}" onclick="selectScenario('${v.id}')"><b>${esc(v.name)}</b><span>${esc(v.description)}</span></div>`).join('');$('configSelect').innerHTML=catalog.configs.map(v=>`<option>${esc(v)}</option>`).join('');$('resultSelect').innerHTML=catalog.scenarios.map(v=>`<option value="${v.id}">${esc(v.name)} (${v.id})</option>`).join('');renderShowcase();selectMode('hover');selectScenario('hover');await Promise.all([loadConfig(),loadResult('hover')]);refresh()}
async function startMode(){try{$('message').textContent='正在启动…';const d=$('duration').value?Number($('duration').value):null;const j=await api('/api/start',{mode:selectedMode,scenario:selectedScenario,rviz:$('rviz').checked,duration:d});$('message').textContent=j.message}catch(e){$('message').textContent=e.message}}
async function stopMode(){try{$('message').textContent=(await api('/api/stop',{})).message}catch(e){$('message').textContent=e.message}}
function openGroundStation(){window.open(catalog.ground_station_url,'_blank','noopener')}
async function loadConfig(){try{const j=await api('/api/config?name='+encodeURIComponent($('configSelect').value));$('editor').value=j.content;$('message').textContent='已加载 '+j.name}catch(e){$('message').textContent=e.message}}
async function saveConfig(){try{const j=await api('/api/config',{name:$('configSelect').value,content:$('editor').value});$('message').textContent=j.message}catch(e){$('message').textContent=e.message}}
const metrics=[['final_position_error_m','最终位置误差','m'],['arrival_time_s','到达时间','s'],['maximum_altitude_overshoot_m','最大高度超调','m'],['steady_state_error_m','稳态误差','m'],['rms_position_error_m','RMS 误差','m'],['minimum_obstacle_clearance_m','最小障碍净距','m'],['path_length_m','实际路径长度','m'],['planned_path_length_m','规划路径长度','m'],['maximum_tilt_deg','最大倾角','°'],['rpm_saturation_ratio','RPM 饱和比例',''],['disturbance_peak_error_m','扰动峰值误差','m'],['disturbance_recovery_time_s','扰动恢复时间','s'],['sensor_position_noise_rms_m','传感位置噪声 RMS','m'],['mission_status','任务状态',''],['planner_status','规划器状态',''],['fault_status','故障状态','']];
function metricValue(v,u){if(v===null||v===undefined)return '—';if(typeof v==='number')return `${v.toFixed(4)}${u?' '+u:''}`;if(typeof v==='string'&&v.startsWith('{')){try{const j=JSON.parse(v);return esc(`mode=${j.mode??'—'} · active=${j.active??'—'} · modified=${j.modified??0} · dropped=${j.dropped??0}`)}catch{}}return esc(v)}
async function loadResult(scenario,scroll=false){try{const j=await api('/api/result-detail?scenario='+encodeURIComponent(scenario));$('resultSelect').value=scenario;$('kpis').innerHTML=metrics.filter(([k])=>j.summary[k]!==undefined).map(([k,n,u])=>`<div class="kpi"><span>${n}</span><b>${metricValue(j.summary[k],u)}</b></div>`).join('');$('gallery').innerHTML=j.artifacts.map(a=>`<figure class="figure"><img loading="lazy" src="${a.url}" alt="${esc(a.title)}"><b>${esc(a.title)}</b></figure>`).join('');if(scroll)$('resultsCard').scrollIntoView({behavior:'smooth'})}catch(e){$('kpis').innerHTML=`<div class="error">${esc(e.message)}</div>`}}
function num(v){return v==null?'—':Number(v).toFixed(4)}
async function refresh(){try{const s=await api('/api/status');$('dot').classList.toggle('running',s.running);$('status').innerHTML=`<span>状态</span><b class="${s.running?'ok':s.exit_code&&s.exit_code!==0?'error':'warn'}">${s.running?'运行中':'空闲'}</b><span>模式</span><b>${esc(s.mode||'—')}</b><span>场景</span><b>${esc(s.scenario||'—')}</b><span>PID</span><b>${s.pid||'—'}</b><span>运行时间</span><b>${s.elapsed_seconds.toFixed(1)} s</b><span>退出码</span><b>${s.exit_code??'—'}</b><span>结果目录</span><b>${esc(s.output_dir||'—')}</b>`;$('log').textContent=s.log_tail||'暂无日志';$('log').scrollTop=$('log').scrollHeight;const rs=await api('/api/results');$('results').innerHTML=rs.map(v=>`<tr onclick="loadResult('${esc(v.scenario)}',true)"><td>${esc(v.scenario)}</td><td>${esc(v.source)}</td><td>${num(v.final_position_error_m)}</td><td>${num(v.steady_state_error_m)}</td><td>${num(v.minimum_obstacle_clearance_m)}</td><td><span class="pill">查看全图</span></td></tr>`).join('')}catch(e){$('message').textContent='Panel 连接失败：'+e.message}setTimeout(refresh,800)}
init();
</script></body></html>"""


class ModeManager:
    def __init__(self, settings):
        self.settings = settings
        self.lock = threading.RLock()
        self.process = None
        self.log_handle = None
        self.mode = None
        self.scenario = None
        self.started_monotonic = None
        self.exit_code = None
        self.output_dir = None
        self.log_path = None
        self.stop_requested = False
        self.scenarios = self.load_scenarios()
        self.descriptions = settings.get("scenario_descriptions", {})
        self.editable_configs = self.load_editable_configs()
        self.run_root = (ROOT / settings["panel_run_root"]).resolve()
        self.log_root = (ROOT / settings["panel_log_root"]).resolve()
        self.backup_root = (ROOT / settings["panel_backup_root"]).resolve()
        for path in (self.run_root, self.log_root, self.backup_root):
            path.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def load_scenarios():
        launch = yaml.safe_load((CONFIG_DIR / "launch.yaml").read_text(encoding="utf-8"))
        return [str(value) for value in launch["experiment"]["scenarios"]]

    def load_editable_configs(self):
        configured = [str(value) for value in self.settings["panel_editable_configs"]]
        available = {path.name for path in CONFIG_DIR.glob("*.yaml")}
        if len(configured) != len(set(configured)) or any(name not in available for name in configured):
            raise ValueError("panel_editable_configs contains duplicates or missing files")
        return configured

    def catalog(self):
        return {
            "modes": MODE_CATALOG,
            "scenarios": [
                {
                    "id": scenario,
                    "name": self.descriptions.get(scenario, {}).get("name", scenario),
                    "description": self.descriptions.get(scenario, {}).get("description", "YAML 场景"),
                }
                for scenario in self.scenarios
            ],
            "configs": self.editable_configs,
            "ground_station_url": self.settings["panel_ground_station_url"],
            "showcase_sections": SHOWCASE_SECTIONS,
            "published_files": {
                key: {"name": title, "available": (ROOT / relative).is_file()}
                for key, (title, relative) in PUBLISHED_FILES.items()
            },
        }

    def _refresh_process(self):
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                self.exit_code = code
                self.process = None
                if self.log_handle is not None:
                    self.log_handle.close()
                    self.log_handle = None

    def status(self):
        with self.lock:
            self._refresh_process()
            running = self.process is not None
            elapsed = 0.0 if self.started_monotonic is None else time.monotonic() - self.started_monotonic
            tail = ""
            if self.log_path and self.log_path.exists():
                lines = self.log_path.read_text(encoding="utf-8", errors="replace").splitlines()
                tail = "\n".join(lines[-int(self.settings["panel_log_tail_lines"]):])
            return {
                "running": running,
                "pid": self.process.pid if running else None,
                "mode": self.mode,
                "scenario": self.scenario,
                "elapsed_seconds": elapsed,
                "exit_code": self.exit_code,
                "output_dir": str(self.output_dir) if self.output_dir else None,
                "log_path": str(self.log_path) if self.log_path else None,
                "log_tail": tail,
                "stop_requested": self.stop_requested,
            }

    def validate_start(self, data):
        mode = str(data.get("mode", ""))
        if mode not in MODE_CATALOG:
            raise ValueError("unknown mode")
        scenario = str(data.get("scenario", "hover"))
        if mode == "experiment" and scenario not in self.scenarios:
            raise ValueError("unknown scenario")
        rviz = data.get("rviz", True)
        if not isinstance(rviz, bool):
            raise ValueError("rviz must be boolean")
        duration = data.get("duration")
        if duration is not None:
            duration = float(duration)
            if not (1.0 <= duration <= float(self.settings["panel_maximum_duration_seconds"])):
                raise ValueError("duration is outside configured bounds")
        return mode, scenario, rviz, duration

    def start(self, data):
        mode, scenario, rviz, duration = self.validate_start(data)
        with self.lock:
            self._refresh_process()
            if self.process is not None:
                raise RuntimeError("another mode is already running; stop it first")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            command = [str(ROOT / "start_sim.sh")]
            output_dir = None
            if mode == "experiment":
                output_dir = self.run_root / f"{scenario}_{stamp}"
                command.extend(["experiment", scenario])
                if rviz:
                    command.append("--rviz")
                command.append(f"output_dir:={output_dir}")
                if duration is not None:
                    command.append(f"duration:={duration}")
            elif mode == "hover":
                command.append("hover")
                if not rviz:
                    command.append("use_rviz:=false")
            elif mode == "multi":
                command.append("multi")
                if not rviz:
                    command.append("use_rviz:=false")
            elif mode == "ground-station":
                command.append("ground-station")
                command.append(f"use_rviz:={'true' if rviz else 'false'}")
            else:
                command.append("batch")
            self.log_path = self.log_root / f"{mode}_{scenario}_{stamp}.log"
            self.log_handle = self.log_path.open("w", encoding="utf-8")
            self.log_handle.write("COMMAND: " + " ".join(str(value) for value in command) + "\n")
            self.log_handle.flush()
            self.process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdout=self.log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
            self.mode = mode
            self.scenario = scenario if mode == "experiment" else None
            self.output_dir = output_dir
            self.started_monotonic = time.monotonic()
            self.exit_code = None
            self.stop_requested = False
            process = self.process
            time.sleep(float(self.settings["panel_startup_grace_seconds"]))
            code = process.poll()
            if code is not None:
                self.exit_code = code
                self.process = None
                if self.log_handle is not None:
                    self.log_handle.close()
                    self.log_handle = None
                detail = self.log_path.read_text(
                    encoding="utf-8", errors="replace"
                ).splitlines()
                raise RuntimeError(
                    f"{mode} failed during startup (exit {code}): "
                    + (detail[-1] if detail else "no log output")
                )
            return {"message": f"started {mode}", "pid": self.process.pid}

    def stop(self):
        with self.lock:
            self._refresh_process()
            if self.process is None:
                return {"message": "no mode is running"}
            process = self.process
            self.stop_requested = True
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=float(self.settings["panel_sigint_timeout_seconds"]))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=float(self.settings["panel_sigterm_timeout_seconds"]))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        with self.lock:
            self.exit_code = process.returncode
            self.process = None
            if self.log_handle is not None:
                self.log_handle.close()
                self.log_handle = None
        return {"message": "mode stopped", "exit_code": process.returncode}

    def config_path(self, name):
        if not isinstance(name, str) or name not in self.editable_configs:
            raise ValueError("config is not editable")
        path = (CONFIG_DIR / name).resolve()
        if path.parent != CONFIG_DIR.resolve():
            raise ValueError("invalid config path")
        return path

    def read_config(self, name):
        path = self.config_path(name)
        return {"name": name, "content": path.read_text(encoding="utf-8")}

    def write_config(self, name, content):
        if not isinstance(content, str) or not content.strip():
            raise ValueError("config content must be non-empty text")
        encoded = content.encode("utf-8")
        if len(encoded) > int(self.settings["panel_maximum_config_bytes"]):
            raise ValueError("config is too large")
        parsed = yaml.safe_load(content)
        if not isinstance(parsed, dict):
            raise ValueError("YAML root must be a mapping")
        path = self.config_path(name)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        backup_path = self.backup_root / f"{name}.{stamp}.bak"
        shutil.copy2(path, backup_path)
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False, suffix=".tmp"
        ) as temporary:
            temporary.write(content)
            temporary_path = Path(temporary.name)
        os.replace(temporary_path, path)
        if bool(self.settings.get("panel_verify_yaml_on_save", True)):
            verification = subprocess.run(
                [str(ROOT / "scripts" / "verify_yaml_parameters.py")],
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
            if verification.returncode != 0:
                shutil.copy2(backup_path, path)
                detail = verification.stderr.strip() or verification.stdout.strip()
                raise ValueError(
                    "cross-config validation failed; the previous file was restored: "
                    + detail[-2000:]
                )
        return {
            "message": f"syntax/cross-config validated and saved {name}",
            "name": name,
            "backup": str(backup_path.relative_to(ROOT)),
        }

    def results(self):
        candidates = [
            (path, "正式批次")
            for path in (ROOT / "artifacts" / "experiments").glob("*/summary.json")
        ]
        candidates.extend((path, "Panel") for path in self.run_root.glob("*/summary.json"))
        candidates.sort(key=lambda item: item[0].stat().st_mtime, reverse=True)
        values = []
        for path, source in candidates[: int(self.settings["panel_maximum_results"])]:
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                continue
            values.append(
                {
                    "scenario": data.get("scenario", path.parent.name),
                    "source": source,
                    "final_position_error_m": data.get("final_position_error_m"),
                    "steady_state_error_m": data.get("steady_state_error_m"),
                    "minimum_obstacle_clearance_m": data.get("minimum_obstacle_clearance_m"),
                }
            )
        return values

    def result_detail(self, scenario):
        if not isinstance(scenario, str) or scenario not in self.scenarios:
            raise ValueError("unknown scenario")
        result_dir = (ROOT / "artifacts" / "experiments" / scenario).resolve()
        root = (ROOT / "artifacts" / "experiments").resolve()
        if result_dir.parent != root:
            raise ValueError("invalid result path")
        summary_path = result_dir / "summary.json"
        if not summary_path.is_file():
            raise ValueError("formal result is not available for this scenario")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        artifacts = [
            {
                "title": title,
                "name": name,
                "url": f"/artifact?scenario={scenario}&name={name}",
            }
            for title, name in RESULT_ARTIFACTS
            if (result_dir / name).is_file()
        ]
        return {"scenario": scenario, "summary": summary, "artifacts": artifacts}

    def artifact_path(self, scenario, name):
        if not isinstance(scenario, str) or scenario not in self.scenarios:
            raise ValueError("unknown scenario")
        allowed = {filename for _, filename in RESULT_ARTIFACTS}
        if not isinstance(name, str) or name not in allowed:
            raise ValueError("artifact is not published")
        result_dir = (ROOT / "artifacts" / "experiments" / scenario).resolve()
        path = (result_dir / name).resolve()
        if path.parent != result_dir or not path.is_file():
            raise ValueError("artifact is not available")
        return path

    @staticmethod
    def published_file_path(name):
        if not isinstance(name, str) or name not in PUBLISHED_FILES:
            raise ValueError("file is not published")
        _, relative = PUBLISHED_FILES[name]
        path = (ROOT / relative).resolve()
        try:
            path.relative_to(ROOT.resolve())
        except ValueError as exception:
            raise ValueError("invalid published file path") from exception
        if not path.is_file():
            raise ValueError(f"published file is not available: {relative}")
        return path


class Handler(BaseHTTPRequestHandler):
    server_version = "DroneModePanel/1.0"

    def log_message(self, format_string, *args):
        if self.server.verbose:
            print(format_string % args)

    def send_json(self, value, status=HTTPStatus.OK):
        payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def send_file(self, path):
        payload = path.read_bytes()
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if path.suffix.lower() in {".md", ".yaml", ".yml", ".txt", ".csv", ".json"}:
            content_type += "; charset=utf-8"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header(
            "Content-Disposition", f"inline; filename*=UTF-8''{quote(path.name)}"
        )
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/":
                payload = HTML_V2.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                return
            if parsed.path == "/api/catalog":
                self.send_json(self.server.manager.catalog())
                return
            if parsed.path == "/api/status":
                self.send_json(self.server.manager.status())
                return
            if parsed.path == "/api/results":
                self.send_json(self.server.manager.results())
                return
            if parsed.path == "/api/result-detail":
                scenario = parse_qs(parsed.query).get("scenario", [None])[0]
                self.send_json(self.server.manager.result_detail(scenario))
                return
            if parsed.path == "/api/config":
                name = parse_qs(parsed.query).get("name", [None])[0]
                self.send_json(self.server.manager.read_config(name))
                return
            if parsed.path == "/artifact":
                query = parse_qs(parsed.query)
                scenario = query.get("scenario", [None])[0]
                name = query.get("name", [None])[0]
                self.send_file(self.server.manager.artifact_path(scenario, name))
                return
            if parsed.path == "/file":
                name = parse_qs(parsed.query).get("name", [None])[0]
                self.send_file(self.server.manager.published_file_path(name))
                return
            self.send_json({"message": "not found"}, HTTPStatus.NOT_FOUND)
        except (ValueError, OSError) as exception:
            self.send_json({"message": str(exception)}, HTTPStatus.BAD_REQUEST)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > int(self.server.maximum_request_bytes):
            self.send_json({"message": "invalid request size"}, HTTPStatus.BAD_REQUEST)
            return
        try:
            data = json.loads(self.rfile.read(length))
            path = urlparse(self.path).path
            if path == "/api/start":
                self.send_json(self.server.manager.start(data), HTTPStatus.ACCEPTED)
                return
            if path == "/api/stop":
                self.send_json(self.server.manager.stop())
                return
            if path == "/api/config":
                self.send_json(
                    self.server.manager.write_config(data.get("name"), data.get("content"))
                )
                return
            self.send_json({"message": "not found"}, HTTPStatus.NOT_FOUND)
        except RuntimeError as exception:
            self.send_json({"message": str(exception)}, HTTPStatus.CONFLICT)
        except (
            ValueError,
            TypeError,
            json.JSONDecodeError,
            OSError,
            yaml.YAMLError,
        ) as exception:
            self.send_json({"message": str(exception)}, HTTPStatus.BAD_REQUEST)


def load_settings(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    settings = data["mode_panel"]
    required_positive = (
        "panel_port", "panel_maximum_request_bytes", "panel_maximum_config_bytes",
        "panel_log_tail_lines", "panel_maximum_results", "panel_maximum_duration_seconds",
        "panel_sigint_timeout_seconds", "panel_sigterm_timeout_seconds",
        "panel_startup_grace_seconds",
    )
    if any(float(settings[name]) <= 0 for name in required_positive):
        raise ValueError("mode panel numeric settings must be positive")
    return settings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--bind")
    parser.add_argument("--port", type=int)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    settings = load_settings(args.config)
    bind = args.bind or str(settings["panel_bind_address"])
    port = args.port or int(settings["panel_port"])
    if not bind or not (1 <= port <= 65535):
        raise SystemExit("invalid bind address or port")
    manager = ModeManager(settings)
    server = ThreadingHTTPServer((bind, port), Handler)
    server.manager = manager
    server.maximum_request_bytes = int(settings["panel_maximum_request_bytes"])
    server.verbose = args.verbose
    print(f"Mode panel: http://{bind}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop()
        server.server_close()


if __name__ == "__main__":
    main()

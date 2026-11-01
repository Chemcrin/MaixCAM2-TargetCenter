
---

## 一、项目一句话

2025 全国大学生电子设计竞赛本科组 **E 题《简易自行瞄准装置》** 的视觉模块。

- **靶心定义**：A4 靶纸四周 1.8cm 黑胶带矩形的**两条对角线交点**（射影不变量）。
  物理靶心是 ≤0.1cm 的红点，在 165cm 处仅约 0.67px、物理不可见，**只能由黑框几何反算**。
- **输出**：靶心像素坐标 `(u, v)`（A 轨另给局部尺度 `px_per_cm`），经串口交给云台主控解算激光指向。
- **运行平台**：MaixCAM2（MaixPy v4 / MaixCDK，Ubuntu），屏幕 640×480。
- **工程形态**：`main.py` + `app.yaml` + `config.py` + `vision/`，另生成单文件版 `dist/main_single.py`。

目标：识别 A4 靶纸四周 1.8cm 黑胶带矩形的「两条对角线交点」= 靶心像素坐标 (u,v)，
经串口交给云台主控解算激光指向。物理靶心是 ≤0.1cm 红点，165cm 处仅 0.67px、物理不可见，
只能由黑框几何反算。运行平台 MaixCAM2（MaixPy v4 / MaixCDK，Ubuntu，屏 640×480）。
识别到黑框中心后，要操控着两个张大头步进电机带动云台旋转yaw和pitch轴，使激光靶心和黑框中心重合

---


### 技术路线差异（可参考）

- **A 轨**：灰度二值化 + 环/亮面双族候选轮廓 + 结构打分 + 角点亚像素精修；靶心用**极角法取四角 → 对角线交点**；One Euro 滤波。
- **B 轨**：Canny + `approxPolyDP` 凸四边形配对内外框 + **显式 8×8 单应** + 正视面 72 点圆逆映射求中心；四边灰度剖面真伪校验；窄带灰度梯度 + `fitLine` 亚像素精修做跟踪；Otsu/多档暗阈值 + Canny 兜底。


---

## 四、当前存在的问题与风险

### A. 阻塞性（板端）

1. **显示链路仍是最大不确定性**：`MAIXVISION=0` 方案使 MaixVision **画面预览无图**（终端日志正常）；
   从设备本地启动才全帧。判据：`[preview] show 探针 ≈505ms` = 图传忙等；`1~5ms` = 健康。
2. **VO/fb0 显示子系统会渐进劣化直至卡死**（历史实测：24.7 → 12.4 → 1.8 fps → 卡在 `SAMPLE_VO_StartChn`）；
   需**重启设备**恢复。属设备侧问题，非代码。
3. **v0.5.2 尚未板上验证**：快移捕获 / 重捕获改善待实测确认。


### D. 代码质量

10. **B 轨 12 个「死参数」**（改了无效，易误导调参）：`SEARCH_BUDGET_MS`（仅拆帧模式生效）、
    `SEARCH_MAX_PROPOSALS`、`SEARCH_GEOMETRY_STEPS`、`MAX_CONTOURS`、`LINE_MAX_PROPOSALS`、
    `LINE_SEARCH_INTERVAL`、`ACQUIRE_CONFIRM_FRAMES`、`CONFIRM_CORNER_PX`、`RECHECK_INTERVAL`、
    `MAX_LINE_RESIDUAL_PX`、`MIN_TRANSITION_COVERAGE`、`MAX_BOUNDARY_WIDTH_ERROR`。
11. **死代码**：`rectangle_center.py::_legacy_search`（旧搜索，保留为兜底但从未被调用）。
12. **`tools/apply_gimbal_track_patch.py` 长期红**：v20 血统，对 v0.4.x/v0.5.x 代码结构性误报，
   挡住 `git_round -m` → 需手工提交。**勿去「修」它**。

---


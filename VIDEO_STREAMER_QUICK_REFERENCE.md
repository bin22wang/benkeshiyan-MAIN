/**
 * @file VIDEO_STREAMER_QUICK_REFERENCE.md
 * @brief 视频流传输模块 - 快速参考指南（一页纸版本）
 */

# 视频流传输模块快速参考

## ? 3 步快速集成

### 1?? 包含头文件
```c
#include "video_streamer.h"
```

### 2?? 初始化（app_main 中）
```c
// 在 camera_ui_init() 和 udp_comm_init() 之后调用
video_streamer_init();
```

### 3?? 处理 V 协议（UDP 回调中）
```c
static void on_udp_data_received(const char *data, size_t len,
                                 const char *src_ip, uint16_t src_port)
{
    proto_parse_message(data, len);
    
    // 处理视频控制命令
    proto_video_cmd_t cmd = proto_consume_video_cmd();
    if (cmd == VIDEO_CMD_START) {
        video_streamer_start();
    } else if (cmd == VIDEO_CMD_STOP) {
        video_streamer_stop();
    }
}
```

---

## ? V 协议消息格式

### 接收（服务器 → 小车）
| 消息 | 格式 | 作用 |
|-----|------|------|
| 开启 | `[V,START]` | 启动视频流 |
| 关闭 | `[V,STOP]` | 停止视频流 |

### 发送（小车 → 服务器）
```
→ [V,META,0,2100]              # 元数据：序列号=0，总字节=2100
→ [V,CHUNK,0,0,3,900]\n<900字节>  # 分片0/3（900字节）
→ [V,CHUNK,0,1,3,900]\n<900字节>  # 分片1/3（900字节）
→ [V,CHUNK,0,2,3,300]\n<300字节>  # 分片2/3（300字节）
```

---

## ? 关键 API

```c
// 初始化（仅调用一次）
void video_streamer_init(void);

// 启动图传
void video_streamer_start(void);

// 停止图传
void video_streamer_stop(void);

// 消费协议命令（在回调中）
proto_video_cmd_t proto_consume_video_cmd(void);
```

---

## ?? 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| JPEG 质量 | 30 | 0~100，平衡压缩率和清晰度 |
| 分片大小 | 900 B | 单个 UDP 分片最大字节数 |
| 分片延时 | 2 ms | 分片间延时，防止 UDP 拥堵 |
| 帧间延时 | 100 ms | 帧与帧之间的发送间隔（≈10FPS） |
| 任务优先级 | IDL+2 | 中等优先级，不阻塞系统 |

**调整建议**：
- 降低帧率：增大 `FRAME_INTERVAL_MS`
- 提升清晰度：增大 `JPEG_QUALITY`
- 加快传输：减小 `CHUNK_DELAY_MS`（需谨慎测试）

---

## ? 文件清单

| 文件 | 说明 |
|-----|------|
| `video_streamer.h` | 公开 API（不修改） |
| `video_streamer.c` | 核心实现（独立任务） |
| `proto_parser.h/c` | 已更新，添加 V 协议支持 |
| `CMakeLists.txt` | 已更新，包含编译源文件 |
| `VIDEO_STREAMER_INTEGRATION.md` | 完整集成指南 |
| `app_main_integration_example.c` | 集成代码示例 |

---

## ? 集成验证检查表

```
[ ] CMakeLists.txt 包含 "modules/video_streamer.c"
[ ] app_main.c 包含 #include "video_streamer.h"
[ ] app_main() 中调用了 video_streamer_init()
[ ] UDP 回调中添加了 VIDEO_CMD 处理
[ ] 摄像头正常显示在屏幕上
[ ] UDP 通信已连接到服务器
[ ] idf.py clean && idf.py build 无错误
[ ] 发送 [V,START] 后有日志输出
[ ] 抓包确认收到 [V,META] 和 [V,CHUNK] 消息
```

---

## ? 对症下药

| 现象 | 可能原因 | 解决方案 |
|-----|--------|--------|
| 编译失败 `undefined reference` | 源文件未编译 | 检查 CMakeLists.txt |
| 启动后无数据 | 未调用 init | 检查 app_main.c 初始化顺序 |
| 收到命令无反应 | 回调未处理 | 检查 UDP 回调代码 |
| 帧率太低 | 网络拥堵或参数 | 调大 FRAME_INTERVAL_MS |
| Wi-Fi 断连 | UDP 过载 | 增加 CHUNK_DELAY_MS |

---

## ? 架构特点

? **完全解耦**：独立任务，不干涉 recon_mgr  
? **高效压缩**：frame2jpg RGB565→JPEG (~20:1 压缩)  
? **防拥堵**：分片+延时机制，稳定传输  
? **内存优化**：流式处理，PSRAM 缓冲  
? **标准协议**：[V,START/STOP/META/CHUNK]  

---

## ? 性能指标

| 指标 | 值 |
|------|-----|
| 分辨率 | QVGA (320×240) |
| 帧率 | ~10 FPS (可配置 5~10) |
| 压缩率 | ~20:1 (RGB565 → JPEG@Q30) |
| 单帧大小 | ~2-3 KB |
| 网络带宽 | ~50-100 KB/s |
| CPU 占用 | ~5-10% (中等优先级任务) |
| 单个分片 | 900 字节（包括头) |

---

## ? 集成检查清单（按顺序）

```
1. [ ] 阅读 VIDEO_STREAMER_INTEGRATION.md 第 1-2 节
2. [ ] 查看 app_main_integration_example.c
3. [ ] 复制初始化代码到 app_main.c
4. [ ] 复制 UDP 回调处理代码
5. [ ] 更新 CMakeLists.txt（如未自动）
6. [ ] idf.py build 编译验证
7. [ ] 测试 [V,START] 命令
8. [ ] 接收端验证 META/CHUNK 数据包
9. [ ] 调整参数（帧率、质量等）适配实际应用
```

---

## ? 进阶资源

- **完整文档**：`VIDEO_STREAMER_INTEGRATION.md`
- **代码示例**：`app_main_integration_example.c`
- **协议细节**：见本文档第 3 节
- **Python 解析**：见集成指南第 6 节

---

**快速开始**：复制集成示例代码 → idf.py build → 发送 [V,START] ?

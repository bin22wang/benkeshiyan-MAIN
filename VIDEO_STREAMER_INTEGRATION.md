/**
 * @file VIDEO_STREAMER_INTEGRATION.md
 * @brief 高度模块化 UDP 按需图传功能集成指南
 *
 * 本文档说明如何将 video_streamer 模块集成到现有系统中。
 */

# 视频流传输模块集成指南

## 1. 文件清单

已创建的新文件：
- `main/modules/video_streamer.h`  - 公开 API 头文件
- `main/modules/video_streamer.c`  - 核心实现
- `main/CMakeLists.txt`             - 已更新，包含 video_streamer.c

已修改的文件：
- `main/modules/proto_parser.h`     - 添加 V 协议数据类型和 API
- `main/modules/proto_parser.c`     - 添加 V 协议解析器和 accessor 函数

## 2. 集成步骤

### 步骤 A：在 app_main.c 中初始化模块

在 `app_main.c` 的初始化阶段（在所有依赖模块初始化完成后）调用：

```c
#include "video_streamer.h"

void app_main(void)
{
    // ... 其他初始化代码（WiFi、UDP、摄像头等）...
    
    // 摄像头初始化 ?（已存在）
    camera_ui_init();
    
    // UDP 通信初始化 ?（已存在）
    udp_comm_init(server_ip, server_port, local_port);
    
    // 协议解析器初始化 ?（已存在）
    proto_parser_init("CAR1");
    
    // ========== 新增：初始化视频流传输模块 ==========
    video_streamer_init();
    // ===================================================
    
    // ... 其他初始化代码...
}
```

### 步骤 B：在 UDP 消息处理中集成 V 协议控制命令

打开 `main/app_main.c`，找到 UDP 数据接收的当前处理流程或 `proto_parse_message()` 调用点。

#### 方案 1：修改现有的 UDP 回调（推荐）

在 `app_main.c` 中的 UDP 接收回调函数（例如 `on_udp_data_received`）中添加 V 协议命令处理：

```c
static void on_udp_data_received(const char *data, size_t len,
                                 const char *src_ip, uint16_t src_port)
{
    ESP_LOGI(TAG, "[RX] from %s:%u, len=%d", src_ip, src_port, (int)len);

    // 解析所有协议消息（包括 V 协议）
    proto_parse_message(data, len);
    recon_mgr_handle_message(data, len);
    
    // ========== 新增：处理 V 协议视频流控制命令 ==========
    proto_video_cmd_t video_cmd = proto_consume_video_cmd();
    switch (video_cmd) {
        case VIDEO_CMD_START:
            ESP_LOGI(TAG, "Video streaming START");
            video_streamer_start();
            break;
        case VIDEO_CMD_STOP:
            ESP_LOGI(TAG, "Video streaming STOP");
            video_streamer_stop();
            break;
        case VIDEO_CMD_NONE:
            // 无新的视频命令
            break;
    }
    // ====================================================
}
```

#### 方案 2：在 proto_parser.c 中拦截（可选高级方案）

如果希望将视频流控制完全隐藏在协议解析层，可以在 `proto_parser.c` 中的 `parse_video_cmd()` 函数末尾直接调用 video_streamer API：

```c
static void parse_video_cmd(const char *data)
{
    ...
    
    if (strncmp(data, "[V,START]", 9) == 0) {
        s_video_cmd.cmd = VIDEO_CMD_START;
        s_video_cmd.valid = true;
        
        // 直接调用（可选）
        // video_streamer_start();
        
        ESP_LOGI(TAG, "[VIDEO] START command received");
        return;
    }
    ...
}
```

但不推荐，因为这样会紧耦合协议解析和业务逻辑。

### 步骤 C：添加必要的头文件包含

在 `app_main.c` 顶部添加：

```c
#include "video_streamer.h"
#include "proto_parser.h"  // 如果还未包含
```

## 3. 通信协议详解

### V 协议格式

#### 接收消息（来自服务器/其他小车）

1. **开启图传**
   ```
   [V,START]
   ```

2. **关闭图传**
   ```
   [V,STOP]
   ```

#### 发送消息（小车 → 服务器）

运行图传时，小车会按以下格式发送视频数据：

1. **元数据包（META）**
   ```
   [V,META,<序列号>,<JPEG总字节数>]
   ```
   - `序列号`：当前 JPEG 帧的序列号（从 0 开始，递增）
   - `JPEG总字节数`：该帧的完整 JPEG 数据大小

2. **分片数据包（CHUNK）**
   ```
   [V,CHUNK,<序列号>,<分片索引IDX>,<总分片数TOT>,<本片大小>]\n<二进制JPEG数据>
   ```
   - `序列号`：与 META 包相同
   - `分片索引IDX`：当前分片的索引，从 0 开始
   - `总分片数TOT`：该 JPEG 的总分片数
   - `本片大小`：当前分片的数据大小（字节）
   - `\n`：文本头与二进制数据的分隔符
   - `<二进制数据>`：当前分片的 JPEG 数据

**发送流程示例**（假设 JPEG 共 2100 字节，分为 3 片）：

```
→ [V,META,0,2100]
→ [V,CHUNK,0,0,3,900]\n<900字节JPEG数据>
  (延时 2ms)
→ [V,CHUNK,0,1,3,900]\n<900字节JPEG数据>
  (延时 2ms)
→ [V,CHUNK,0,2,3,300]\n<300字节JPEG数据>
  (延时 2ms)
← 帧间延时 100ms
→ [V,META,1,2050]
... (下一帧)
```

## 4. 关键性能参数

| 参数 | 值 | 说明 |
|------|-----|------|
| **JPEG 压缩质量** | 30 | 均衡压缩率和图质（0~100） |
| **最大分片大小** | 900 字节 | 单个 UDP 分片的最大数据量 |
| **分片间延时** | 2 ms | 防止 UDP 队列拥堵 |
| **帧间延时** | 100 ms | 对应约 10 FPS 实时帧率 |
| **任务优先级** | tskIDLE_PRIORITY + 2 | 中等优先级，不阻塞系统 |
| **任务栈大小** | 4 KB | 栈内存占用 |

## 5. 设计特点

### 高度模块化

- ? **独立任务运行**：视频流传输在独立的 FreeRTOS 任务中进行，不干涉现有控制逻辑
- ? **零耦合协议解析**：与 recon_mgr、vel_solver 等完全解耦
- ? **通过 UDP 接口通信**：复用现有的 udp_comm 模块，无额外硬件需求
- ? **标准 C 语言库依赖**：仅使用 FreeRTOS、ESP-IDF 标准 API

### 内存高效

- ? **流式分片**：不缓存整个帧，逐片发送，内存占用低
- ? **JPEG 软件压缩**：利用 frame2jpg，压缩率 ~20:1
- ? **PSRAM 优化**：摄像头缓冲使用 PSRAM，不占用 IRAM

### 实时性控制

- ?? **可控帧率**：通过 `FRAME_INTERVAL_MS` 调节帧率（5~10 FPS）
- ?? **防拥堵机制**：分片间延时防止 UDP 队列溢出
- ?? **非阻塞启停**：`start/stop` 函数立即返回，不产生延迟

## 6. 服务器端处理示例（Python）

服务器在接收图传数据时需要按照协议进行解包：

```python
import socket
import struct

def parse_v_protocol(data):
    """解析 V 协议包"""
    if data.startswith(b'[V,META,'):
        # 解析 META 包: [V,META,<seq>,<len>]
        parts = data.decode().strip('[]').split(',')
        if len(parts) == 3:
            seq_num = int(parts[1])
            jpg_len = int(parts[2])
            return ('META', seq_num, jpg_len)
    
    elif data.startswith(b'[V,CHUNK,'):
        # 解析 CHUNK 包: [V,CHUNK,<seq>,<idx>,<tot>,<size>]\n<binary>
        header_end = data.find(b'\n')
        if header_end > 0:
            header = data[:header_end].decode()
            binary = data[header_end+1:]
            
            parts = header.strip('[]').split(',')
            if len(parts) == 5:
                seq_num = int(parts[1])
                chunk_idx = int(parts[2])
                total_chunks = int(parts[3])
                chunk_size = int(parts[4])
                
                return ('CHUNK', seq_num, chunk_idx, total_chunks, chunk_size, binary)
    
    return ('UNKNOWN', data)

# 接收和处理循环示例
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5000))

frames = {}  # 缓存未完成的帧 {seq_num: {'chunks': {...}, 'total': N, 'size': B}}

while True:
    data, addr = sock.recvfrom(1024)
    result = parse_v_protocol(data)
    
    if result[0] == 'META':
        _, seq_num, jpg_len = result
        frames[seq_num] = {'chunks': {}, 'total': 0, 'size': jpg_len}
        print(f"? Frame {seq_num}: {jpg_len} bytes")
    
    elif result[0] == 'CHUNK':
        _, seq_num, chunk_idx, total_chunks, chunk_size, binary = result
        if seq_num not in frames:
            frames[seq_num] = {'chunks': {}, 'total': 0, 'size': 0}
        
        frames[seq_num]['chunks'][chunk_idx] = binary
        frames[seq_num]['total'] = total_chunks
        
        # 检查是否收完整
        if len(frames[seq_num]['chunks']) == total_chunks:
            # 组装完整 JPEG
            jpg_data = b''.join(
                frames[seq_num]['chunks'][i] 
                for i in range(total_chunks)
            )
            print(f"? Frame {seq_num} complete: {len(jpg_data)} bytes")
            
            # 保存或处理 JPEG
            # with open(f'frame_{seq_num:04d}.jpg', 'wb') as f:
            #     f.write(jpg_data)
            
            del frames[seq_num]
```

## 7. 故障排查

### 编译错误

**错误**：`undefined reference to 'video_streamer_init'`

**解决**：确保 CMakeLists.txt 中包含了 `video_streamer.c`，并执行 `idf.py clean` 后重新构建。

### 运行时问题

**症状**：图传启动后没有数据发送

**检查清单**：
1. ? `video_streamer_init()` 已在 app_main 中调用
2. ? 摄像头已成功初始化（屏幕显示画面）
3. ? UDP 模块已初始化且服务器地址正确
4. ? 调用了 `video_streamer_start()`

**诊断**：查看日志中是否有 `video_stream_task` 的 ERROR 或 WARNING

### 性能问题

**症状**：帧率低于预期或 Wi-Fi 断连

**优化建议**：
- 增加 `FRAME_INTERVAL_MS` 值（减少发送频率）
- 调整 `CHUNK_DELAY_MS` 到 5~10 ms（增加分片间延时）
- 提升视频流任务优先级（需谨慎，可能影响其他任务）

## 8. 后续扩展方向

完成基础按需图传后，可进行以下扩展：

1. **多分辨率支持**：引入 FRAMESIZE_VGA、FRAMESIZE_SVGA 等切换
2. **动态码率控制**：根据网络状况调整 JPEG 质量参数
3. **AI 视觉处理**：在视频流任务中添加目标检测、特征提取等
4. **录制功能**：将流写入 SPIFFS/SD 卡
5. **双摄像头支持**：复用模块架构支持多个摄像头
6. **RTorRTCP 协议**：升级为标准流协议

## 9. 许可和归属

本模块为 BJTU 小车项目的核心组件，遵循项目统一的开发规范。

---

**文档版本**：1.0  
**最后更新**：2026-04-29  
**作者**：项目开发组

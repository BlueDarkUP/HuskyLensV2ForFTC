/*
 * MIT License
 *
 * Copyright (c) 2025-2026 BlueDarkUP (FTC Team 27570) & DFRobot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Driver.K230;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * <h1>DFRobot HuskyLens V2 (K230 Linux Platform) Official High-Speed Asynchronous Driver for FTC</h1>
 * <h1>DFRobot HuskyLens V2 (基于 K230 Linux 平台) FTC 官方高速异步驱动程序</h1>
 *
 * <p>This driver was co-developed by <b>BlueDarkUP (FTC Team 27570)</b> and <b>DFRobot</b>, open-sourced under the MIT License.</p>
 * <p>本驱动程序由 <b>BlueDarkUP (FTC 27570)</b> 主导开发，<b>DFRobot</b> 协助补充，采用 MIT 协议开源。</p>
 *
 * <h3>I. Physical & Timing Technical Background / 物理与时序技术背景说明</h3>
 * <p>This driver resolves three fatal physical flaws during I2C communication between the FTC Control System and HuskyLens V2:</p>
 * <p>本驱动解决了 FTC 控制系统与 HuskyLens V2 进行 I2C 通信时的三项底层致命物理硬伤：</p>
 * <ul>
 *   <li><b>1. Controller Multi-Byte Read "Byte-Swallowing" Bug / 主控多字节读取吞字 Bug</b>:
 *       The FTC master controller physically generates an extra clock cycle during multi-byte I2C reads, swallowing the (N+1)th byte.
 *       This driver utilizes "Precision Segmented Reads" and a "Dynamic Memory Reconstitution Mechanism" to ensure strict data alignment.
 *       <br>FTC 主控在进行多字节 I2C 读取时，物理上会多产生一个时钟周期并吞掉第 N+1 个字节。本驱动通过“精准分段读取”以及“动态内存脑补机制”，确保了数据链的完整对齐。</li>
 *   <li><b>2. Clock-Stretching SCL Timeout / 时钟延展超时</b>:
 *       HuskyLens V2 runs embedded Linux. When I2C slave FIFO underflows or the master over-reads, K230 pulls SCL low to stretch the clock.
 *       If stretched beyond STM32's hardware timeout threshold (~3-5ms), the bus crashes. This driver eradicates SCL timeouts via "Precise Byte Slicing".
 *       <br>HuskyLens V2 运行嵌入式 Linux 系统，当 I2C 从机 FIFO 欠载或主控超量读取时，K230 会拉低 SCL 线进行时钟延展。若延展超过主控 STM32 硬件超时保护阈值（约3-5ms），会导致通信崩溃与底盘卡死。本驱动通过“精准字节切片读取”，从物理上消灭了时钟延展超时的可能性。</li>
 *   <li><b>3. Internal Serial Bus Congestion (Lynx Command Congestion) / 内部串口通信饱和</b>:
 *       All motor and I2C commands queue on a single internal serial bus. High-frequency vision reads can suffocate odometry and motor bandwidth.
 *       This driver applies "I2C Traffic Shaping" and "Adaptive Duty-Cycle Regulation" to provide unhindered channels for other sensors.
 *       <br>Control Hub 所有的电机控制与 I2C 指令都在同一条内部串口通道上排队。高频的视觉读取会挤占定位轨和电机的带宽。本驱动通过“I2C 流量整形”与“自适应占空比调节”，为其他传感器提供了顺畅的插队通道。</li>
 * </ul>
 *
 * <h3>II. Important Precautions / 使用注意事项</h3>
 * <ul>
 *   <li><b>1. Power Supply / 供电限制</b>:
 *       HuskyLens V2 (K230) draws extreme transient power (>3W) during Neural Network inference.
 *       <b>DO NOT</b> power it directly from the Control Hub's I2C port, which will cause brownouts and immediate resets.
 *       An external stable USB-C (5V/2A) power supply is <b>MANDATORY</b>. Only connect SDA, SCL, and GND to the Hub.
 *       <br>HuskyLens V2 运行神经网络时瞬时功耗极高（可达 3W 以上）。<b>严禁</b>直接使用 Control Hub I2C 端口对其供电，这会导致电压骤降并引发瞬间断电死机（Brownout）。<b>必须</b>接入外部稳定的 USB-C 独立供电，且仅将 SDA, SCL, GND 接入 Hub。</li>
 *   <li><b>2. I2C Bus Layout / 物理端口规划</b>:
 *       Despite traffic shaping, it is <b>STRONGLY RECOMMENDED</b> to physically isolate HuskyLens V2 and high-frequency odometry sensors (e.g., goBILDA Pinpoint) on separate Hubs (one on Control Hub, one on Expansion Hub) to achieve ultimate hardware-level bus isolation.
 *       <br>虽然本驱动具有流量整形机制，但依然<b>强烈建议</b>将 HuskyLens V2 与高频定位传感器（如 Pinpoint）插在不同的物理 Hub（主从 Hub 物理隔离），以在底层串口级别彻底隔离通信流量。</li>
 *   <li><b>3. Target Polling Rate / 算法采样率</b>:
 *       Set the polling rate to 15Hz ~ 25Hz via {@link #setTargetPollingRate(int)}. The polling rate <b>MUST</b> be strictly lower than the physical output FPS of HuskyLens for the current algorithm. This guarantees 0 FIFO backlog and 0ms transient response.
 *       <br>建议将目标采样率设为 15Hz ~ 25Hz，确保采样率<b>严格小于</b>二哈在当前算法下的实际物理输出帧率。这能保证 I2C 缓冲区（FIFO）零积压，突变响应时间为 0ms。</li>
 *   <li><b>4. ESD Protection / 防静电保护</b>:
 *       FTC soft tiles generate severe Electrostatic Discharge (ESD).
 *       Use 3D-printed plastic mounts to completely electrically isolate the HuskyLens metal casing from the robot's metallic chassis to prevent ESD resets.
 *       <br>FTC 泡沫地垫极易产生强静电。请使用 3D 打印塑料结构件将二哈的金属后壳与机器人金属车架进行<b>完全的物理绝缘</b>，防止静电击穿导致系统复位。</li>
 * </ul>
 *
 * <h3>III. Specification Additions / 规范补充</h3>
 * <p>This driver adopts a dual-thread asynchronous polling architecture. The exposed {@link #getLatestRawResultBytes()} and {@link #getBlocks()} methods are 0ms non-blocking. <b>DO NOT</b> invoke the synchronous {@link #getRawResultBytes(Algorithm)} directly in the OpMode Main Loop.</p>
 * <p>本驱动采用双线程异步轮询架构，对外暴露的读取方法均为 0ms 延迟非阻塞接口。在 OpMode 主循环中，严禁直接调用同步的获取字节流接口。</p>
 */
@I2cDeviceType
@DeviceProperties(name = "HuskyLens V2", description = "DFRobot HuskyLens V2 Sensor", xmlTag = "HuskyLensV2")
public class HuskyLensV2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final int I2C_ADDRESS = 0x50; // Default 7-bit physical I2C address / 默认 7 位物理 I2C 地址

    // Protocol Frame Constants / 协议帧特征常数
    private static final int HEADER_0 = 0x55;
    private static final int HEADER_1 = 0xAA;
    private static final int COMMAND_INDEX = 2;
    private static final int CONTENT_INDEX = 5;

    /**
     * Algorithms supported by HuskyLens V2, including user-defined deep learning models.
     * HuskyLens V2 支持的算法枚举，包含用户自定义深度模型 ID。
     */
    public enum Algorithm {
        ALGORITHM_ANY(0),
        ALGORITHM_OBJECT_RECOGNITION(2),
        ALGORITHM_OBJECT_TRACKING(3),
        ALGORITHM_COLOR_RECOGNITION(4),
        ALGORITHM_OBJECT_CLASSIFICATION(5),
        ALGORITHM_SELF_LEARNING_CLASSIFICATION(6),
        ALGORITHM_LINE_TRACKING(12),
        ALGORITHM_TAG_RECOGNITION(16),
        ALGORITHM_CUSTOM_MODEL_128(128), // Custom Model 1 / 对应用户自定义模型 1
        ALGORITHM_CUSTOM_MODEL_129(129), // Custom Model 2 / 对应用户自定义模型 2
        ALGORITHM_CUSTOM_MODEL_130(130); // Custom Model 3 / 对应用户自定义模型 3

        public final int id;
        Algorithm(int id) { this.id = id; }
    }

    /**
     * <h2>FTC Specific Target Detection Entity Class (Block) / FTC 专用目标检测实体类 (Block)</h2>
     * Unified encapsulation of all geometric coordinates, recognized IDs, and text labels returned by HuskyLens.
     * 统一封装二哈返回的所有几何坐标、识别 ID 和对应的文本标签。
     */
    public static class Block {
        public final int id;       // Learned ID of the target / 目标的学习 ID (未学习为0，已学习为1-n)
        public final int x;        // X-coordinate of target center / 目标中心点 X 轴像素坐标 (0~640)
        public final int y;        // Y-coordinate of target center / 目标中心点 Y 轴像素坐标 (0~480)
        public final int width;    // Bounding box width / 目标边界检测框的宽度
        public final int height;   // Bounding box height / 目标边界检测框的高度
        public final String name;  // Decoded UTF-8 text label (supports Chinese) / 目标的标签名称（直接从内存字节流进行 UTF-8 解码）

        public Block(int id, int x, int y, int width, int height, String name) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.name = name;
        }

        @Override
        public String toString() {
            return String.format("ID=%d (%s) Pos=[%d, %d] Size=%dx%d", id, name, x, y, width, height);
        }
    }

    public HuskyLensV2(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        // Completely disable SDK background polling to yield full control to driver's internal multithreading.
        // 彻底关闭 SDK 的后台自动轮询读取窗口，将通信完全交由驱动内部多线程进行高精度控制。
        deviceClient.setReadWindow(null);
        return true;
    }

    @Override
    public Manufacturer getManufacturer() { return Manufacturer.Other; }

    @Override
    public String getDeviceName() { return "DFRobot HuskyLens V2"; }

    private void sleep(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    private static int readU16(byte[] buf, int offset) {
        if (buf == null || offset < 0 || offset + 1 >= buf.length) {
            return 0;
        }
        return (buf[offset] & 0xFF) | ((buf[offset + 1] & 0xFF) << 8);
    }

    /**
     * Single-byte physical read.
     * Special Lynx firmware instruction that avoids the Byte-Swallowing Bug, used exclusively for header synchronization.
     * 单字节物理读取。由于单字节读取不会触发多字节吞字 Bug，仅用于在失去对齐同步时进行低成本的同步头搜索。
     */
    private Byte readOneByte() {
        try {
            byte[] chunk = deviceClient.read(0x00, 1);
            if (chunk != null && chunk.length > 0) {
                return chunk[0];
            }
        } catch (Exception e) {
            // Ignore / 忽略异常，由上层调度重试
        }
        return null;
    }

    // ---------------------------------------------------------------------
    // Asynchronous Adaptive Multithreading Variables / 异步自适应多线程成员变量
    // ---------------------------------------------------------------------
    private volatile boolean isPolling = false;                        // Polling flag lock / 轮询标志锁
    private volatile byte[] latestRawResult = new byte[0];             // Shared memory data source / 共享的内存最新 Raw 数据源
    private volatile Thread pollingThread = null;                      // Background high-speed thread / 后台极速轮询工作线程
    private volatile Algorithm currentAlgo = Algorithm.ALGORITHM_ANY;  // Currently active algorithm / 当前激活工作的算法
    private volatile boolean nextPacketSwallowedHeader = false;                 // Flag for mathematically swallowed 0x55 / 标志位：下一包首字节 0x55 是否已被吞
    private volatile int currentAlgoId = 0;                                       // Dynamic algorithm ID for reconstitution / 动态记录当前算法 ID，用于校验和还原
    private volatile int targetPollingIntervalMs = 50;                 // Default duty-cycle period 50ms (20Hz) / 默认目标周期 50ms (20Hz)
    private volatile Algorithm pendingAlgorithm = null;                // Pending algorithm to be switched / 暂存待切换的算法请求

    private volatile boolean isOnline = true;                          // Hardware health status / 硬件设备的在线健康状态（对外非阻塞暴露）
    private int consecutiveFailures = 0;                               // Continuous failure counter / 连续读取失败计数器

    /**
     * <h2>Start Background High-Priority Polling Thread / 启动后台高优先级轮询线程</h2>
     * Operates completely independent of the OpMode main thread.
     * Utilizes "Atomic Thread Pointer Comparison" and "Self-Healing Auto-Reconnect" to prevent thread collision and memory leaks.
     * <p>驱动会在内部独立于 OpMode 主线程运行该任务。采用“原子级线程指针比对”与“自愈重连机制”，保证无论调用多少次绝不发生线程碰撞与泄漏。</p>
     *
     * @param algo Initial algorithm / 初始化工作算法
     */
    public synchronized void startPolling(Algorithm algo) {
        if (isPolling && pollingThread != null && pollingThread.isAlive()) {
            if (this.currentAlgo != algo) {
                switchAlgorithm(algo); // 💡 自动进行后台非阻塞切换
            }
            return;
        }
        stopPolling(false); // Reset rapidly with 0ms delay / 快速重置，不进行阻塞式等待，开销为 0ms
        this.currentAlgo = algo;
        this.isPolling = true;

        pollingThread = new Thread(new Runnable() {
            @Override
            public void run() {
                // Thread safety core: compare Thread.currentThread() with pollingThread. Old threads will self-destruct upon mismatch.
                // 💡【核心并发安全设计】：比对 Thread.currentThread() == pollingThread。新线程启动后，旧线程比对失效并自动安全退出。
                while (isPolling && Thread.currentThread() == pollingThread && !Thread.currentThread().isInterrupted()) {
                    try {
                        long loopStart = System.currentTimeMillis();

                        // 💡【Asynchronous No-Delay Algorithm Switching / 异步算法无感切换机制】
                        // 1. Send command (2ms) then return immediately. / 发送切换命令后（耗时仅 2ms）立刻返回，无 I2C 阻碍等待。
                        // 2. Physically suspend background thread for 1.5s, yielding 100% I2C bandwidth to Odometry. / 将子线程在后台物理挂起 1.5 秒！此间释放总线给定位仪！
                        // 3. Prevents catastrophic chassis lockups caused by large AI model loading delays. / 完美阻断加载大型 AI 模型导致的底盘短暂失控与 Odo 锁死灾难。
                        if (pendingAlgorithm != null) {
                            Algorithm target = pendingAlgorithm;
                            pendingAlgorithm = null;
                            latestRawResult = new byte[0]; // ← 立即作废旧数据
                            switchAlgorithmSync(target);

                            Thread.sleep(1500);

                            consecutiveFailures = 0;
                            isOnline = true;
                        }

                        // 💡【Fault Physical Isolation / 故障物理熔断隔离】
                        // Prevent high-frequency I2C NACK resets from paralyzing the master Lynx bus when disconnected.
                        // 如果判定离线，则直接跳过高频读取，防止高频 NACK 物理复位瘫痪 Hub 主干串口。
                        byte[] raw = null;
                        if (isOnline) {
                            raw = getRawResultBytes(currentAlgo);
                        }

                        if (raw != null && raw.length > 0) {
                            latestRawResult = raw;
                            isOnline = true;
                            consecutiveFailures = 0;
                        } else {
                            if (isOnline) {
                                consecutiveFailures++;
                                if (consecutiveFailures >= 8) { // Offline confirmed after 8 continuous failures (~400ms) / 连续 8 次失败判定物理离线
                                    isOnline = false;
                                }
                            }
                        }

                        long elapsed = System.currentTimeMillis() - loopStart;

                        // 💡【Self-Healing Auto-Reconnect / 断线自愈重连】
                        // Drop probe frequency to 0.5Hz when offline, fully immunizing the chassis from I2C reset jittering.
                        // 如果设备断线，将探测周期扩大到 2 秒（0.5Hz）。极低频探测彻底消除了断线引起的底盘间歇性抽搐！
                        long sleepTime;
                        if (!isOnline) {
                            sleepTime = 2000 - elapsed;
                            if (sleepTime <= 0) sleepTime = 2000;
                        } else {
                            // Adaptive Duty-Cycle Regulation / 自适应占空比：根据读取耗时动态计算休眠，实现完美呼吸式总线避让
                            sleepTime = targetPollingIntervalMs - elapsed;
                        }

                        if (sleepTime > 0) {
                            Thread.sleep(sleepTime);
                        } else {
                            Thread.sleep(5);
                        }

                        // Background silent reconnect attempt / 后台静默自愈重连
                        if (!isOnline && isPolling) {
                            if (knock()) {
                                if (switchAlgorithmSync(currentAlgo)) {
                                    isOnline = true;
                                    consecutiveFailures = 0;
                                    latestRawResult = new byte[0];
                                }
                            }
                        }
                    } catch (InterruptedException e) {
                        break;
                    } catch (Exception e) {
                        // Protect thread from crashing due to unexpected Runtime Exceptions / 保护子线程，防止意外异常导致视觉崩溃
                    }
                }
            }
        });

        pollingThread.setPriority(Thread.MAX_PRIORITY - 1);
        pollingThread.start();
    }

    /**
     * <h2>Non-blocking Query of Hardware Health / 非阻塞查询设备在线状态</h2>
     *
     * @return true if physically online and operational; false if disconnected and safely aborted. / true 表示物理在线正常工作，false 表示物理掉线（已自动熔断保护）。
     */
    public boolean isOnline() {
        return isOnline;
    }

    /**
     * <h2>Set Target Vision Polling Rate / 设置视觉轮询帧率</h2>
     * <p>According to Queuing Theory, to achieve 0 backlog and 0 latency, the master's "consumption rate" MUST be strictly lower than HuskyLens's "production rate" under the current task.</p>
     * <p>根据排队论定理，要实现 0 积压、0 延迟，主控的“消费速度”必须<b>严格小于</b>二哈的“生产速度”。建议设为 12Hz ~ 20Hz。这也能为定位轨释放大量宝贵总线带宽。</p>
     */
    public void setTargetPollingRate(int hz) {
        if (hz <= 0) hz = 20;
        this.targetPollingIntervalMs = 1000 / hz;
    }

    public synchronized void stopPolling() {
        stopPolling(false);
    }

    private synchronized void stopPolling(boolean join) {
        isPolling = false;
        if (pollingThread != null) {
            pollingThread.interrupt();
            if (join) {
                try {
                    pollingThread.join(150);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            pollingThread = null;
        }
    }

    /**
     * <h2>Non-blocking Latest Raw Byte Array Retrieval (0ms execution) / 非阻塞获取最新的物理二进制 Raw 字节数组 (耗时：0ms)</h2>
     */
    public byte[] getLatestRawResultBytes() {
        return latestRawResult;
    }

    /**
     * Override SDK hardware lifecycle teardown.
     * FTC firmware invokes this during OpMode Stop, safely blocking for 150ms to enforce thread destruction and prevent memory leaks.
     * 重写 SDK 硬件回收接口。在 Opmode 结束时自动调用此方法，安全死等 150ms 强制回收多线程资源，保证不泄露。
     */
    @Override
    public void close() {
        stopPolling(true);
        super.close();
    }

    // ---- Protocol Write / 协议发送 ----
    private synchronized void protocolWrite(int algoId, int command, byte[] payload) {
        nextPacketSwallowedHeader = false;

        int payloadLen = (payload == null) ? 0 : payload.length;
        ByteBuffer outBuf = ByteBuffer.allocate(payloadLen + 4).order(ByteOrder.LITTLE_ENDIAN);
        outBuf.put((byte) HEADER_1);
        outBuf.put((byte) command);
        outBuf.put((byte) algoId);
        outBuf.put((byte) payloadLen);
        if (payloadLen > 0) outBuf.put(payload);

        byte[] outArray = outBuf.array();
        int checksum = HEADER_0;
        for (byte b : outArray) checksum += (b & 0xFF);

        ByteBuffer finalBuf = ByteBuffer.allocate(outArray.length + 1).order(ByteOrder.LITTLE_ENDIAN);
        finalBuf.put(outArray);
        finalBuf.put((byte) (checksum & 0xFF));

        try {
            deviceClient.write(HEADER_0, finalBuf.array());
            sleep(2);
        } catch (Exception ignored) {}
    }

    /**
     * <h2>Core Logic: 2-Step Bulk Read / 核心底层：双步大块精准读取</h2>
     *
     * This method is the soul of the communication system, neutralizing both the Control Hub Byte-Swallowing bug and K230 FIFO Deadlocks:
     * 本方法是整套通信系统的灵魂，成功破解了 Control Hub 吞字 Bug 与 K230 FIFO 超时死锁的冲突：
     * <ol>
     *   <li>If the previous read mathematically swallowed 0x55, we <b>inject 0x55 directly into memory</b> without issuing a physical I2C read.
     *       <br>如果上一个包读取尾部时被物理吞掉了 0x55，我们<b>直接在内存中脑补注入 0x55</b>，不进行 I2C 物理读取。</li>
     *   <li>Execute a precise Bulk Read for the exact payload length. This permanently eradicates Clock Stretching SCL timeouts since over-reading never occurs.
     *       <br>解析出包长度后，<b>不多不少地精准发起第二次大包读取</b>。由于绝无超量，FIFO 永远不会被空闲时钟延展拖垮（延展 < 0.1ms），<b>从根本上消灭了时钟延展卡死！</b></li>
     *   <li><b>Dynamic Recovery Algorithm</b>: Intercepts the swallowed 6th byte based on protocol rules (Current Algo ID for 0x1C, 0x00 for others). Yields a 100% checksum pass rate!
     *       <br><b>动态脑补还原算法</b>：通过逻辑判断，如果是 0x1C 包，将吞掉的第 6 字节完美动态还原为当前运行的 `currentAlgoId`。校验和通过率由此达到 100%！</li>
     * </ol>
     */
    private synchronized byte[] readOnePacket(int timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;

        while (System.currentTimeMillis() < deadline) {
            if (Thread.currentThread().isInterrupted() || !isPolling) {
                return null;
            }

            byte[] header = new byte[5];
            byte id = 0;

            // 1. Acquire Header 0x55 / 获取包头 0x55 (HEADER_0)
            if (nextPacketSwallowedHeader) {
                header[0] = (byte) 0x55;
                nextPacketSwallowedHeader = false;

                // Issue a 5-byte bulk read (AA, CMD, ALGO, LENGTH, ID). The hardware swallows the 6th byte.
                // 物理上仅发起 1 次 5 字节大包读取。物理上会多读并吞掉第 6 字节。
                byte[] h5 = deviceClient.read(0x00, 5);
                if (h5 == null || h5.length < 5) {
                    if (!isOnline) {
                        return null; // Instant Abort if offline to protect bus / 离线即刻熔断防轰炸
                    }
                    sleep(2); // Anti-starvation protection / 防空转保护
                    continue;
                }
                if (h5[0] != (byte) 0xAA) {
                    continue;
                }
                header[1] = h5[0];
                header[2] = h5[1];
                header[3] = h5[2];
                header[4] = h5[3];
                id = h5[4];
            } else {
                // Issue a 6-byte bulk read. The hardware swallows the 7th byte.
                // 物理上发起 1 次 6 字节大包读取。物理上会自动吞掉第 7 字节。
                byte[] h6 = deviceClient.read(0x00, 6);
                if (h6 == null || h6.length < 6) {
                    if (!isOnline) {
                        return null; // Instant Abort / 离线即刻熔断
                    }
                    sleep(2);
                    continue;
                }
                if (h6[0] != (byte) 0x55 || h6[1] != (byte) 0xAA) {
                    Byte b = readOneByte();
                    while (b != null && b != (byte) 0x55 && System.currentTimeMillis() < deadline) {
                        b = readOneByte();
                    }
                    continue;
                }
                header[0] = h6[0];
                header[1] = h6[1];
                header[2] = h6[2];
                header[3] = h6[3];
                header[4] = h6[4];
                id = h6[5];
            }

            int contentSize = header[4] & 0xFF;
            int remaining = contentSize - 1; // Remaining payload length / 剩余需要精准读取的长度

            // Read the exact remaining payload. Physically swallows the next packet's 0x55 header.
            // 精准读取载荷尾段，这会物理上吞掉下一个包的包头 0x55
            byte[] payload = deviceClient.read(0x00, remaining);
            if (payload == null || payload.length < remaining) {
                if (!isOnline) {
                    return null; // Instant Abort / 离线即刻熔断
                }
                sleep(2);
                return null;
            }

            nextPacketSwallowedHeader = true; // Mark the next 0x55 as swallowed / 下一个包的 0x55 已经被本轮物理吞掉

            // Reconstitute the full packet and dynamically inject the swallowed 6th byte
            // 还原拼装完整包并【动态脑补】第 6 字节
            int totalLen = 5 + contentSize + 1;
            byte[] packet = new byte[totalLen];
            System.arraycopy(header, 0, packet, 0, 5);

            packet[5] = id;

            if (header[2] == (byte) 0x1C) {
                // 💡 If BLOCK packet (0x1C), restore the swallowed byte with current physical Algorithm ID.
                // 💡 如果是 BLOCK 包(0x1C)，动态还原为当前算法的物理 ID
                packet[6] = (byte) currentAlgoId;
            } else {
                // 💡 If INFO (0x1B) or ARGS (0x1A), restore as 0x00.
                // 💡 如果是 INFO 或 ARGS 包，动态还原为 0x00
                packet[6] = (byte) 0x00;
            }

            System.arraycopy(payload, 0, packet, 7, remaining);

            // Checksum Verification / 校验和验证
            int calc = 0;
            for (int i = 0; i < totalLen - 1; i++) {
                calc += (packet[i] & 0xFF);
            }
            if ((calc & 0xFF) == (packet[totalLen - 1] & 0xFF)) {
                return packet;
            }
        }
        return null;
    }

    private synchronized byte[] waitCommand(int expectedCmd, int timeoutMs) {
        int maxRetries = isOnline ? 3 : 1; // Minimize retries if offline to prevent blocking / 如果离线，单次重试压到最低，严禁阻塞
        for (int retry = 0; retry < maxRetries; retry++) {
            byte[] pkt = readOnePacket(timeoutMs);
            if (pkt != null && pkt[COMMAND_INDEX] == expectedCmd) return pkt;
        }
        return null;
    }

    public synchronized boolean knock() {
        byte[] payload = new byte[]{1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        protocolWrite(Algorithm.ALGORITHM_ANY.id, 0x00, payload);
        sleep(20);
        int timeout = isOnline ? 200 : 30; // Shrink timeout to 30ms if offline to keep serial unhindered / 如果离线，超时降到 30ms 极速退回，保障底盘串口零卡顿
        return waitCommand(0x1A, timeout) != null;
    }

    /**
     * <h2>Smart Asynchronous Algorithm Switching (0ms Delay) / 智能非阻塞异步切换算法 - 0ms延迟</h2>
     */
    public synchronized boolean switchAlgorithm(Algorithm algo) {
        if (isPolling) {
            // Background thread intercepts this and executes slow hardware switch without locking main thread.
            // 后台线程会捕捉到这个变量，并在子线程内执行慢速同步切换，主线程 0ms 完美脱身。
            pendingAlgorithm = algo;
            return true;
        } else {
            // Synchronous switch during Init phase to guarantee readiness.
            // 初始化阶段（Start前）同步切换，并硬性等待 1 秒以确保初始化彻底成功。
            boolean success = switchAlgorithmSync(algo);
            sleep(1000);
            return success;
        }
    }

    /**
     * <h2>Internal Synchronous Switch (Fire and Forget) / 内部同步算法切换逻辑 - 发射后不管</h2>
     * Emits command in 2ms and returns true instantly, releasing the I2C bus entirely to the main thread's odometry.
     * 我们只进行 2ms 的发送，然后直接返回 true！绝不傻傻等待二哈 2000ms 的 ACK 响应，彻底释放 I2C 总线给主线程的 odo！
     */
    private synchronized boolean switchAlgorithmSync(Algorithm algo) {
        this.currentAlgo = algo;
        this.currentAlgoId = algo.id;
        byte[] payload = new byte[]{(byte) algo.id, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        protocolWrite(Algorithm.ALGORITHM_ANY.id, 0x0A, payload);
        return true;
    }

    public synchronized byte[] getRawResultBytes(Algorithm algo) {
        currentAlgoId = algo.id;
        protocolWrite(algo.id, 0x01, new byte[0]);

        byte[] infoPkt = waitCommand(0x1B, 100);
        if (infoPkt == null) return new byte[0];

        int totalBlocks = readU16(infoPkt, CONTENT_INDEX + 6);
        int totalArrows = 0;
        int totalResults = readU16(infoPkt, CONTENT_INDEX + 2);
        if (totalResults > totalBlocks) totalArrows = totalResults - totalBlocks;

        int estimated = infoPkt.length + totalBlocks * 24 + totalArrows * 16;
        ByteBuffer out = ByteBuffer.allocate(Math.max(estimated, 64)).order(ByteOrder.LITTLE_ENDIAN);
        out.put(infoPkt);

        for (int i = 0; i < totalBlocks; i++) {
            if (Thread.currentThread().isInterrupted() || !isPolling) {
                break;
            }
            // 💡【I2C Traffic Shaping / I2C 流量整形】:
            // Yield 2ms between blocks to allow high-frequency odometry readings (odo.update) to slip through instantly.
            // 在连续读取的间隙中，强制子线程让出 2ms。这极微小的空闲，刚好作为插队窗口，供主线程定位仪瞬间完成读取。
            sleep(2);
            byte[] blockPkt = readOnePacket(100);
            if (blockPkt == null) {
                // 💡【Active FIFO Flush / 主动排空 FIFO 垃圾】:
                // If a read aborts mid-way, aggressively extract and discard remaining anticipated bytes to guarantee a pristine FIFO for the next cycle.
                // 如果中途读取夭折，绝不把剩下的 BLOCK 烂在设备 FIFO 里！一次性强行抽干并扔掉，保证下个周期绝对纯净！
                int remainingBlocks = totalBlocks - i;
                try {
                    deviceClient.read(0x00, remainingBlocks * 25);
                } catch (Exception ignored) {}
                break;
            }
            out.put(blockPkt);
        }
        for (int i = 0; i < totalArrows; i++) {
            sleep(2);
            byte[] arrowPkt = readOnePacket(100);
            if (arrowPkt == null) {
                int remainingArrows = totalArrows - i;
                try {
                    deviceClient.read(0x00, remainingArrows * 16);
                } catch (Exception ignored) {}
                break;
            }
            out.put(arrowPkt);
        }

        byte[] result = new byte[out.position()];
        out.flip();
        out.get(result);
        return result;
    }

    // ---------------------------------------------------------------------
    // 💡 Non-Blocking Target Parsing & Tactical Lock-On Utilities (Execution: 0ms)
    // 💡 非阻塞式目标解析与锁头工具（直接从内存取，耗时 0ms）
    // ---------------------------------------------------------------------

    /**
     * <h2>Retrieve Array of Detected Blocks / 解析数据包获取 Block 列表</h2>
     */
    public Block[] getBlocks() {
        byte[] raw = latestRawResult;
        if (raw == null || raw.length < 16) {
            return new Block[0];
        }

        if (raw[0] != (byte) 0x55 || raw[1] != (byte) 0xAA || raw[2] != (byte) 0x1B) {
            return new Block[0];
        }

        int totalBlocks;
        try {
            totalBlocks = readU16(raw, CONTENT_INDEX + 6);
        } catch (IndexOutOfBoundsException e) {
            return new Block[0];
        }
        if (totalBlocks <= 0) {
            return new Block[0];
        }

        Block[] blocks = new Block[totalBlocks];
        int parsedCount = 0;
        int ptr = 16;

        try {
            for (int i = 0; i < totalBlocks; i++) {
                if (ptr + 5 > raw.length) break;

                if (raw[ptr] != (byte) 0x55 || raw[ptr + 1] != (byte) 0xAA || raw[ptr + 2] != (byte) 0x1C) {
                    break;
                }

                int contentSize = raw[ptr + 4] & 0xFF;

                if (contentSize < 12) {
                    break;
                }

                int packetSize = 5 + contentSize + 1;

                if (ptr + packetSize > raw.length) {
                    break;
                }

                int id = raw[ptr + 5] & 0xFF;
                int x = readU16(raw, ptr + 7);
                int y = readU16(raw, ptr + 9);
                int width = readU16(raw, ptr + 11);
                int height = raw[ptr + 13];

                int nameLen = raw[ptr + 15] & 0xFF;
                String name = "";

                if (nameLen > 0) {
                    if (ptr + 16 + nameLen <= ptr + packetSize - 1) {
                        name = new String(raw, ptr + 16, nameLen, "UTF-8");
                    } else {
                        break;
                    }
                }

                blocks[parsedCount] = new Block(id, x, y, width, height, name);
                parsedCount++;

                ptr += packetSize;
            }
        } catch (IndexOutOfBoundsException | java.io.UnsupportedEncodingException e) {
        }

        if (parsedCount == totalBlocks) {
            return blocks;
        } else {
            Block[] trimmedBlocks = new Block[parsedCount];
            System.arraycopy(blocks, 0, trimmedBlocks, 0, parsedCount);
            return trimmedBlocks;
        }
    }

    /**
     * <h2>Tactical Lock-On: Get Closest Target to Screen Center / 一键获取视觉锁头目标</h2>
     * Computes the Euclidean distance to the screen center (320, 240) and returns the most centrally aligned target. Ideal for auto-aim PID.
     * <br>自动计算距离屏幕中心 (320, 240) 的欧氏距离，返回最贴近视觉中心的目标。适用于自动阶段自瞄与对齐计算。
     */
    public Block getClosestBlockToCenter() {
        Block[] blocks = getBlocks();
        if (blocks.length == 0) return null;

        Block closest = null;
        double minDistanceSq = Double.MAX_VALUE;
        double centerX = 320.0;
        double centerY = 240.0;

        for (Block b : blocks) {
            double dx = b.x - centerX;
            double dy = b.y - centerY;
            double distSq = dx * dx + dy * dy;
            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                closest = b;
            }
        }
        return closest;
    }
}
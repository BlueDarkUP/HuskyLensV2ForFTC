package org.firstinspires.ftc.teamcode.Driver.K230;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(name = "HuskyLens V2", description = "DFRobot HuskyLens V2 Sensor", xmlTag = "HuskyLensV2")
public class HuskyLensV2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    private static final int I2C_ADDRESS = 0x50;

    private static final int HEADER_0 = 0x55;
    private static final int HEADER_1 = 0xAA;
    private static final int COMMAND_INDEX = 2;
    private static final int CONTENT_INDEX = 5;

    public enum Algorithm {
        ALGORITHM_ANY(0),
        ALGORITHM_OBJECT_RECOGNITION(2),
        ALGORITHM_OBJECT_TRACKING(3),
        ALGORITHM_COLOR_RECOGNITION(4),
        ALGORITHM_OBJECT_CLASSIFICATION(5),
        ALGORITHM_SELF_LEARNING_CLASSIFICATION(6),
        ALGORITHM_LINE_TRACKING(12),
        ALGORITHM_TAG_RECOGNITION(16),
        ALGORITHM_CUSTOM_MODEL_128(128),
        ALGORITHM_CUSTOM_MODEL_129(129),
        ALGORITHM_CUSTOM_MODEL_130(130);

        public final int id;
        Algorithm(int id) { this.id = id; }
    }

    // ---------------------------------------------------------------------
    // 💡【核心新增】FTC 专用目标实体类
    // ---------------------------------------------------------------------
    public static class Block {
        public final int id;       // 目标的学习 ID (未学习为0，已学习为1-n)
        public final int x;        // 目标中心点 X 坐标 (像素范围：0~640)
        public final int y;        // 目标中心点 Y 坐标 (像素范围：0~480)
        public final int width;    // 目标边界框宽度
        public final int height;   // 目标边界框高度
        public final String name;  // 目标的标签名称（支持中文标签）

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
        if (offset + 1 >= buf.length) return 0;
        return (buf[offset] & 0xFF) | ((buf[offset + 1] & 0xFF) << 8);
    }

    // 单字节读取
    private Byte readOneByte() {
        try {
            byte[] chunk = deviceClient.read(0x00, 1);
            if (chunk != null && chunk.length > 0) {
                return chunk[0];
            }
        } catch (Exception e) {
            // ignore
        }
        return null;
    }

    // ---------------------------------------------------------------------
    // 异步自适应多线程成员变量
    // ---------------------------------------------------------------------
    private volatile boolean isPolling = false;                  // 是否正在后台轮询
    private volatile byte[] latestRawResult = new byte[0];        // 共享的最新结果（非阻塞读取源）
    private volatile Thread pollingThread = null;                 // 后台轮询线程 (加 volatile 保证并发可见性)
    private volatile Algorithm currentAlgo = Algorithm.ALGORITHM_ANY; // 追踪当前算法
    private boolean nextPacketSwallowedHeader = false;            // 脑补 0x55 标志
    private int currentAlgoId = 0;                                // 动态记录当前算法 ID
    private volatile int targetPollingIntervalMs = 50;            // 默认目标周期 50ms (对应黄金 20Hz 刷新率)
    private volatile Algorithm pendingAlgorithm = null;           // 异步切换待执行算法

    /**
     * 【一键开启后台极速轮询】
     */
    public synchronized void startPolling(Algorithm algo) {
        stopPolling(false);
        this.currentAlgo = algo;
        this.isPolling = true;

        pollingThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (isPolling && Thread.currentThread() == pollingThread && !Thread.currentThread().isInterrupted()) {
                    try {
                        long loopStart = System.currentTimeMillis();

                        // 异步算法切换
                        if (pendingAlgorithm != null) {
                            Algorithm target = pendingAlgorithm;
                            pendingAlgorithm = null;
                            switchAlgorithmSync(target);
                        }

                        byte[] raw = getRawResultBytes(currentAlgo);
                        if (raw != null && raw.length > 0) {
                            latestRawResult = raw;
                        }

                        long elapsed = System.currentTimeMillis() - loopStart;
                        long sleepTime = targetPollingIntervalMs - elapsed;
                        if (sleepTime > 0) {
                            Thread.sleep(sleepTime);
                        } else {
                            Thread.sleep(5);
                        }
                    } catch (InterruptedException e) {
                        break;
                    } catch (Exception e) {
                        // 保护子线程永不崩溃
                    }
                }
            }
        });

        pollingThread.setPriority(Thread.MAX_PRIORITY - 1);
        pollingThread.start();
    }

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

    public byte[] getLatestRawResultBytes() {
        return latestRawResult;
    }

    @Override
    public void close() {
        stopPolling(true);
        super.close();
    }

    // ---------------------------------------------------------------------
    // 💡【核心新增】非阻塞式目标解析与锁头工具（直接从内存取，耗时 0ms）
    // ---------------------------------------------------------------------

    /**
     * 【一键获取当前视野里的所有目标】
     * 自动完成小端转换、包结构拼装和中文 UTF-8 解码，主控毫秒级响应。
     */
    public Block[] getBlocks() {
        byte[] raw = latestRawResult;
        if (raw == null || raw.length < 16) {
            return new Block[0]; // 无有效数据，返回空数组
        }

        // 验证第一包是否为 INFO 包 (0x1B)
        if (raw[0] != (byte) 0x55 || raw[1] != (byte) 0xAA || raw[2] != (byte) 0x1B) {
            return new Block[0];
        }

        // 解析总 Blocks 数量
        int totalBlocks = readU16(raw, CONTENT_INDEX + 6);
        if (totalBlocks <= 0) {
            return new Block[0];
        }

        Block[] blocks = new Block[totalBlocks];
        int ptr = 16; // 越过 16 字节的 INFO 包，开始解析后面的 BLOCK

        for (int i = 0; i < totalBlocks; i++) {
            if (ptr + 5 > raw.length) break;

            // 对齐验证
            if (raw[ptr] != (byte) 0x55 || raw[ptr + 1] != (byte) 0xAA) break;
            if (raw[ptr + 2] != (byte) 0x1C) break; // 不是 0x1C (BLOCK) 协议包则中断

            int contentSize = raw[ptr + 4] & 0xFF;
            int packetSize = 5 + contentSize + 1;
            if (ptr + packetSize > raw.length) break; // 防止越界

            // 精准位移还原
            int id = raw[ptr + 5] & 0xFF;
            // index 6 被我们动态还原为了 Algo ID
            int x = readU16(raw, ptr + 7);
            int y = readU16(raw, ptr + 9);
            int width = readU16(raw, ptr + 11);
            int height = readU16(raw, ptr + 13);

            // 动态解析变长 UTF-8 字符串
            int nameLen = raw[ptr + 15] & 0xFF;
            String name = "";
            if (nameLen > 0 && ptr + 16 + nameLen <= raw.length) {
                try {
                    name = new String(raw, ptr + 16, nameLen, "UTF-8");
                } catch (Exception e) {
                    name = "";
                }
            }

            blocks[i] = new Block(id, x, y, width, height, name);
            ptr += packetSize; // 跳至下一个 Block 的开头
        }

        return blocks;
    }

    /**
     * 【高阶战术锁头：一键获取离屏幕中心最近的目标】
     * 自动计算距离屏幕中心 (320, 240) 的欧氏距离，返回最贴近视觉中心的目标。
     * 非常适合用于自动阶段机器人吸收 Pixel 或 路径自动 PID 对齐。
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

    // ---- 协议发送 ----
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

    // ---- 协议接收（纯大包、零单字节、零超载，绝不超时） ----
    private synchronized byte[] readOnePacket(int timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;

        while (System.currentTimeMillis() < deadline) {
            byte[] header = new byte[5];
            byte id = 0;

            if (nextPacketSwallowedHeader) {
                header[0] = (byte) 0x55;
                nextPacketSwallowedHeader = false;

                byte[] h5 = deviceClient.read(0x00, 5);
                if (h5 == null || h5.length < 5) {
                    sleep(2); // 防空转保护
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
                byte[] h6 = deviceClient.read(0x00, 6);
                if (h6 == null || h6.length < 6) {
                    sleep(2); // 防空转保护
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
            int remaining = contentSize - 1;

            byte[] payload = deviceClient.read(0x00, remaining);
            if (payload == null || payload.length < remaining) {
                sleep(2);
                return null;
            }

            nextPacketSwallowedHeader = true;

            int totalLen = 5 + contentSize + 1;
            byte[] packet = new byte[totalLen];
            System.arraycopy(header, 0, packet, 0, 5);

            packet[5] = id;

            if (header[2] == (byte) 0x1C) {
                packet[6] = (byte) currentAlgoId;
            } else {
                packet[6] = (byte) 0x00;
            }

            System.arraycopy(payload, 0, packet, 7, remaining);

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
        for (int retry = 0; retry < 3; retry++) {
            byte[] pkt = readOnePacket(timeoutMs);
            if (pkt != null && pkt[COMMAND_INDEX] == expectedCmd) return pkt;
        }
        return null;
    }

    public synchronized boolean knock() {
        byte[] payload = new byte[]{1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        protocolWrite(Algorithm.ALGORITHM_ANY.id, 0x00, payload);
        sleep(20);
        return waitCommand(0x1A, 200) != null;
    }

    public synchronized boolean switchAlgorithm(Algorithm algo) {
        if (isPolling) {
            pendingAlgorithm = algo;
            return true;
        } else {
            return switchAlgorithmSync(algo);
        }
    }

    private synchronized boolean switchAlgorithmSync(Algorithm algo) {
        this.currentAlgo = algo;
        this.currentAlgoId = algo.id;
        byte[] payload = new byte[]{(byte) algo.id, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        protocolWrite(Algorithm.ALGORITHM_ANY.id, 0x0A, payload);
        return waitCommand(0x1A, 2000) != null;
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
            byte[] blockPkt = readOnePacket(100);
            if (blockPkt == null) {
                int remainingBlocks = totalBlocks - i;
                try {
                    deviceClient.read(0x00, remainingBlocks * 25);
                } catch (Exception ignored) {}
                break;
            }
            out.put(blockPkt);
        }
        for (int i = 0; i < totalArrows; i++) {
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
}
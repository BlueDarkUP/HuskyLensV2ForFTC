package org.firstinspires.ftc.teamcode.Driver.K230;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Driver.K230.HuskyLensV2;

@TeleOp(name = "HuskyLens V2 终极完整功能测试", group = "Sensor")
public class HuskyLensV2TeleOp extends LinearOpMode {

    private HuskyLensV2 huskyLens;
    private HuskyLensV2.Algorithm currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_OBJECT_RECOGNITION;
    private int pollingRateHz = 20; // 默认 20Hz (兼顾 0延迟与K230极低负载)
    private boolean isPollingActive = true;

    // 用来防止按键抖动（Debounce）的变量
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {
        telemetry.addLine("=== HuskyLens V2 极速异步多线程测试程序 ===");
        telemetry.addLine("正在初始化 I2C 通信...");
        telemetry.update();

        // 1. 获取设备并执行同步握手 (按下 START 之前，同步阻塞是可以接受的)
        huskyLens = hardwareMap.get(HuskyLensV2.class, "huskylens");

        boolean alive = huskyLens.knock();
        if (alive) {
            telemetry.addData("Knock 状态", "成功在线！(OK)");
        } else {
            telemetry.addData("Knock 状态", "离线 (FAIL) - 请检查连接与 USB-C 外部供电");
        }
        telemetry.update();

        // 2. 设置初始算法并启动内部异步轮询
        huskyLens.switchAlgorithm(currentAlgo);
        huskyLens.setTargetPollingRate(pollingRateHz);
        huskyLens.startPolling(currentAlgo);

        telemetry.addLine("\n==================================");
        telemetry.addLine("🎮 交互控制说明：");
        telemetry.addLine("按【A】：异步切为 目标检测 (Object Recognition)");
        telemetry.addLine("按【B】：异步切为 标签识别 (Tag/AprilTag)");
        telemetry.addLine("按【X】：异步切为 颜色识别 (Color Recognition)");
        telemetry.addLine("按【Y】：异步切为 自定义模型 128 (Custom Model 128)");
        telemetry.addLine("按【Dpad Up】：提升轮询帧率上限 (Max 40Hz)");
        telemetry.addLine("按【Dpad Down】：降低轮询帧率下限 (Min 5Hz)");
        telemetry.addLine("按【Left Bumper】：开启 / 停止 异步轮询子线程");
        telemetry.addLine("==================================");
        telemetry.addLine("初始化完成，按下 START 开启极速测试！");
        telemetry.update();

        waitForStart();

        long lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            // ---- 计算主循环耗时与频率 ----
            long currentTime = System.currentTimeMillis();
            long loopTime = currentTime - lastTime;
            lastTime = currentTime;

            // ---- 按键事件处理（防抖动边缘触发） ----
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentLeftBumper = gamepad1.left_bumper;

            // A 键：无卡顿异步切换到目标检测 (命令下发耗时 0ms)
            if (currentA && !lastA) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_OBJECT_RECOGNITION;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            // B 键：异步切换到标签识别
            if (currentB && !lastB) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_TAG_RECOGNITION;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            // X 键：异步切换到颜色识别
            if (currentX && !lastX) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_COLOR_RECOGNITION;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            // Y 键：异步切换到自定义算法 128
            if (currentY && !lastY) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_CUSTOM_MODEL_128;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            // Dpad Up: 提升轮询帧率
            if (currentDpadUp && !lastDpadUp) {
                if (pollingRateHz < 40) {
                    pollingRateHz += 5;
                    huskyLens.setTargetPollingRate(pollingRateHz);
                }
            }
            // Dpad Down: 降低轮询帧率
            if (currentDpadDown && !lastDpadDown) {
                if (pollingRateHz > 5) {
                    pollingRateHz -= 5;
                    huskyLens.setTargetPollingRate(pollingRateHz);
                }
            }
            // Left Bumper: 启停后台轮询
            if (currentLeftBumper && !lastLeftBumper) {
                isPollingActive = !isPollingActive;
                if (isPollingActive) {
                    huskyLens.startPolling(currentAlgo);
                } else {
                    huskyLens.stopPolling();
                }
            }

            // 更新按键缓存
            lastA = currentA;
            lastB = currentB;
            lastX = currentX;
            lastY = currentY;
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastLeftBumper = currentLeftBumper;

            // ---- 核心非阻塞数据获取 (内存读取，耗时均为 0ms!) ----
            byte[] raw = huskyLens.getLatestRawResultBytes();
            HuskyLensV2.Block[] targets = huskyLens.getBlocks();
            HuskyLensV2.Block closest = huskyLens.getClosestBlockToCenter();

            // ---- Telemetry 完整数据渲染与显示 ----
            telemetry.addLine("=== ⚙️ 硬件与驱动状态监控 ===");
            telemetry.addData("主循环 Looptime", "%d ms (主频: %.1f Hz) <-- 证明绝对无阻碍！", loopTime, 1000.0 / Math.max(1, loopTime));
            telemetry.addData("当前激活算法", currentAlgo.name() + " (ID: " + currentAlgo.id + ")");
            telemetry.addData("轮询线程状态", isPollingActive ? "正在运行 (RUNNING)" : "已停止 (STOPPED)");
            telemetry.addData("设定轮询帧率", "%d Hz (周期间隔: %d ms)", pollingRateHz, 1000 / pollingRateHz);
            telemetry.addData("读回 Raw 字节长度", raw.length + " 字节");

            telemetry.addLine("\n=== 🎯 二哈物理检测包数据 ===");
            telemetry.addData("视野内识别目标总数", targets.length);

            if (targets.length == 0) {
                telemetry.addLine("  [未检测到任何目标]");
            } else {
                // 1. 循环打印视野内所有目标的物理信息
                for (int i = 0; i < targets.length; i++) {
                    HuskyLensV2.Block t = targets[i];
                    telemetry.addLine(String.format("  [%d] ID:%d | 标签:\"%s\" | 中心:[%d, %d] | 尺寸:%dx%d",
                            i + 1, t.id, t.name, t.x, t.y, t.width, t.height));
                }

                // 2. 打印黄金锁头目标（最靠近中心的目标）的所有偏差物理数据
                if (closest != null) {
                    telemetry.addLine("\n=== 🎯 FTC 高阶锁头目标 ===");
                    telemetry.addData("锁头目标 ID", closest.id);
                    telemetry.addData("标签名称", closest.name);
                    telemetry.addData("屏幕中心偏差 X (自瞄误差值)", closest.x - 320);
                    telemetry.addData("屏幕中心偏差 Y (自瞄误差值)", closest.y - 240);
                    telemetry.addData("目标物理面积", (closest.width * closest.height) + " 像素平方");
                }
            }

            telemetry.update();
        }

        // 💡 提示：即使不写，close() 也会自动回收物理线程，此处显式调用再次双保险
        huskyLens.stopPolling();
    }
}
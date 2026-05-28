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

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * <h1>Ultimate Asynchronous Vision & Odometry TeleOp / 终极异步视觉与定位 TeleOp 示例</h1>
 *
 * <p>This OpMode demonstrates the seamless integration of a Mecanum drivetrain, goBILDA Pinpoint Odometry, and the ultra-low latency HuskyLens V2 driver.</p>
 * <p>本程序展示了麦克纳姆轮底盘、goBILDA Pinpoint 定位仪，以及极低延迟 HuskyLens V2 驱动的完美无缝融合。</p>
 *
 * <h3>Important Engineering Precautions / 核心工程注意事项:</h3>
 * <ul>
 *   <li><b>Power Supply / 独立供电</b>: HuskyLens V2 MUST be powered by an external USB-C source. Never rely solely on the I2C 3.3V pin. / HuskyLens V2 必须接入外部 USB-C 供电，绝不能仅依靠 I2C 的 3.3V 供电。</li>
 *   <li><b>Bus Isolation / 总线隔离</b>: Keep HuskyLens V2 and Pinpoint on separate I2C buses (or separate Hubs) to maximize bandwidth. / 务必将 HuskyLens V2 和 Pinpoint 挂载在不同的 I2C 总线或物理 Hub 上，以最大化串行带宽。</li>
 *   <li><b>0ms Blocking / 零阻塞架构</b>: The main loop remains strictly under 5ms, ensuring PID controllers and drivetrain reactivity are never compromised by vision processing. / 主循环耗时被严格限制在 5ms 以内，确保底盘响应和 PID 控制绝不被视觉处理拖累。</li>
 * </ul>
 *
 * @author BlueDarkUP (FTC 27570)
 * @author DFRobot
 */
@TeleOp(name = "Sensor: HuskyLens", group = "Sensor")
public class SensorHuskyLensV2 extends LinearOpMode {

    // Hardware Components / 硬件组件
    private DcMotor lf, rf, lb, rb;
    private GoBildaPinpointDriver odo;
    private HuskyLensV2 huskyLens;

    // Vision States / 视觉系统状态变量
    private HuskyLensV2.Algorithm currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_OBJECT_RECOGNITION;
    private int pollingRateHz = 20; // Optimal 20Hz ensures 0 FIFO backlog / 黄金 20Hz 确保 FIFO 零积压
    private boolean isPollingActive = true;

    // Gamepad Debounce Cache / 手柄按键防抖缓存 (Edge-Triggering)
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // =====================================================================
        // 1. Drivetrain Initialization / 麦轮底盘初始化
        // =====================================================================
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // Reverse left motors to ensure positive power drives forward
        // 反转左侧电机，确保给予正功率时机器人向前行驶
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // =====================================================================
        // 2. goBILDA Pinpoint Initialization / 定位计算仪初始化
        // =====================================================================
        telemetry.addLine("Connecting to Pinpoint Odometry... / 正在连接定位仪...");
        telemetry.update();
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        /*
         * Set physical offsets of the dead wheels relative to the robot's center.
         * 设置死轮相对于机器人中心的物理偏移量 (单位：毫米)。请根据实际底盘尺寸进行测量和修改。
         * Params: [X Offset (Forward/Back), Y Offset (Left/Right)]
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU(); // Reset origin / 重置原点

        // =====================================================================
        // 3. HuskyLens V2 Initialization / 视觉传感器初始化
        // =====================================================================
        telemetry.addLine("Connecting to HuskyLens V2... / 正在连接 HuskyLens V2...");
        telemetry.update();
        huskyLens = hardwareMap.get(HuskyLensV2.class, "huskylens");

        // Synchronous knock during initialization ensures physical connectivity
        // 在 Init 阶段进行同步握手，确保物理连线 100% 正常
        boolean isConnected = huskyLens.knock();
        if (isConnected) {
            telemetry.addData("HuskyLens Status", "Online (OK) / 成功连接！");
        } else {
            telemetry.addData("HuskyLens Status", "Offline (FAIL) / 离线！请检查外部供电与接线！");
        }
        telemetry.update();

        // Switch algorithm and start background multi-threading polling
        // 切换到初始算法并启动内部零延迟多线程异步轮询
        huskyLens.switchAlgorithm(currentAlgo);
        huskyLens.setTargetPollingRate(pollingRateHz);
        huskyLens.startPolling(currentAlgo);

        telemetry.addLine("\n==================================");
        telemetry.addLine("🎮 Controls / 交互操作说明：");
        telemetry.addLine("Left Stick / 左摇杆: Translate (平移)");
        telemetry.addLine("Right Stick / 右摇杆: Rotate (自转)");
        telemetry.addLine("[RB]: Reset Odometry Origin / 重置定位原点");
        telemetry.addLine("[A]: Object Recognition / 目标检测");
        telemetry.addLine("[B]: Tag Recognition / 标签识别");
        telemetry.addLine("[X]: Color Recognition / 颜色识别");
        telemetry.addLine("[Y]: Custom Model 128 / 自定义模型128");
        telemetry.addLine("[Dpad Up]: Increase Hz / 增加轮询帧率");
        telemetry.addLine("[Dpad Down]: Decrease Hz / 降低轮询帧率");
        telemetry.addLine("[LB]: Toggle Vision Thread / 启停视觉子线程");
        telemetry.addLine("==================================");
        telemetry.addLine("Ready. Press START. / 初始化完成，请按下 START！");
        telemetry.update();

        waitForStart();
        long lastTime = System.currentTimeMillis();

        // =====================================================================
        // 🚀 MAIN LOOP / 比赛主循环
        // =====================================================================
        while (opModeIsActive()) {

            // Loop time monitoring (Proofs the 0ms blocking architecture)
            // 循环耗时监控（用于证明零阻塞架构的有效性）
            long currentTime = System.currentTimeMillis();
            long loopTime = currentTime - lastTime;
            lastTime = currentTime;

            // -------------------------------------------------------------
            // A. Mecanum Kinematics / 麦轮底盘运动学计算
            // -------------------------------------------------------------
            double driveY = -gamepad1.left_stick_y;  // Forward is positive / 摇杆向上为负，取反使向前为正
            double driveX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing friction / 乘1.1抵消横移物理摩擦阻力
            double turnRX = gamepad1.right_stick_x;  // Rotation / 左右自转

            // Normalize vector to ensure motor power does not exceed 1.0
            // 向量归一化，防止多方向合力导致电机功率超过 1.0 而产生比例失真
            double denominator = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(turnRX), 1.0);
            lf.setPower((driveY + driveX + turnRX) / denominator);
            lb.setPower((driveY - driveX + turnRX) / denominator);
            rf.setPower((driveY - driveX - turnRX) / denominator);
            rb.setPower((driveY + driveX - turnRX) / denominator);

            // -------------------------------------------------------------
            // B. Odometry Update / 定位计算仪读取
            // -------------------------------------------------------------
            odo.update(); // Trigger internal high-speed bus read / 触发内部高速总线读取
            Pose2D pose2D = odo.getPosition();
            double odoX = pose2D.getX(DistanceUnit.INCH);
            double odoY = pose2D.getY(DistanceUnit.INCH);
            double odoHeading = pose2D.getHeading(AngleUnit.DEGREES);

            // -------------------------------------------------------------
            // C. Gamepad Edge-Trigger Logic / 手柄边缘触发逻辑 (防抖动)
            // -------------------------------------------------------------
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;
            boolean currentX = gamepad1.x;
            boolean currentY = gamepad1.y;
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;
            boolean currentLeftBumper = gamepad1.left_bumper;
            boolean currentRightBumper = gamepad1.right_bumper;

            // Asynchronous Fire-and-Forget Algorithm Switch (0ms overhead in main thread)
            // 无卡顿异步切换算法 (发射后不管，主线程耗时 0ms)
            if (currentA && !lastA) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_OBJECT_RECOGNITION;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            if (currentB && !lastB) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_TAG_RECOGNITION;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            if (currentX && !lastX) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_COLOR_RECOGNITION;
                huskyLens.switchAlgorithm(currentAlgo);
            }
            if (currentY && !lastY) {
                currentAlgo = HuskyLensV2.Algorithm.ALGORITHM_CUSTOM_MODEL_128;
                huskyLens.switchAlgorithm(currentAlgo);
            }

            // Adjust adaptive duty-cycle polling rate / 调节自适应占空比轮询帧率
            if (currentDpadUp && !lastDpadUp && pollingRateHz < 40) {
                pollingRateHz += 5;
                huskyLens.setTargetPollingRate(pollingRateHz);
            }
            if (currentDpadDown && !lastDpadDown && pollingRateHz > 5) {
                pollingRateHz -= 5;
                huskyLens.setTargetPollingRate(pollingRateHz);
            }

            // Toggle background vision thread / 启停后台视觉子线程
            if (currentLeftBumper && !lastLeftBumper) {
                isPollingActive = !isPollingActive;
                if (isPollingActive) {
                    huskyLens.startPolling(currentAlgo);
                } else {
                    huskyLens.stopPolling();
                }
            }

            // Reset physical odometry origin / 物理重置定位仪原点
            if (currentRightBumper && !lastRightBumper) {
                odo.resetPosAndIMU();
            }

            // Update cache / 更新按键缓存
            lastA = currentA;
            lastB = currentB;
            lastX = currentX;
            lastY = currentY;
            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastLeftBumper = currentLeftBumper;
            lastRightBumper = currentRightBumper;

            // -------------------------------------------------------------
            // D. Zero-Latency Vision Data Retrieval / 零延迟视觉数据提取
            // -------------------------------------------------------------
            // Fetches fully decoded targets directly from shared memory (0 I2C calls here).
            // 直接从共享内存提取完全解码的目标数组（此处无任何 I2C 通信，耗时 0ms）。
            HuskyLensV2.Block[] targets = huskyLens.getBlocks();
            HuskyLensV2.Block closest = huskyLens.getClosestBlockToCenter();
            boolean isVisionHealthy = huskyLens.isOnline();

            // -------------------------------------------------------------
            // E. Telemetry Dashboard / 仪表盘渲染
            // -------------------------------------------------------------
            telemetry.addLine("=== ⚙️ SYSTEM HEALTH / 系统监控 ===");
            telemetry.addData("Looptime", "%d ms (%.1f Hz) <- Proof of 0ms block / 证明无阻塞", loopTime, 1000.0 / Math.max(1, loopTime));
            telemetry.addData("Vision Hardware", isVisionHealthy ? "ONLINE (OK)" : "OFFLINE (MELTDOWN PROTECT)");
            telemetry.addData("Vision Thread", isPollingActive ? "RUNNING" : "STOPPED");
            telemetry.addData("Algorithm", currentAlgo.name());
            telemetry.addData("Target Rate", "%d Hz", pollingRateHz);

            telemetry.addLine("\n=== 📍 ODOMETRY / 实时定位 ===");
            telemetry.addData("Pos [X, Y]", "[%.2f, %.2f] Inch", odoX, odoY);
            telemetry.addData("Heading", "%.2f Deg", odoHeading);

            telemetry.addLine("\n=== 🎯 TARGETS / 视觉识别数据 ===");
            telemetry.addData("Total Detected", targets.length);

            if (targets.length == 0) {
                telemetry.addLine("  [No Targets in FOV / 视野内无目标]");
            } else {
                for (int i = 0; i < targets.length; i++) {
                    HuskyLensV2.Block t = targets[i];
                    telemetry.addLine(String.format("  [%d] ID:%d | Lbl:\"%s\" | C:[%d, %d] | S:%dx%d",
                            i + 1, t.id, t.name, t.x, t.y, t.width, t.height));
                }

                // Auto-Aim Output Demonstration / 自瞄数据输出演示
                if (closest != null) {
                    telemetry.addLine("\n>> TACTICAL LOCK-ON / 视觉锁头目标 <<");
                    telemetry.addData("Locked ID", closest.id);
                    telemetry.addData("Label", closest.name);
                    telemetry.addData("Center Error X (Auto-Aim YAW)", closest.x - 320); // Feed to PID Turn / 可喂给底盘转向 PID
                    telemetry.addData("Center Error Y (Auto-Aim PITCH)", closest.y - 240);
                }
            }

            telemetry.update();
        }

        // =====================================================================
        // Post-Match Cleanup / 比赛结束清理
        // =====================================================================
        // Safely terminates background thread to prevent memory leak into the next match.
        // SDK will implicitly call huskyLens.close() anyway, but explicit stop is best practice.
        // 物理杀死多线程资源，防止干扰下一场比赛。
        huskyLens.stopPolling();
    }
}
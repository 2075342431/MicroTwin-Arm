// #include <Arduino.h>
// #include <SCServo.h>

// // --- 硬件引脚定义 ---
// const int BUS_IO_4 = 4; // 目标：原本应该是 ID 15 的舵机所在引脚

// SMS_STS sts; // 总线舵机对象

// void setup()
// {
//     // 1. 初始化 USB 调试串口 (请在电脑打开串口监视器，波特率选 115200)
//     Serial.begin(115200);
//     while(!Serial); // 等待串口连接
//     Serial.println("\n--- 舵机 ID 探测与修正程序开始 ---");

//     // 2. 初始化目标 IO 4 所在的串口 (Serial2)
//     Serial2.begin(1000000, SERIAL_8N1, BUS_IO_4, BUS_IO_4);
//     sts.pSerial = &Serial2;
//     delay(1000);

//     Serial.println("正在扫描 IO 4 引脚上的舵机...");

//     // 3. 使用广播地址 Ping (0xfe 会响应总线上唯一的舵机)
//     int currentID = sts.Ping(0xfe);

//     if (currentID != -1) {
//         Serial.print(">>> 成功发现舵机！当前真实 ID 为: ");
//         Serial.println(currentID);

//         if (currentID != 15) {
//             Serial.println(">>> 正在执行 ID 修正：将 ID 修改为 15...");
            
//             sts.unLockEprom(currentID);      // 解锁当前的 ID
//             delay(100);
//             sts.writeByte(currentID, 5, 15); // 寄存器 5 是 ID 寄存器，写入 15
//             delay(200);
            
//             // 验证修改是否成功
//             if (sts.Ping(15) != -1) {
//                 Serial.println(">>> 修正成功！现在它的 ID 已经是 15 了。");
//                 sts.LockEprom(15); // 锁定新 ID 的 EPROM
//             } else {
//                 Serial.println(">>> 修改似乎失败了，请检查供电或接线。");
//             }
//         } else {
//             Serial.println(">>> 该舵机 ID 已经是 15，无需修改。");
//         }

//         // 4. 尝试进行中位校准 (如果你现在想校准的话)
//         Serial.println(">>> 正在尝试将当前位置设置为中位(2048)...");
//         sts.EnableTorque(15, 0); // 关闭扭力才能校准
//         delay(100);
//         sts.unLockEprom(15);
//         sts.CalibrationOfs(15);  // 校准指令
//         delay(500);
//         sts.LockEprom(15);
//         Serial.println(">>> 校准指令已发送。请重启机械臂查看效果。");

//     } else {
//         Serial.println(">>> [错误] 未发现舵机！");
//         Serial.println("请确认：");
//         Serial.println("1. 舵机是否已经接通 6V-12V 电源？");
//         Serial.println("2. 数据线是否连接在 IO 4 引脚？");
//         Serial.println("3. 舵机指示灯是否亮起？");
//     }
// }

// void loop()
// {
//     // 保持空循环
// }

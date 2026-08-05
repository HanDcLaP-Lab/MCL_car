#ifndef TEST_H_
#define TEST_H_

// ================== 测试程序 ==================
void test_program_imu(void); // 静止并持续输出 IMU 姿态 (自带 while(1), 保持不变)

// ================== 非阻塞测试框架 ==================
// [新增] 测试执行入口: 由 main 主循环每轮调用一次 (TEST_MODE != NORMAL 时),
// 按 test.c 中 ACTIVE_TEST 宏派发到具体测试; 非阻塞状态机, 每轮调用推进一个阶段
void Test_Execute(void);

#endif

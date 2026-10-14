/**
  ******************************************************************************
  * @file    host_test.c
  * @brief   PC 端测试入口 —— 证明 protocol/test 两层真的脱离了硬件
  *
  *  注意本文件不加入 Keil 工程（它自带 main()，会和 User/main.c 冲突），
  *  只在命令行里用 gcc 编译运行：
  *
  *      cd test
  *      gcc -std=c99 -Wall -Wextra -pedantic \
  *          -I../protocol ../protocol/protocol.c test_protocol.c host_test.c \
  *          -o host_test.exe
  *      ./host_test.exe
  *
  *  退出码 = 失败断言数，0 表示全通过，方便接到 CI 里。
  ******************************************************************************
  */
#include <stdio.h>
#include "test_protocol.h"

int main(void)
{
    TEST_Result_t r;
    uint16_t fails;
    uint16_t i;

    fails = TEST_Protocol_Run();
    TEST_Protocol_GetResult(&r);

    printf("[protocol] assertions: %u, failed: %u\r\n",
           (unsigned)r.total, (unsigned)r.failed);

    for (i = 0u; (i < TEST_MAX_RECORDED_FAILS) && (i < r.failed); i++)
    {
        printf("  -> fail at test_protocol.c line %u\r\n", (unsigned)r.failLine[i]);
    }

    printf("[protocol] %s\r\n", (r.failed == 0u) ? "ALL PASSED" : "FAILED");
    return (int)fails;
}

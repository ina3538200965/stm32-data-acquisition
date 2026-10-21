# -*- coding: utf-8 -*-
"""往 Keil 工程文件 Project.uvprojx 里插入新的文件条目

用法：改下面的 JOBS 再跑一次。脚本是幂等的 ——
      已经存在的 FileName 会被跳过，重复跑不会插入两份。

工程里每个源文件长这样（注意缩进是固定 12/14 个空格，改错 Keil 会读不出来）：

            <File>
              <FileName>xxx.c</FileName>
              <FileType>1</FileType>
              <FilePath>.\\driver\\xxx.c</FilePath>
            </File>

FileType：1 = C 文件，5 = 头文件，2 = 汇编，3/4 = 其他文档。
"""
import io

UV = "Project.uvprojx"

# 要插到哪里：键 = 锚点文件的 FilePath（插在它之后），值 = 要插入的文件列表
# (文件名, FileType, 相对路径)
JOBS = [
    (".\\driver\\driver_usart.h", [
        ("ringbuf.c", 1, ".\\driver\\ringbuf.c"),
        ("ringbuf.h", 5, ".\\driver\\ringbuf.h"),
    ]),
    (".\\test\\test_protocol.h", [
        ("test_ringbuf.c", 1, ".\\test\\test_ringbuf.c"),
        ("test_ringbuf.h", 5, ".\\test\\test_ringbuf.h"),
    ]),
    (".\\test\\test_ringbuf.h", [
        ("test_pipeline.c", 1, ".\\test\\test_pipeline.c"),
        ("test_pipeline.h", 5, ".\\test\\test_pipeline.h"),
    ]),
]

with io.open(UV, "r", encoding="utf-8") as fp:
    text = fp.read()

added = 0
for anchor_path, items in JOBS:
    marker = "              <FilePath>%s</FilePath>\n" % anchor_path
    idx = text.find(marker)
    if idx < 0:
        raise SystemExit("找不到锚点 %s，脚本需要更新" % anchor_path)

    end = text.find("            </File>\n", idx)
    if end < 0:
        raise SystemExit("找不到锚点 %s 之后的 </File>" % anchor_path)
    end += len("            </File>\n")

    block = ""
    for name, ftype, path in items:
        if "<FileName>%s</FileName>" % name in text:
            print("已存在，跳过：", name)
            continue
        block += "            <File>\n"
        block += "              <FileName>%s</FileName>\n" % name
        block += "              <FileType>%d</FileType>\n" % ftype
        block += "              <FilePath>%s</FilePath>\n" % path
        block += "            </File>\n"
        added += 1
        print("已插入：", name)

    text = text[:end] + block + text[end:]

with io.open(UV, "w", encoding="utf-8", newline="") as fp:
    fp.write(text)

print("本次新增 %d 个，File 条目总数：%d" % (added, text.count("            <File>")))

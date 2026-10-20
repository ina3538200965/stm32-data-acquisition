# -*- coding: utf-8 -*-
"""把新文件插进 Keil 工程文件 Project.uvprojx 的 driver 组

用法：改下面的 NEW 和 ANCHOR_FILE 再跑一次。
"""
import io

UV = "Project.uvprojx"

# 插在 driver_ssd1306.h 这个条目之后
ANCHOR_FILE = "driver_ssd1306.h"

# (文件名, 类型, 相对路径)  类型：1 = C 文件，5 = 头文件
NEW = [
    ("driver_spi.c",     1, ".\\driver\\driver_spi.c"),
    ("driver_spi.h",     5, ".\\driver\\driver_spi.h"),
    ("driver_w25q64.c",  1, ".\\driver\\driver_w25q64.c"),
    ("driver_w25q64.h",  5, ".\\driver\\driver_w25q64.h"),
]

with io.open(UV, "r", encoding="utf-8") as fp:
    text = fp.read()

marker = "              <FilePath>.\\driver\\%s</FilePath>\n" % ANCHOR_FILE
idx = text.find(marker)
if idx < 0:
    raise SystemExit("找不到锚点 %s，脚本需要更新" % ANCHOR_FILE)

end = text.find("            </File>\n", idx)
if end < 0:
    raise SystemExit("找不到锚点之后的 </File>")
end += len("            </File>\n")

block = ""
for name, ftype, path in NEW:
    if "<FileName>%s</FileName>" % name in text:
        print("已存在，跳过：", name)
        continue
    block += "            <File>\n"
    block += "              <FileName>%s</FileName>\n" % name
    block += "              <FileType>%d</FileType>\n" % ftype
    block += "              <FilePath>%s</FilePath>\n" % path
    block += "            </File>\n"

text = text[:end] + block + text[end:]
with io.open(UV, "w", encoding="utf-8", newline="") as fp:
    fp.write(text)

print("File 条目总数：", text.count("            <File>"))

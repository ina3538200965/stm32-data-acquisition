# -*- coding: utf-8 -*-
"""
生成 SSD1306 用的 6x8 ASCII 点阵字库（ASCII 32 ~ 126，共 95 个）。

布局：每个字符 6 字节，每字节一列，bit0 = 最上面那一行的像素
     （SSD1306 页地址模式就是这个位序）。
     前 5 列是字形，第 6 列留空当字符间隔；第 8 行留空当行间隔。

做法：用 Lucida Console 8 号原生渲染（不缩放，缩放必然糊），
     每个字符画在固定位置，再取固定窗口 (x=1..5, y=1..7)。
     固定窗口 = 所有字符基线自动对齐，不需要按 bbox 再排一次。
"""
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "C:/Windows/Fonts/lucon.ttf"
FONT_SIZE = 8
THRESHOLD = 100

ORIGIN_X = 1
ORIGIN_Y = 0
CHAR_W = 5
CHAR_H = 8          # 8 行正好占满 SSD1306 的一页；第 8 行留给 , ; Q 的下降部
BYTES_PER_CHAR = 6
OUT = "font6x8.h"


def render(ch, report):
    """渲染一个字符，返回 7 行 x 5 列的 0/1 矩阵"""
    img = Image.new("L", (24, 24), 0)
    ImageDraw.Draw(img).text((ORIGIN_X, ORIGIN_Y), ch, fill=255,
                             font=ImageFont.truetype(FONT_PATH, FONT_SIZE))
    px = img.load()

    mat = []
    for r in range(CHAR_H):
        row = []
        for c in range(CHAR_W):
            row.append(1 if px[ORIGIN_X + c, ORIGIN_Y + 1 + r] >= THRESHOLD else 0)
        mat.append(row)

    # 越界检查：字形是否画到窗口外（说明取样窗口要调）
    touched = 0
    for y in range(24):
        for x in range(24):
            if px[x, y] >= THRESHOLD:
                touched += 1
                if not (ORIGIN_X <= x < ORIGIN_X + CHAR_W
                        and ORIGIN_Y + 1 <= y < ORIGIN_Y + 1 + CHAR_H):
                    report.append((ch, x, y))
    return mat, touched


def main():
    chars = [chr(c) for c in range(32, 127)]
    report = []
    lines = []
    empty = []

    for ch in chars:
        mat, touched = render(ch, report)
        if touched == 0:
            empty.append(ch)
        colbytes = []
        for c in range(BYTES_PER_CHAR):
            v = 0
            if c < CHAR_W:
                for r in range(CHAR_H):
                    if mat[r][c]:
                        v |= (1 << r)
            colbytes.append(v)
        lines.append((ch, colbytes))

    with open(OUT, "w", encoding="utf-8") as fp:
        fp.write("/* 6x8 ASCII 点阵字库 —— 由 tools/gen_font.py 自动生成，勿手改 */\n")
        fp.write("/* 每字符 6 字节，每字节一列，bit0 = 顶部像素；第 6 列为字符间隔 */\n")
        fp.write("/* 覆盖 ASCII 32(空格) ~ 126(~)，共 95 个，占 570 字节 Flash */\n")
        fp.write("#ifndef __FONT6X8_H\n#define __FONT6X8_H\n\n")
        fp.write("#include \"stm32f10x.h\"\n\n")
        fp.write("const uint8_t Font6x8[95][6] = {\n")
        for ch, cb in lines:
            desc = "sp" if ch == " " else ch
            fp.write("    {")
            fp.write(", ".join("0x%02X" % b for b in cb))
            fp.write("},   /* %s */\n" % desc)
        fp.write("};\n\n#endif\n")

    print("生成完成：%s，共 %d 个字符" % (OUT, len(lines)))
    if empty:
        print("空白（未画到窗口内）：", empty)
    if report:
        print("!! 有像素落到取样窗口外，需要调窗口：")
        for item in report[:20]:
            print("   ", item)
    else:
        print("所有字符都在 5x7 窗口内")

    for test in "A0g%@MW:._":
        for ch, cb in lines:
            if ch == test:
                print("--- '%s' ---" % ch)
                for r in range(CHAR_H):
                    print("  " + "".join("#" if (cb[c] >> r) & 1 else "."
                                         for c in range(BYTES_PER_CHAR)))


if __name__ == "__main__":
    main()

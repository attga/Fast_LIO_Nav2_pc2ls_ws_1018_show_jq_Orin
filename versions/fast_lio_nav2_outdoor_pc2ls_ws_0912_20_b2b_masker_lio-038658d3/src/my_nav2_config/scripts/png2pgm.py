#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键同步 png 到 pgm 格式
用法: python png2pgm.py
"""
import os, sys
from PIL import Image

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.abspath(os.path.join(script_dir, ".."))
    map_dir = os.path.join(pkg_dir, "maps", "lio_sam_map")
    
    png_path = os.path.join(map_dir, "keepout_mask.png")
    pgm_path = os.path.join(map_dir, "keepout_mask.pgm")
    
    if not os.path.exists(png_path):
        print(f"错误: 找不到文件 {png_path}")
        return
        
    img = Image.open(png_path)
    img.convert('L').save(pgm_path)
    print(f"成功将手绘修改后的 PNG 同步转换为 PGM 格式！")
    print(f"  源图片: {png_path}")
    print(f"  目标图: {pgm_path}")

if __name__ == "__main__":
    main()

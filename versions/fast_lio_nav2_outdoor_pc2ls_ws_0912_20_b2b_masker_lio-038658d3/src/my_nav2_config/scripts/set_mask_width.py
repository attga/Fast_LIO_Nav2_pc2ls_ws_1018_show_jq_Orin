#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
设置 Nav2 禁区掩膜走廊宽度的辅助脚本
用法: python set_mask_width.py --radius 1.5
(radius 1.5 表示走廊单侧半宽 1.5米，即总通道宽度为 3.0米)
"""
import sys, os, argparse, re
import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt

def main():
    parser = argparse.ArgumentParser(description="调整 Keepout 禁区走廊宽度")
    parser.add_argument("--radius", type=float, default=1.5, help="走廊单侧扩展半径(米)，例如 1.5 表示总宽3.0m")
    args = parser.parse_args()

    resolution = 0.05
    edit_origin_x = -68.213730
    edit_origin_y = -108.572270
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.abspath(os.path.join(script_dir, ".."))
    map_dir = os.path.join(pkg_dir, "maps", "lio_sam_map")
    
    script_path = os.path.join(script_dir, "waypoint_through_poses_buinding2building.py")
    mask_pgm_path = os.path.join(map_dir, "keepout_mask.pgm")
    mask_png_path = os.path.join(map_dir, "keepout_mask.png")
    map_edited_path = os.path.join(map_dir, "map_edited.pgm")

    if not os.path.exists(script_path):
        print(f"错误: 找不到航点文件 {script_path}")
        return

    edited_img = Image.open(map_edited_path)
    w_edit, h_edit = edited_img.size
    y_max_edit = edit_origin_y + h_edit * resolution

    # 1. 提取航点数据
    wp_pattern = re.compile(r"\(\s*([-+]?\d*\.\d+|\d+),\s*([-+]?\d*\.\d+|\d+),\s*([-+]?\d*\.\d+|\d+),\s*\"([^\"]+)\"\)")
    wps = []
    with open(script_path, 'r', encoding='utf-8') as f:
        for line in f:
            m = wp_pattern.search(line)
            if m:
                wps.append((float(m.group(1)), float(m.group(2))))

    def world_to_pixel(gx, gy):
        col = int((gx - edit_origin_x) / resolution)
        row = int((y_max_edit - gy) / resolution)
        return col, row

    pix_wps = [world_to_pixel(x, y) for x, y in wps]

    # 2. 绘制基础中心线
    canvas = Image.new('L', (w_edit, h_edit), 0)
    draw = ImageDraw.Draw(canvas)
    draw.line(pix_wps, fill=255, width=4)
    path_arr = np.array(canvas) > 0

    # 3. 按指定的半径计算扩展走廊
    corridor_radius_m = args.radius
    corridor_radius_px = int(corridor_radius_m / resolution)
    print(f"正在重新生成掩膜... 走廊半径: {corridor_radius_m} 米 (总宽度: {corridor_radius_m*2} 米 / {corridor_radius_px*2} 像素)")

    dist_map = distance_transform_edt(~path_arr)
    inside_corridor = (dist_map <= corridor_radius_px)

    # 4. 生成 0/254 栅格 (254为禁区/黑色，0/205为自由走廊)
    # Nav2 Keepout 规范: 0 为自由通行，254 为 LETHAL 禁区
    edited_arr = np.array(edited_img.convert('L'))
    is_free_space = (edited_arr > 180)  # 避开建筑物墙壁
    
    tight_corridor = inside_corridor & is_free_space
    
    # 建立 mask grid: 通道外设为 0 (掩膜图中254/黑色为禁区)
    mask_grid = np.zeros((h_edit, w_edit), dtype=np.uint8)
    # 保持走廊部分可通行
    mask_grid[tight_corridor] = 254

    Image.fromarray(mask_grid).save(mask_pgm_path)
    Image.fromarray(mask_grid).save(mask_png_path)
    print(f"✅ 成功保存新掩膜文件:")
    print(f"   PGM: {mask_pgm_path}")
    print(f"   PNG: {mask_png_path}")

if __name__ == "__main__":
    main()

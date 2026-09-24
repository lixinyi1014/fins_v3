"""字体加载，绕开 pygame 的 SysFont。

pygame 在 Windows 上的 SysFont 会去扫字体注册表（sysfont.py: initsysfonts_win32），
如果注册表里有哪一项的值不是字符串（常见于装过某些字体管理器或游戏之后），
就会抛 `TypeError: expected str, bytes or os.PathLike object, not int`。
这跟具体要用哪个字体无关，只要调用 SysFont 就会炸。

直接按文件路径加载就完全不碰注册表。顺带解决另一个问题：
原来指定的 Consolas 没有中文字形，HUD 上的中文标签会显示成方块。
"""

from __future__ import annotations

import os

import pygame

#: 按优先级排列的字体文件。前面几个是 Windows 自带且含中文的。
CANDIDATES = (
    r"C:\Windows\Fonts\msyh.ttc",      # 微软雅黑
    r"C:\Windows\Fonts\msyh.ttf",
    r"C:\Windows\Fonts\simhei.ttf",    # 黑体
    r"C:\Windows\Fonts\deng.ttf",      # 等线
    r"C:\Windows\Fonts\simsun.ttc",    # 宋体
    "/System/Library/Fonts/PingFang.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
)

_resolved: str | None | bool = False   # False = 还没找过


def font_path() -> str | None:
    """返回第一个存在的字体文件路径；一个都没有就返回 None。"""
    global _resolved
    if _resolved is not False:
        return _resolved            # type: ignore[return-value]
    _resolved = None
    for path in CANDIDATES:
        if os.path.exists(path):
            _resolved = path
            break
    return _resolved                # type: ignore[return-value]


def load_font(size: int, *, bold: bool = False) -> pygame.font.Font:
    """加载指定字号的字体。

    找不到任何中文字体时退回 pygame 自带的默认字体，
    此时中文会显示成方块，但程序不会崩 —— 显示降级好过启动失败。
    """
    path = font_path()
    font = pygame.font.Font(path, size) if path else pygame.font.Font(None, size)
    font.set_bold(bold)
    return font

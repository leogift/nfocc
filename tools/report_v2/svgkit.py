#!/usr/bin/env python3
"""Tiny SVG emitter for the NFOCC report figures.

Why a helper rather than hand-written SVG strings: every figure carries
measured numbers, and those numbers get corrected.  A data-driven figure is a
one-line diff; a hand-written one is a re-layout.

All strings pass through `esc()` so labels like `C<Z>` or `x & y` cannot
produce malformed XML.
"""
from __future__ import annotations

import math
from pathlib import Path

INK = "#1b2b3a"
MUTED = "#5d6d7e"
FAINT = "#8794a3"
ACC = "#1e5a96"
GREEN = "#2e7d32"
AMBER = "#b26a00"
RED = "#c0392b"
PURPLE = "#6a4c93"
TEAL = "#17706e"
WHITE = "#ffffff"

FILL_BLUE = "#edf3fa"
FILL_GREEN = "#eaf5ec"
FILL_AMBER = "#fdf4e5"
FILL_RED = "#fbeceb"
FILL_PURPLE = "#f3eef8"
FILL_GREY = "#f4f6f8"

FONT = '"PingFang SC","Hiragino Sans GB","Heiti SC",sans-serif'
MONO = '"SF Mono","Menlo",monospace'


def esc(s) -> str:
    return (str(s).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


def est_w(s: str, size: float) -> float:
    """Rough advance width: CJK = 1.0 em, ASCII = 0.56 em."""
    return sum(1.0 if ord(c) > 0x2E80 else 0.56 for c in str(s)) * size


class Fig:
    def __init__(self, w: int, h: int, bg: str = WHITE):
        self.w, self.h = w, h
        self.parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" '
            f'width="{w}" height="{h}">',
            f'<rect width="{w}" height="{h}" fill="{bg}"/>',
            f"<style>text{{font-family:{FONT};}}"
            f".mono{{font-family:{MONO};}}</style>",
            '<defs>',
            '<marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M0 0 L10 5 L0 10 z" fill="#456"/></marker>',
            '<marker id="arb" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0 0 L10 5 L0 10 z" fill="{ACC}"/></marker>',
            '<marker id="arg" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0 0 L10 5 L0 10 z" fill="{GREEN}"/></marker>',
            '<marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            f'<path d="M0 0 L10 5 L0 10 z" fill="{RED}"/></marker>',
            "</defs>",
        ]

    # ------------------------------------------------------------------ text
    def text(self, x, y, s, size=16, color=INK, anchor="start",
             weight=None, mono=False):
        w = f' font-weight="{weight}"' if weight else ""
        cls = ' class="mono"' if mono else ""
        self.parts.append(
            f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
            f'fill="{color}" text-anchor="{anchor}"{w}{cls}>{esc(s)}</text>')
        return self

    def mtext(self, x, y, lines, size=15, color=INK, anchor="start",
              line_h=None, weight=None):
        lh = line_h or size * 1.35
        for i, line in enumerate(lines):
            self.text(x, y + i * lh, line, size, color, anchor, weight)
        return self

    # ------------------------------------------------------------- primitives
    def rect(self, x, y, w, h, fill=WHITE, stroke=None, rx=6, sw=1.4, dash=None):
        st = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
        da = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
            f'rx="{rx}" fill="{fill}"{st}{da}/>')
        return self

    def box(self, x, y, w, h, title, sub="", sub2="", fill=FILL_BLUE,
            stroke=ACC, fs=16, ss=13.5, sw=1.4, title_color=INK):
        self.rect(x, y, w, h, fill, stroke, 7, sw)
        n = 1 + bool(sub) + bool(sub2)
        lh = fs * 1.28
        total = (n - 1) * lh
        cy = y + h / 2 - total / 2 + fs * 0.34
        self.text(x + w / 2, cy, title, fs, title_color, "middle", "600")
        if sub:
            self.text(x + w / 2, cy + lh, sub, ss, MUTED, "middle")
        if sub2:
            self.text(x + w / 2, cy + 2 * lh, sub2, ss - 0.6, FAINT, "middle")
        return self

    def line(self, x1, y1, x2, y2, color=MUTED, sw=1.4, dash=None):
        da = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{color}" stroke-width="{sw}"{da}/>')
        return self

    def arrow(self, x1, y1, x2, y2, color=MUTED, dash=None, sw=1.8,
              marker="ar", label="", lx=None, ly=None, lsize=12.5):
        m = {"ar": "ar", "acc": "arb", "green": "arg", "red": "arr",
             "arb": "arb", "arg": "arg", "arr": "arr"}[marker]
        da = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{color}" stroke-width="{sw}"{da} marker-end="url(#{m})"/>')
        if label:
            self.text(lx if lx is not None else (x1 + x2) / 2,
                      ly if ly is not None else (y1 + y2) / 2 - 7,
                      label, lsize, color, "middle")
        return self

    def circle(self, cx, cy, r, fill=WHITE, stroke=ACC, sw=1.5):
        self.parts.append(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="{sw}"/>')
        return self

    def poly(self, pts, fill="none", stroke=ACC, sw=1.6, dash=None):
        p = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        da = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<polygon points="{p}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{sw}"{da} stroke-linejoin="round"/>')
        return self

    def path(self, d, fill="none", stroke=ACC, sw=1.8, dash=None,
             marker=None):
        da = f' stroke-dasharray="{dash}"' if dash else ""
        mk = f' marker-end="url(#{marker})"' if marker else ""
        self.parts.append(
            f'<path d="{d}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"'
            f'{da}{mk} stroke-linejoin="round"/>')
        return self

    def pill(self, x, y, w, h, s, fill=WHITE, stroke=ACC, color=INK,
             size=14, weight="600"):
        self.rect(x, y, w, h, fill, stroke, h / 2, 1.3)
        self.text(x + w / 2, y + h / 2 + size * 0.36, s, size, color,
                  "middle", weight)
        return self

    def title(self, s, sub="", y=34):
        self.text(30, y, s, 25, INK, weight="600")
        if sub:
            self.text(30, y + 25, sub, 15, MUTED)
        return self

    def note(self, x, y, s, size=12.5, color=FAINT):
        return self.text(x, y, s, size, color)

    # ------------------------------------------------------------------ chart
    def axes(self, x0, y0, w, h, xmin, xmax, ymin, ymax, xticks, yticks,
             xlabel="", ylabel="", fmt_x="{:.0f}", fmt_y="{:.0f}"):
        self.line(x0, y0 + h, x0 + w, y0 + h, INK, 1.3)
        self.line(x0, y0, x0, y0 + h, INK, 1.3)
        for v in xticks:
            px = x0 + (v - xmin) / (xmax - xmin) * w
            self.line(px, y0 + h, px, y0 + h + 4, INK, 1.2)
            self.text(px, y0 + h + 20, fmt_x.format(v), 13, MUTED, "middle")
        for v in yticks:
            py = y0 + h - (v - ymin) / (ymax - ymin) * h
            self.line(x0 - 4, py, x0, py, INK, 1.2)
            self.line(x0, py, x0 + w, py, "#e3e8ee", 0.9, "3 3")
            self.text(x0 - 9, py + 4.5, fmt_y.format(v), 13, MUTED, "end")
        if xlabel:
            self.text(x0 + w / 2, y0 + h + 42, xlabel, 14, MUTED, "middle")
        if ylabel:
            self.parts.append(
                f'<text x="{x0 - 58}" y="{y0 + h/2}" font-size="14" '
                f'fill="{MUTED}" text-anchor="middle" '
                f'transform="rotate(-90 {x0 - 58} {y0 + h/2})">'
                f'{esc(ylabel)}</text>')
        return self

    def save(self, path):
        svg = "\n".join(self.parts + ["</svg>"]) + "\n"
        Path(path).write_text(svg, encoding="utf-8")
        return svg

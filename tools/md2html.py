#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""md (+ LaTeX math) -> self-contained HTML, for Chromium headless PDF.

Usage:
    python3 md2html.py in.md out.html
    "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge" --headless \
      --disable-gpu --no-sandbox --print-to-pdf=out.pdf --no-pdf-header-footer \
      "file://$PWD/out.html"

Why: Chromium dropped MathML, so LaTeX math must be converted by hand (no KaTeX/network).
Keeps emoji native; uses system fonts. Dense default: body 9pt, A4 14mm margins.
"""
import re, subprocess, sys, shutil

PANDOC = shutil.which("pandoc") or "/Users/dora/.local/pandoc/pandoc-3.11-arm64/bin/pandoc"


def tex_to_html(s):
    def frac(m):
        return ('<span class="frac"><span class="num">%s</span>'
                '<span class="den">%s</span></span>' % (tex_to_html(m.group(1)), tex_to_html(m.group(2))))
    for _ in range(5):  # nested \frac
        s2 = re.sub(r'\\frac\{([^{}]*)\}\{([^{}]*)\}', frac, s)
        if s2 == s:
            break
        s = s2
    s = re.sub(r'\\mathrm\{([^{}]*)\}', r'\1', s)
    s = re.sub(r'\\text\{([^{}]*)\}', r'\1', s)
    s = re.sub(r'\\mathbb\{R\}', 'ℝ', s)
    s = re.sub(r'\\mathbb\{([^{}]*)\}', r'𝕜', s)
    s = re.sub(r'\\mathcal\{L\}', 'ℒ', s)
    s = re.sub(r'\\mathcal\{([^{}]*)\}', r'\1', s)
    s = re.sub(r'\\mathbf\{([^{}]*)\}', r'<b>\1</b>', s)
    s = re.sub(r'\\hat\s*([A-Za-z])', lambda m: m.group(1) + '\u0302', s)
    for a, b in [('\\big(', '('), ('\\Big(', '('), ('\\big)', ')'), ('\\Big)', ')'),
                 ('\\big[', '['), ('\\Big[', '['), ('\\big]', ']'), ('\\Big]', ']')]:
        s = s.replace(a, b)
    s = re.sub(r'_\{([^{}]*)\}', r'<sub>\1</sub>', s)
    s = re.sub(r'\^\{([^{}]*)\}', r'<sup>\1</sup>', s)
    s = re.sub(r'_([A-Za-z0-9])', r'<sub>\1</sub>', s)
    s = re.sub(r'\^([A-Za-z0-9])', r'<sup>\1</sup>', s)
    repl = {
        r'\odot': '⊙', r'\cdot': '·', r'\times': '×', r'\sum': 'Σ',
        r'\max': 'max', r'\min': 'min', r'\exp': 'exp', r'\log': 'log',
        r'\sin': 'sin', r'\cos': 'cos', r'\tan': 'tan',
        r'\in': '∈', r'\|': '‖', r'\ge': '≥', r'\leq': '≤', r'\le': '≤',
        r'\to': '→', r'\Delta': 'Δ', r'\lambda': 'λ', r'\theta': 'θ',
        r'\alpha': 'α', r'\gamma': 'γ', r'\phi': 'φ', r'\pi': 'π',
        r'\quad': '&nbsp;&nbsp;', r'\;': '&nbsp;', r'\ ': ' ',
    }
    for k, v in repl.items():
        s = s.replace(k, v)
    return s


CSS = """
@page { size: A4; margin: 14mm 14mm 14mm 14mm; }
* { box-sizing: border-box; }
html, body { margin: 0; padding: 0; }
body {
  font-family: -apple-system, "PingFang SC", "Hiragino Sans GB", "Helvetica Neue", Arial, sans-serif;
  font-size: 9pt; line-height: 1.34; color: #1a1a1a;
}
h1 { font-size: 14pt; margin: 0 0 6pt; color: #0b2545; }
h2 { font-size: 11.5pt; margin: 10pt 0 4pt; color: #0b3d6b; border-bottom: 1px solid #c9d6e4; padding-bottom: 2pt; }
h3 { font-size: 10pt; margin: 7pt 0 2pt; color: #1c4e80; }
h4 { font-size: 9.5pt; margin: 5pt 0 2pt; color: #333; }
p { margin: 2pt 0 4pt; }
ul, ol { margin: 2pt 0 4pt; padding-left: 18pt; }
li { margin: 1pt 0; }
table { border-collapse: collapse; width: 100%; margin: 4pt 0 6pt; font-size: 8.3pt; }
th, td { border: 0.6pt solid #b7c3d0; padding: 2pt 4pt; text-align: left; vertical-align: top; }
th { background: #eef3f9; font-weight: 600; }
code { font-family: "Menlo", "SF Mono", monospace; font-size: 8pt; background: #f2f4f7; padding: 0 2pt; border-radius: 2pt; }
pre { background: #f5f7fa; border: 0.6pt solid #dde3ea; border-radius: 3pt; padding: 5pt 7pt; overflow-x: auto; }
pre code { background: none; padding: 0; font-size: 7.6pt; line-height: 1.25; }
blockquote { margin: 4pt 0; padding: 3pt 8pt; border-left: 3pt solid #7ba3c8; background: #f4f8fb; color: #33414f; }
blockquote p { margin: 1pt 0; }
hr { border: none; border-top: 0.6pt solid #c9d6e4; margin: 6pt 0; }
img { width: 12.5cm; display: block; margin: 4pt auto; }
.m { font-style: normal; }
.eq { text-align: center; margin: 4pt 0; font-size: 9pt; }
.frac { display: inline-block; vertical-align: middle; text-align: center; margin: 0 2pt; }
.frac .num { display: block; border-bottom: 0.7pt solid #1a1a1a; padding: 0 2pt; }
.frac .den { display: block; padding: 0 2pt; }
sub, sup { font-size: 6.5pt; }
"""


def main():
    src, out = sys.argv[1], sys.argv[2]
    md = open(src, encoding="utf-8").read()
    # display $$...$$ first, then inline $...$
    md = re.sub(r'\$\$(.+?)\$\$',
                lambda m: '<div class="eq">%s</div>' % tex_to_html(m.group(1).strip()),
                md, flags=re.S)
    md = re.sub(r'\$([^$\n]+?)\$',
                lambda m: '<span class="m">%s</span>' % tex_to_html(m.group(1)), md)
    open("/tmp/pre.md", "w", encoding="utf-8").write(md)

    body = subprocess.run([PANDOC, "/tmp/pre.md", "-f", "markdown+tex_math_dollars",
                           "-t", "html", "--wrap=none"], capture_output=True, text=True).stdout
    html = ("<!DOCTYPE html>\n<html lang=\"zh-CN\"><head><meta charset=\"utf-8\">\n"
            "<style>%s</style></head>\n<body>\n%s\n</body></html>" % (CSS, body))
    open(out, "w", encoding="utf-8").write(html)
    print("wrote %s (%d chars)" % (out, len(html)))


if __name__ == "__main__":
    main()

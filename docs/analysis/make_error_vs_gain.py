#!/usr/bin/env python3
"""Generates docs/figures/error_vs_gain.svg from the measured gain-sweep data."""
import math

# (P gain, elbow tracking error in degrees) for target 0, all else held fixed
T0 = [(16, 7.96), (32, 3.32), (48, 2.82)]
T1 = (48, 17.67)                      # extended-reach pose, same gain, same elbow load
C = sum(p * e for p, e in T0) / len(T0)   # fitted constant of err = C / P

W, H = 780, 470
L, R, T, B = 82, 34, 58, 106
PX0, PX1, PY0, PY1 = L, W - R, T, H - B
XMAX, YMAX = 56.0, 20.0

def x(p): return PX0 + (p / XMAX) * (PX1 - PX0)
def y(e): return PY1 - (e / YMAX) * (PY1 - PY0)

s = []
s.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
         f'font-family="ui-sans-serif,-apple-system,Segoe UI,Helvetica,Arial,sans-serif">')
s.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
s.append(f'<text x="{L}" y="26" font-size="15" font-weight="600" fill="#0f172a">'
         'Elbow tracking error vs servo proportional gain</text>')
s.append(f'<text x="{L}" y="44" font-size="11.5" fill="#64748b">'
         'Target, calibration, planner and trajectory held fixed; only the elbow gain varied.</text>')

# grid + y ticks
for e in range(0, 21, 4):
    yy = y(e)
    s.append(f'<line x1="{PX0}" y1="{yy:.1f}" x2="{PX1}" y2="{yy:.1f}" stroke="#e2e8f0" stroke-width="1"/>')
    s.append(f'<text x="{PX0-10}" y="{yy+4:.1f}" font-size="11" fill="#64748b" text-anchor="end">{e}</text>')
# x ticks
for p in [0, 8, 16, 24, 32, 40, 48, 56]:
    xx = x(p)
    s.append(f'<line x1="{xx:.1f}" y1="{PY1}" x2="{xx:.1f}" y2="{PY1+5}" stroke="#94a3b8" stroke-width="1"/>')
    s.append(f'<text x="{xx:.1f}" y="{PY1+20}" font-size="11" fill="#64748b" text-anchor="middle">{p}</text>')
s.append(f'<line x1="{PX0}" y1="{PY1}" x2="{PX1}" y2="{PY1}" stroke="#94a3b8" stroke-width="1.4"/>')
s.append(f'<line x1="{PX0}" y1="{PY0}" x2="{PX0}" y2="{PY1}" stroke="#94a3b8" stroke-width="1.4"/>')
s.append(f'<text x="{(PX0+PX1)/2:.0f}" y="{PY1+46}" font-size="12" fill="#334155" text-anchor="middle">'
         'servo proportional gain P</text>')
s.append(f'<text transform="translate(24,{(PY0+PY1)/2:.0f}) rotate(-90)" font-size="12" fill="#334155" '
         'text-anchor="middle">elbow tracking error (deg)</text>')

# fitted 1/P curve
pts = []
p = 10.0
while p <= XMAX + 0.01:
    e = C / p
    if e <= YMAX:
        pts.append(f"{x(p):.1f},{y(e):.1f}")
    p += 0.5
s.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="#2563eb" stroke-width="2" '
         'stroke-dasharray="6 4" opacity="0.75"/>')
s.append(f'<text x="{x(20):.1f}" y="{y(C/20)-12:.1f}" font-size="11.5" fill="#2563eb">'
         f'steady-state P-control model:  error = {C:.0f}/P</text>')

# measured T0 points
for p, e in T0:
    s.append(f'<circle cx="{x(p):.1f}" cy="{y(e):.1f}" r="6" fill="#2563eb" stroke="#ffffff" stroke-width="2"/>')
    s.append(f'<text x="{x(p):.1f}" y="{y(e)-14:.1f}" font-size="11.5" font-weight="600" fill="#1e3a8a" '
             f'text-anchor="middle">{e:.2f}&#176;</text>')

# the off-curve point
s.append(f'<circle cx="{x(T1[0]):.1f}" cy="{y(T1[1]):.1f}" r="7" fill="#dc2626" stroke="#ffffff" stroke-width="2"/>')
s.append(f'<line x1="{x(T1[0]):.1f}" y1="{y(T1[1])+10:.1f}" x2="{x(T1[0]):.1f}" y2="{y(C/48)-10:.1f}" '
         'stroke="#dc2626" stroke-width="1.3" stroke-dasharray="3 3"/>')
s.append(f'<text x="{x(T1[0])-12:.1f}" y="{y(T1[1])-14:.1f}" font-size="11.5" font-weight="600" fill="#b91c1c" '
         f'text-anchor="end">17.67&#176; &#8212; extended-reach pose</text>')
s.append(f'<text x="{x(T1[0])-12:.1f}" y="{y(T1[1])+2:.1f}" font-size="11" fill="#b91c1c" text-anchor="end">'
         'same gain, same elbow gravity load (0.437 N&#183;m)</text>')
s.append(f'<text x="{x(T1[0])-12:.1f}" y="{y(T1[1])+17:.1f}" font-size="11" fill="#b91c1c" text-anchor="end">'
         '6.3&#215; the error &#8594; static compliance refuted</text>')

# legend
ly = PY1 + 78
s.append(f'<circle cx="{L+6}" cy="{ly-4}" r="5" fill="#2563eb"/>')
s.append(f'<text x="{L+18}" y="{ly}" font-size="11" fill="#475569">target 0 (folded, 82% reach)</text>')
s.append(f'<circle cx="{L+248}" cy="{ly-4}" r="5" fill="#dc2626"/>')
s.append(f'<text x="{L+260}" y="{ly}" font-size="11" fill="#475569">target 1 (extended, 90% reach)</text>')
s.append('</svg>')

out = "docs/figures/error_vs_gain.svg"
open(out, "w").write("\n".join(s))
print(f"wrote {out}  (fitted C = {C:.1f}; model predicts "
      f"{', '.join(f'{C/p:.2f}' for p, _ in T0)} deg at P = 16/32/48)")

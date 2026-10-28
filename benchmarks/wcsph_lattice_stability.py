"""Linear stability of a prestressed square SPH lattice (symmetric p_i/rho_i^2 + p_j/rho_j^2 form).

Standalone analysis behind the WCSPH default kernel choice (docs/wcsph-solver.md).
Run: python benchmarks/wcsph_lattice_stability.py  (h is the full support radius;
p/rho = 14.7 m^2/s^2 is the floor pressure of a 1.5 m water column, c = 40 m/s).
It ignores walls, damping, density diffusion and shifting.

omega^2(k) = eig( c^2 V^2 g g^T + 2 (p0/rho0) V sum_s H(s) (1 - cos k.s) )
Negative eigenvalue => exponential growth with rate sqrt(-omega^2).
"""
import math, sys

def radial(family, r, h):
    q = r / h
    if q >= 1: return 0.0, 0.0
    if family == 'spiky':
        n = 10 / (math.pi * h * h)
        return n * (-3 / h) * (1 - q) ** 2, n * (6 / h / h) * (1 - q)
    if family == 'cubic':
        n = 40 / (7 * math.pi * h * h)
        u = 2 * q
        if u < 1:
            return n * (-6 * u + 4.5 * u * u) * 2 / h, n * (-6 + 9 * u) * 4 / h / h
        return n * (-1.5 * (2 - u) ** 2) * 2 / h, n * (3 * (2 - u)) * 4 / h / h
    if family == 'wendland':
        n = 7 / (math.pi * h * h)
        return n * (-20 * q * (1 - q) ** 3) / h, n * (20 * (1 - q) ** 2 * (4 * q - 1)) / h / h
    raise ValueError(family)

def eigmin(a, b, c):  # [[a,b],[b,c]]
    m = 0.5 * (a + c); d = math.sqrt(0.25 * (a - c) ** 2 + b * b)
    return m - d

def analyse(family, hratio, p_over_rho=14.7, c=40.0, d=0.1, pmode='sum'):
    h = hratio * d
    V = d * d
    n = int(math.ceil(hratio)) + 1
    data = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            if (i, j) == (0, 0): continue
            x, y = i * d, j * d
            r = math.hypot(x, y)
            w1, w2 = radial(family, r, h)
            if w1 == 0 and w2 == 0: continue
            ex, ey = x / r, y / r
            t = w1 / r
            H = (w2 * ex * ex + t * (1 - ex * ex), (w2 - t) * ex * ey, w2 * ey * ey + t * (1 - ey * ey))
            data.append((x, y, w1 * ex, w1 * ey, H))
    worst = (0, None)
    N = 48
    for a in range(N + 1):
        for b in range(N + 1):
            kx = math.pi / d * a / N; ky = math.pi / d * b / N
            gx = gy = 0.0; P = [0.0, 0.0, 0.0]
            for x, y, dx_, dy_, H in data:
                ks = kx * x + ky * y
                s = math.sin(ks); cc = 1 - math.cos(ks)
                gx += dx_ * s; gy += dy_ * s
                P[0] += H[0] * cc; P[1] += H[1] * cc; P[2] += H[2] * cc
            A = c * c * V * V
            pf = 2 * p_over_rho * V if pmode == 'sum' else 0.0
            lam = eigmin(A * gx * gx + pf * P[0], A * gx * gy + pf * P[1], A * gy * gy + pf * P[2])
            if lam < worst[0]: worst = (lam, (a / N, b / N))
    rate = math.sqrt(-worst[0]) if worst[0] < 0 else 0.0
    return rate, worst[1], len(data)

for fam in ['spiky', 'cubic', 'wendland']:
    for hr in [2.0, 2.5, 3.0]:
        rate, kk, nn = analyse(fam, hr)
        print(f"{fam:9s} h/dx={hr}: neighbours={nn:3d} max growth rate {rate:7.3f} 1/s at k/(pi/dx)={kk}")

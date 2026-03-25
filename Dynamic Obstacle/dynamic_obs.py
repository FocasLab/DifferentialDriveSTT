import numpy as np
import matplotlib.pyplot as plt
from z3 import *
import pandas as pd
import time
import matplotlib.animation as animation

USE_C1 = True
TIMEOUT_MS = 30000           # per-check timeout; guarantees return

dim = 2
N = 1000          # number of timesamples
eta = -0.0075    # useed for verification
t0 = 1e-5       # Start time
tf = 5.0        # Final time
t_vec = np.linspace(t0, tf, N)
S_X, S_Y, S_r = 0.35, 0.35, 0.20  # Start region
T_X, T_Y, T_r = 2.65, 2.65, 0.20  # Target region

# State space bounds
X_L, Y_L = 0.0, 0.0
X_U, Y_U = 3.0, 3.0

# Segmentation (3 segments, equal length)
M = 4
breaks = np.linspace(t0, tf, M+1)  # [t0, t1, t2, tf]
print(breaks)
dts = np.diff(breaks)              # [dt0, dt1, dt2]

OA_x0, OA_y0, OA_R = 0.95, 0.35, 0.25  # Obstacle A: upward motion
OB_x0, OB_y0, OB_R = 1.65, 2.65, 0.25  # Obstacle B: downward motion
OC_x0, OC_y0, OC_R = 0.95, 0.35, 0.25   # Obstacle C: static
OD_x0, OD_y0, OD_R = 1.35, 2.65, 0.25   # Obstacle D: static

# Speed of obstacle
speed_A = (1.35 - 0.35) / (tf-t0) 
speed_B = (2.65 - 1.35) / (tf - t0)

def obsA(t):
    return (OA_x0, OA_y0 + speed_A * t, OA_R) # going upward

def obsB(t):
    return (OB_x0, OB_y0 - speed_B * t, OB_R) # going downward

def obsC(t):
    return (OC_x0, OC_y0, OC_R) # static

def obsD(t):
    return (OD_x0, OD_y0, OD_R) # static

obsA_samples = [obsA(tk) for tk in t_vec] 
obsB_samples = [obsB(tk) for tk in t_vec]
obsC_samples = [obsC(tk) for tk in t_vec]
obsD_samples = [obsD(tk) for tk in t_vec]

# Define variables
# For each segment s=0,1,2 we have (ax, bx, cx), (ay, by, cy), (ar, br, cr)
# That’s 3 coeffs * 3 signals * M segments = 9M reals
ax = [Real(f"ax_{s}") for s in range(M)]
bx = [Real(f"bx_{s}") for s in range(M)]
cx = [Real(f"cx_{s}") for s in range(M)]

ay = [Real(f"ay_{s}") for s in range(M)]
by = [Real(f"by_{s}") for s in range(M)]
cy = [Real(f"cy_{s}") for s in range(M)]

ar = [Real(f"ar_{s}") for s in range(M)]
br = [Real(f"br_{s}") for s in range(M)]
cr = [Real(f"cr_{s}") for s in range(M)]

# Define the tubes at t = tk
def seg_index(tk):
    # ensure last point maps to last segment
    for i in range(M-1):
        if tk <= breaks[i+1]:
            return i
    return M-1

def gam_exprs_at(tk):
    sidx = seg_index(tk)
    tau = (tk - breaks[sidx]) / (breaks[sidx+1] - breaks[sidx])
    gx = ax[sidx] + bx[sidx]*tau + cx[sidx]*(tau*tau)
    gy = ay[sidx] + by[sidx]*tau + cy[sidx]*(tau*tau)
    gr = ar[sidx] + br[sidx]*tau + cr[sidx]*(tau*tau)
    return gx, gy, gr

# Initialize Solver 
s = Solver()

# Start at t0 and End at tf (tau=1 for segment M)
s.add(ax[0] == S_X, ay[0] == S_Y, ar[0] == S_r)
s.add(ax[M-1] + bx[M-1] + cx[M-1] == T_X)
s.add(ay[M-1] + by[M-1] + cy[M-1] == T_Y)
s.add(ar[M-1] + br[M-1] + cr[M-1] == T_r)

# Ensure continuity
# C0: p0(1) == p1(0)  -> a0 + b0 + c0 == a1
# C1: dp0/dt at tau=1 == dp1/dt at tau=0
# (b0 + 2*c0)/dt0 == (b1)/dt1
# At t1 (between seg0 and seg1)
for k in range(M-1):
    # C0: a_k + b_k + c_k == a_{k+1}
    s.add(ax[k] + bx[k] + cx[k] == ax[k+1])
    s.add(ay[k] + by[k] + cy[k] == ay[k+1])
    s.add(ar[k] + br[k] + cr[k] == ar[k+1])
    if USE_C1:
        # (b_k + 2 c_k)/dt_k == (b_{k+1})/dt_{k+1}  -> cross multiply
        s.add((bx[k] + 2*cx[k]) * dts[k+1] == bx[k+1] * dts[k])
        s.add((by[k] + 2*cy[k]) * dts[k+1] == by[k+1] * dts[k])
        s.add((br[k] + 2*cr[k]) * dts[k+1] == br[k+1] * dts[k])

# State-space constraint
for tk in t_vec:
    gx, gy, gr = gam_exprs_at(tk)
    s.add(And(gx - X_L - gr > 0, gy - Y_L - gr > 0, X_U - gx - gr > 0, Y_U - gy - gr > 0, gr > 0.1))

# Obstacle avoidance constraint : Circular keep-out: (gx-ox)^2 + (gy-oy)^2 > (gr + R)^2
for idx, tk in enumerate(t_vec):
    gx, gy, gr = gam_exprs_at(tk)

    for (ox, oy, RR) in (obsA_samples[idx], obsB_samples[idx], obsC_samples[idx], obsD_samples[idx]):
        s.add((gx - ox)*(gx - ox) + (gy - oy)*(gy - oy) - (gr + RR)*(gr + RR) > -eta)

# Solution
tic = time.time()
print("Model Check start")
res = s.check()
toc = time.time()
print("SMT status:", res, "in", round(toc - tic, 3), "s")
print("Epsilon = ", (tf - t0)/N, "eta = ", eta)

if res != sat:
    print("Unsatisfiable. Try reducing N, relaxing to box constraints, or dropping C1 continuity.")
else:
    m = s.model()
    def getv(v): 
        val = m[v]
        return float(val.numerator_as_long())/float(val.denominator_as_long()) if is_rational_value(val) else float(val.as_decimal(20).replace('?', ''))

    coef = {
        'ax': np.array([getv(v) for v in ax]),
        'bx': np.array([getv(v) for v in bx]),
        'cx': np.array([getv(v) for v in cx]),
        'ay': np.array([getv(v) for v in ay]),
        'by': np.array([getv(v) for v in by]),
        'cy': np.array([getv(v) for v in cy]),
        'ar': np.array([getv(v) for v in ar]),
        'br': np.array([getv(v) for v in br]),
        'cr': np.array([getv(v) for v in cr]),
    }
    df = pd.DataFrame(coef)
    df.to_excel("dynamic_tube.xlsx", index=False)

    # Animation Setup 
    fig, ax_plot = plt.subplots()
    ax_plot.set_xlim([X_L-0.1, X_U+0.1])
    ax_plot.set_ylim([Y_L-0.1, Y_U+0.1])
    ax_plot.set_aspect('equal', adjustable='box')
    ax_plot.set_xlabel('X')
    ax_plot.set_ylabel('Y')
    ax_plot.set_title('Trajectory Animation with Dynamic Obstacles')
    ax_plot.grid(True)

    # Draw start and target
    start_patch = plt.Circle((S_X, S_Y), S_r, fill=False, linestyle='--', label='Start')
    target_patch = plt.Circle((T_X, T_Y), T_r, fill=False, linestyle='--', label='Target')
    ax_plot.add_patch(start_patch)
    ax_plot.add_patch(target_patch)

    # Tube center line (will draw as it grows)
    traj_line, = ax_plot.plot([], [], 'r-', lw=2, label='Tube center')

    # Obstacles
    obsA_patch = plt.Circle((0, 0), 0.0, color='r', alpha=0.3, label='Obstacle A')
    obsB_patch = plt.Circle((0, 0), 0.0, color='r', alpha=0.3, label='Obstacle B')
    obsC_patch = plt.Circle((0, 0), 0.0, color='r', alpha=0.3, label='Obstacle C')
    obsD_patch = plt.Circle((0, 0), 0.0, color='r', alpha=0.3, label='Obstacle D')
    ax_plot.add_patch(obsA_patch)
    ax_plot.add_patch(obsB_patch)
    ax_plot.add_patch(obsC_patch)
    ax_plot.add_patch(obsD_patch)

    ax_plot.legend()

    # Use 300 frames for smooth animation
    frames = 300
    t_anim = np.linspace(t0, tf, frames)
    Xc_anim, Yc_anim, Rc_anim = [], [], []
    for tk in t_anim:
        # Find segment
        if tk <= breaks[1]:
            sidx = 0
        elif tk <= breaks[2]:
            sidx = 1
        elif tk <= breaks[3]:
            sidx = 2
        else:
            sidx = 3
        tau = (tk - breaks[sidx]) / (breaks[sidx+1] - breaks[sidx])
        Xc_anim.append(coef['ax'][sidx] + coef['bx'][sidx]*tau + coef['cx'][sidx]*(tau*tau))
        Yc_anim.append(coef['ay'][sidx] + coef['by'][sidx]*tau + coef['cy'][sidx]*(tau*tau))
        Rc_anim.append(coef['ar'][sidx] + coef['br'][sidx]*tau + coef['cr'][sidx]*(tau*tau))

    def init():
        traj_line.set_data([], [])
        obsA_patch.center = (OA_x0, OA_y0)
        obsB_patch.center = (OB_x0, OB_y0)
        obsC_patch.center = (OC_x0, OC_y0)
        obsD_patch.center = (OD_x0, OD_y0)
        obsA_patch.set_radius(OA_R)
        obsB_patch.set_radius(OB_R)
        obsC_patch.set_radius(OC_R)
        obsD_patch.set_radius(OD_R)
        return traj_line, obsA_patch, obsB_patch, obsC_patch, obsD_patch

    def animate(i):
        # Draw trajectory up to frame i
        traj_line.set_data(Xc_anim[:i+1], Yc_anim[:i+1])

        # Move obstacle A upward
        oxA, oyA, RA = obsA(t_anim[i])
        obsA_patch.center = (oxA, oyA)
        obsA_patch.set_radius(RA)

        # Move obstacle B downward
        oxB, oyB, RB = obsB(t_anim[i])
        obsB_patch.center = (oxB, oyB)
        obsB_patch.set_radius(RB)

        # Move obstacle C downward
        oxC, oyC, RC = obsC(t_anim[i])
        obsC_patch.center = (oxC, oyC)
        obsC_patch.set_radius(RC)

        # Move obstacle D downward
        oxD, oyD, RD = obsD(t_anim[i])
        obsD_patch.center = (oxD, oyD)
        obsD_patch.set_radius(RD)

        return traj_line, obsA_patch, obsB_patch, obsC_patch, obsD_patch

    # Run Animation
    ani = animation.FuncAnimation(
        fig, animate, init_func=init,
        frames=frames, interval=30, blit=True
    )

    plt.show()
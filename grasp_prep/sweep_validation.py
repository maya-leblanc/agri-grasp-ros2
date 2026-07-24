import numpy as np, csv, itertools
import single_view as sv

GT_MM = 86.6            # <-- replace with sphere_mm from the MATLAB script above
AZ = range(0, 360, 30)  # 12 azimuths
EL = [-10, 10, 30]      # 3 elevations, all clear of the branch
RADIUS = 0.30
NOISE_LEVELS = [0.0, 0.001, 0.003]

rows=[]
for noise in NOISE_LEVELS:
    sv.NOISE_SIGMA = noise
    for az, el in itertools.product(AZ, EL):
        a, e = np.radians(az), np.radians(el)
        off = RADIUS*np.array([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)])
        try:
            _, out = sv.run(view_offset=off, save=False)
        except Exception as ex:
            print(f"noise={noise} az={az} el={el} FAILED: {ex}"); continue
        err = out["diameter_mm"] - GT_MM
        rows.append([noise, az, el, out["diameter_mm"], err,
                     out["rms_mm"], out["inlier_frac"], out["n_points"]])
        print(f"noise={noise:.4f} az={az:3d} el={el:3d} "
              f"d={out['diameter_mm']:.2f}mm err={err:+.2f}mm rms={out['rms_mm']:.2f}")

with open("sweep_results.csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(["noise","az","el","diameter_mm","err_mm","rms_mm","inlier_frac","n_points"])
    w.writerows(rows)

a=np.array(rows,dtype=float)
print("\n=== SUMMARY ===")
for noise in NOISE_LEVELS:
    s=a[a[:,0]==noise]
    if len(s)==0: continue
    print(f"noise={noise*1000:.1f}mm  n={len(s):3d}  "
          f"bias={s[:,4].mean():+.2f}mm  std={s[:,4].std():.2f}mm  "
          f"MAE={np.abs(s[:,4]).mean():.2f}mm")
import numpy as np, csv, itertools
import single_view as sv

GT_MM = 85.01            # <-- REPLACE with sphere_mm from the MATLAB sphere-fit script
AZ = range(0, 360, 30)  # 12 azimuths
EL = [15, 35, 55]       # upper hemisphere only — never looking up into the branch
RADIUS = 0.30
NOISE_LEVELS = [0.0, 0.001, 0.003]

rows = []
for noise in NOISE_LEVELS:
    sv.NOISE_SIGMA = noise
    for az, el in itertools.product(AZ, EL):
        a, e = np.radians(az), np.radians(el)
        off = RADIUS*np.array([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)])
        try:
            _, out = sv.run(view_offset=off, save=False)
        except Exception as ex:
            print(f"noise={noise:.4f} az={az:3d} el={el:3d} FAILED: {ex}")
            continue
        err = out["diameter_mm"] - GT_MM
        rows.append([noise, az, el, out["diameter_mm"], err, out["rms_mm"],
                     out["inlier_frac"], out["coverage_deg"], out["center_err_mm"],
                     out["n_points"], int(out["accepted"])])
        tag = "OK " if out["accepted"] else "REJ"
        print(f"noise={noise:.4f} az={az:3d} el={el:3d} [{tag}] "
              f"d={out['diameter_mm']:.2f} err={err:+.2f} cov={out['coverage_deg']:.0f} "
              f"why={out['reject_reason']}")

with open("sweep_results.csv","w",newline="") as f:
    w = csv.writer(f)
    w.writerow(["noise","az","el","diameter_mm","err_mm","rms_mm",
                "inlier_frac","coverage_deg","center_err_mm","n_points","accepted"])
    w.writerows(rows)

a = np.array(rows, dtype=float)
print("\n=== SUMMARY (accepted views only) ===")
for noise in NOISE_LEVELS:
    s = a[a[:,0]==noise]
    acc = s[s[:,-1]==1]
    if len(acc)==0:
        print(f"noise={noise*1000:.1f}mm  accepted=0/{len(s)}  (all views rejected)")
        continue
    print(f"noise={noise*1000:.1f}mm  accepted={len(acc)}/{len(s)}  "
          f"bias={acc[:,4].mean():+.2f}mm  std={acc[:,4].std():.2f}mm  "
          f"MAE={np.abs(acc[:,4]).mean():.2f}mm")
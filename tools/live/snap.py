import subprocess, sys
import numpy as np, cv2
out = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 2
raw = "/tmp/opencode/_snap.yuyv"
subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--stream-mmap",
                f"--stream-count={n+2}", "--stream-to=" + raw],
               check=True, capture_output=True)
d = np.fromfile(raw, dtype=np.uint8)
frames = d.reshape(n+2, 480, 848*2)[:, :, ::2]  # Y planes, skip first 2 settled
img = np.median(frames[2:].astype(np.float32), axis=0).astype(np.uint8)
cv2.imwrite(out, img)
print(out, img.shape, "mean", round(float(img.mean()),1))

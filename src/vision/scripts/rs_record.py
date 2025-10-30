import pyrealsense2 as rs
import numpy as np
import cv2

# --- Konfigurasi hemat ruang ---
WIDTH, HEIGHT, FPS = 640, 360, 15   # ubah sesuai kebutuhan

pipeline = rs.pipeline()
config = rs.config()

# Aktifkan hanya COLOR; pakai resolusi & FPS rendah
# Catatan: rs.format.bgr8 menghasilkan frame siap untuk OpenCV
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

# Mulai kamera
profile = pipeline.start(config)

# Siapkan encoder video
# Coba H.264 dulu (lebih kecil), fallback ke MP4V kalau tidak tersedia
fourcc_candidates = [
    cv2.VideoWriter_fourcc(*'avc1'),  # H.264 (sering works)
    cv2.VideoWriter_fourcc(*'H264'),  # H.264 alternatif
    cv2.VideoWriter_fourcc(*'mp4v'),  # fallback (MPEG-4 Part 2)
]
out = None
for f in fourcc_candidates:
    out = cv2.VideoWriter('rgb_output.mp4', f, FPS, (WIDTH, HEIGHT))
    if out.isOpened():
        break

print("Recording RGB only... Tekan ESC untuk stop.")
try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())  # BGR
        out.write(color_image)

        # (opsional) preview
        cv2.imshow('RGB', color_image)
        if cv2.waitKey(1) == 27:  # ESC
            break
finally:
    if out: out.release()
    pipeline.stop()
    cv2.destroyAllWindows()

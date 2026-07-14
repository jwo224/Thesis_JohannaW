import cv2
from pathlib import Path

out = Path("aruco_print_4x4_50")
out.mkdir(exist_ok=True)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

for marker_id in range(28):
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, 800)
    cv2.imwrite(str(out / f"aruco_4x4_50_id_{marker_id}.png"), img)

print(f"Generated markers in {out.resolve()}")
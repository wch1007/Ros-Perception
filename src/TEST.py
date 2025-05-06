import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
img = cv2.aruco.drawMarker(aruco_dict, 7, 400)
cv2.imwrite("aruco_id_7.png", img)

print("✅ Marker saved as aruco_id_7.png")

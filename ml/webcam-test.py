import os

import cv2

from ultralytics import YOLO

model_path = os.path.join(os.path.dirname(__file__), 'models', 'final', 'best.pt')
model = YOLO(model_path)

cap = cv2.VideoCapture(0)

while True:
	success, frame = cap.read()

	if success:
		results = model(frame, conf=0.5, max_det=1)

		print(results)

		cv2.imshow('test', results[0].plot())

	if cv2.waitKey(1) == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

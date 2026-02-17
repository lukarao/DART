import os

import av
import cv2

from ultralytics import YOLO

model = YOLO(os.path.join(os.path.dirname(__file__), '..', 'ml', 'models', 'final', 'best.pt'))

container = av.open('tcp://192.168.4.4:3000')

for packet in container.demux():
	for frame in packet.decode():
		img = frame.to_ndarray(format='bgr24')

		results = model(img, conf=0.8, max_det=1)
		
		if len(results[0].boxes) > 0:
			x, y = results[0].boxes.xywh[0][:2]
			print(f'Trash detected at coordinates: ({x}, {y})')

		cv2.imshow('results', results[0].plot())

	if cv2.waitKey(1) == ord('q'):
		break

cv2.destroyAllWindows()
container.close()

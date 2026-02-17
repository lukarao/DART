import av
import cv2

container = av.open('tcp://192.168.4.4:3000')

for packet in container.demux():
	for frame in packet.decode():
		img = frame.to_ndarray(format='bgr24')

		cv2.imshow('drone', img)

	if cv2.waitKey(1) == ord('q'):
		break

cv2.destroyAllWindows()
container.close()

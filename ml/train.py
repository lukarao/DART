import os

from ultralytics import YOLO

if __name__ == '__main__':
	os.chdir(os.path.join(os.path.dirname(__file__), 'models', 'pretrained'))

	model = YOLO('yolo11s.pt')

	model.train(
		data=os.path.join(os.path.dirname(__file__), 'dataset3', 'data.yaml'),
		project=os.path.join(os.path.dirname(__file__), 'runs'),
		name='train',
		device=0,
		epochs=300
	)

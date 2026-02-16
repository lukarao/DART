import glob
import os
import sys

import requests
import splitfolders

from io import BytesIO
from PIL import Image

dataset_dir = os.path.join(os.path.dirname(__file__), 'dataset2')

# by Pedro F. Proenza
# https://github.com/pedropro/TACO/blob/master/download.py

# fetch the COCO data
coco_data = requests.get('https://raw.githubusercontent.com/pedropro/TACO/refs/heads/master/data/annotations.json').json()

nr_images = len(coco_data['images'])
for i in range(nr_images):

    image = coco_data['images'][i]

    file_name = image['file_name']
    url_original = image['flickr_url']
    url_resized = image['flickr_640_url']

    file_path = os.path.join(dataset_dir, file_name)

    # create subdir if necessary
    subdir = os.path.dirname(file_path)
    if not os.path.isdir(subdir):
        os.mkdir(subdir)

    if not os.path.isfile(file_path):
        # load and save image
        response = requests.get(url_original)
        img = Image.open(BytesIO(response.content))
        if img._getexif():
            img.save(file_path, exif=img.info["exif"])
        else:
            img.save(file_path)

    # show loading bar
    bar_size = 30
    x = int(bar_size * i / nr_images)
    sys.stdout.write("%s[%s%s] - %i/%i\r" % ('loading: ', "=" * x, "." * (bar_size - x), i, nr_images))
    sys.stdout.flush()
    i+=1

sys.stdout.write('finished\n')

# by Zubin Bhuyan
# https://github.com/z00bean/coco2yolo-seg/blob/main/COCO2YOLO-seg.py

# create temp/labels and temp/images folder
temp_folder = os.path.join(os.path.dirname(__file__), 'dataset', 'temp')
output_folder = os.path.join(temp_folder, 'labels')
images_folder = os.path.join(temp_folder, 'images')
os.makedirs(output_folder, exist_ok=True)
os.makedirs(images_folder, exist_ok=True)

for annotation in coco_data['annotations']:
	image_id = annotation['image_id']
	category_id = annotation['category_id']
	segmentation = annotation['segmentation']
	bbox = annotation['bbox']

	image_filename = str(image_id).zfill(6)

	# find the image width and height from the COCO data
	for image in coco_data['images']:
		if image['id'] == image_id:
			image_width = image['width']
			image_height = image['height']
			# copy image file to images folder
			image_path = os.path.join(images_folder, f'{image_filename}.jpg')
			if not os.path.isfile(image_path):
				os.rename(os.path.join(dataset_dir, image['file_name']), image_path)
			break

	# calculate the normalized center coordinates and width/height
	x_center = (bbox[0] + bbox[2] / 2) / image_width
	y_center = (bbox[1] + bbox[3] / 2) / image_height
	bbox_width = bbox[2] / image_width
	bbox_height = bbox[3] / image_height

	# convert COCO segmentation to YOLO segmentation
	yolo_segmentation = [f'{(x) / image_width:.5f} {(y) / image_height:.5f}' for x, y in zip(segmentation[0][::2], segmentation[0][1::2])]
	yolo_segmentation = ' '.join(yolo_segmentation)

	# save the YOLO segmentation annotation in a file
	output_filename = os.path.join(output_folder, f'{image_filename}.txt')
	with open(output_filename, 'w') as file:
		file.write(f'{category_id} {yolo_segmentation}\n')

# try to delete batch_* folders
try:
	for f in glob.glob(os.path.join(os.path.dirname(dataset_dir), 'batch_*')):
		os.rmdir(f)
except:
	print('unable to delete batch_* folders')

# split dataset into train, val, and test folders
splitfolders.ratio(temp_folder, output=os.path.join(os.path.dirname(__file__), 'dataset'), move=True)

# try to delete temp folder
try:
	os.rmdir(temp_folder)
except:
	print('unable to delete temp folder')

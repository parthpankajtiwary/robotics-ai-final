import os
import cv2
import numpy as np
import random


# structure of the bounding boxes should be (left, top, right, bottom, label)
def random_horizontal_flip(img, bounding_boxes):
	p = 0.5	
	img_center = np.array(img.shape[:2])[::-1]/2	
	img_center = np.hstack((img_center, img_center))
	if random.random() < p:
		img = img[:, ::-1, :]
		bounding_boxes[:, [0, 2]] += 2*(img_center[[0, 2]] - bounding_boxes[:, [0, 2]])
	
		box_width = abs(bboxes[:, 0] - bboxes[:, 2])
		bounding_boxes[:, 0] -= box_width
		bounding_boxes[:, 2] += box_width
	
	return img, bounding_boxes

def change_brightness(img):
	'''
	# convert to hsv
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# select a random value to change
	value = random.randint(-50, 50)

	# First approach
	print 'Before'
	print hsv
	
	#h, s, v = cv2.split(hsv)
	
	# change brightness by value
	hsv[:,:,2] = hsv[:,:,2] + value

	# change for brightness to in 0-255 range
	hsv[:,:,2][hsv[:,:,2] < 0] = 0
	hsv[:,:,2][hsv[:,:,2] > 255] = 255
	#v[v>255] = 255

	# convert to rgb
	#hsv = cv2.merge((h, s, v))
	image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
	print 'After'
	print hsv
	'''
	# Second approach
	
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# select a random value to change
	value = random.randint(50, 200)
		
	hsv[:,:,2] = value
	image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
	
	return image
	
def original_crop(img,box):
	print(img.shape)
	return img[box[0]:box[1], box[2]:box[3]]	

def change_crop(img, box):
	img_height, img_width, _ = img.shape	

	height = box[1] - box[0]
	width = box[3] - box[2]

	scale = 4

	change_height = round(height / scale)
	change_width = round(height / scale)
	
	h1_change = random.randint(-change_height, change_height)
	h2_change = random.randint(-change_height, change_height)
	w1_change = random.randint(-change_width, change_width)
	w2_change = random.randint(-change_width, change_width)
	
	newbox = [0,0,0,0]
	newbox[0] = box[0] + h1_change
	newbox[1] = box[1] + h2_change
	newbox[2] = box[2] + w1_change
	newbox[3] = box[3] + w2_change
	print(img_height)
	print(img_width)
	
	if newbox[0] < 0:
		newbox[0] = 0
	if newbox[0] > img_height:
		newbox[0] = img_height-1
	if newbox[1] < 0:
		newbox[1] = 0
	if newbox[1] > img_height:
		newbox[1] = img_height-1

	if newbox[2] < 0:
		newbox[2] = 0
	if newbox[2] > img_width:
		newbox[2] = img_width-1
	if newbox[3] < 0:
		newbox[3] = 0
	if newbox[3] > img_width:
		newbox[3] = img_width-1

	print(newbox)

	return img[newbox[0]:newbox[1],newbox[2]:newbox[3]]

	

def flip_image_vert(img):
	vert_img = cv2.flip(img,0)
	return vert_img

def flip_image_hor(img):
	hor_img = cv2.flip(img,1)
	return hor_img

rootdir = '/home/student/dataset/'
writedir = '/home/student/Desktop/'
label = 'tomatosoup'

# first we extract all the index in the folder

for dirs, subdirs, files in os.walk(rootdir):

	for dir in subdirs:
		root_dir = dirs + dir + '/'
		print(dir)
		files_index = []
		for files in os.walk(root_dir):
			for file in files[2]:
				split_holder = file.split('.')
				index_value = int(split_holder[0])
				if index_value not in files_index:
					files_index.append(index_value)

		files_index.sort()
		
		os.makedirs('final_images_' + dir)
		# loop through all the corresponding image and csv file

		for file_index in files_index:
			file_image = str(file_index) + str('.jpg')	
			file_csv = str(file_index) + str('.csv')

			# top bottom left right	
			bounding_box = []	

			# open csv file and extract bounding box parameters
			with open(root_dir + file_csv) as f:		
				for row in f:
					bounding_box.append(int(row))

			print(bounding_box)

			# load the image
			image = cv2.imread(root_dir + file_image)
			
			
			# change brightness
			image_bright = change_brightness(image)
	
			
			# save original crop image
			original_image = original_crop(image, bounding_box)
			
			write_name = str(file_index) + '-original.jpg'	

			print(writedir + 'final_images_' + dir + '/' + write_name)		
			original_image_resize = cv2.resize(original_image, (150,150))			
			cv2.imwrite(os.path.join(writedir + 'final_images_' + dir + '/' + write_name), original_image_resize)
			
			# cv2.imshow('Original Crop', original_image_resize)
			# cv2.waitKey(0)	
			
			# save brightness changed image
			crop_bright = original_crop(image_bright, bounding_box)
			
			write_name = str(file_index) + '-bright.jpg'
			crop_bright_resize = cv2.resize(crop_bright, (150,150))
			cv2.imwrite(os.path.join(writedir + 'final_images_' + dir + '/' +  write_name), crop_bright)

			# cv2.imshow('Bright Crop', crop_bright_resize)
			# cv2.waitKey(0)

			
			# save random cropped image
			image_crop = change_crop(image, bounding_box)	

			write_name = str(file_index) + '-crop.jpg'
			image_crop_resize = cv2.resize(image_crop, (150,150))
			cv2.imwrite(os.path.join(writedir + 'final_images_' + dir + '/' +  write_name), image_crop)

			# cv2.imshow('Random Crop', image_crop_resize)
			# cv2.waitKey(0)


			# save flipped image
			flipped_image = flip_image_hor(original_image)
			

			write_name = str(file_index) + '-flip.jpg'
			flipped_image_resize = cv2.resize(flipped_image, (150,150))
			cv2.imwrite(os.path.join(writedir + 'final_images_' + dir + '/' +  write_name), flipped_image)
			
			# cv2.imshow('Flip Crop', flipped_image_resize)
			# cv2.waitKey(0)

			





		

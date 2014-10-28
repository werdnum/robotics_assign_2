#!/usr/bin/env python

import cv2
import numpy as np
import os
import sys

kNN = cv2.KNearest()

images = []
responses = []

testImages = []
testResponses = []

count = 0

print "Loading images..."

for index in range(ord('a'), ord('z') + 1):
	letter = chr(index)
	with open(letter + '/files.txt') as f:
		files = f.readlines()
	for filename in files:
		filename = filename.strip()
		img = cv2.imread(letter + '/' + filename)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		row = np.reshape(img, -1)
		# if count % 2 < 2:
		images.append(row)
		responses.append(index)
		# else:
		# 	testImages.append(row)
		# 	testResponses.append(index)
		count += 1

print "Saving ", len(images), " images..."

images = np.array(images, np.float32)
responses = np.array(responses, np.float32)

np.savez_compressed('knn.npz', images=images, letters=responses)

# testImages = np.array(testImages, np.float32)
# testResponses = np.array(testResponses, np.float32)

# print "Testing with ", len(testImages), " images..."

# sys.exit

# ret,result,neighbours,dist = kNN.find_nearest(testImages,k=3)

# correct = 0
# correctByChar = {chr(char) : 0 for char in range(ord('a'), ord('z')+1)}
# distanceByChar = {chr(char) : 0 for char in range(ord('a'), ord('z')+1)}
# numByChar = {chr(char) : 0 for char in range(ord('a'), ord('z')+1)}

# for i in range(0, len(result)):
# 	char = chr(testResponses[i])
# 	numByChar[char] += 1
# 	if result[i][0] == testResponses[i]:
# 		avgDistance = reduce(lambda x, y: x+ y, dist[i]) / 3
# 		correct += 1
# 		correctByChar[char] += 1
# 		distanceByChar[char] += avgDistance

# for char in correctByChar.keys():
# 	print char, "\t", (correctByChar[char] * 100 / numByChar[char]), "\t", \
# 		(distanceByChar[char] / numByChar[char])

# accuracy = correct*100.0/result.size
# print "Total\t", accuracy

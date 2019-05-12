import os
import Augmentor

# directories of image data
right_dir = "<Your project directory>//Gesture Dataset//right//"
left_dir = "<Your project directory>//Gesture Dataset//left//"

# create two seperate pipelines
pr = Augmentor.Pipeline(right_dir)
pl = Augmentor.Pipeline(left_dir)

# augmentation
pr.rotate(probability=0.5, max_left_rotation=6, max_right_rotation=6)
pr.zoom(probability=0.3, min_factor=1.1, max_factor=1.4)
pr.crop_random(probability=1, percentage_area=0.85)

pl.rotate(probability=0.5, max_left_rotation=6, max_right_rotation=6)
pl.zoom(probability=0.3, min_factor=1.1, max_factor=1.4)
pl.crop_random(probability=1, percentage_area=0.85)

# no of sample you want to generate
pr.sample(1500)
pl.sample(1500)





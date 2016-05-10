import tensorflow as tf
import numpy as np
import os

def build_gc_bin():
    r = np.array([])
    g = np.array([])
    b = np.array([])
    label = np.array([])
    index = 0
    data_dir = '/home/student/Downloads/images/train'
    #init tensorflow read, so the data format will be tensor and we can use tensorflow resizing tool
    reader = tf.WholeFileReader()
    init_op = tf.initialize_all_variables()
    filenames = [os.path.join(data_dir, 'frame00%02d.jpg' % (i,)) for i in xrange(0, 61)]
    for f in filenames:
        filename_queue = tf.train.string_input_producer([f]) 
        key, value = reader.read(filename_queue)
        image = tf.image.decode_jpeg(value) 
        # using K-NN Interpolation
        image = tf.image.resize_images(image, 32, 32, method=1, align_corners=False)
        #convert image shape from raw to 32x32x3                     
        with tf.Session() as sess:
            coord = tf.train.Coordinator()
            threads = tf.train.start_queue_runners(coord=coord)
            sess.run(init_op)
            image = image.eval() #here is your image Tensor :) 
            coord.request_stop()
            coord.join(threads)
        #convert to numpy array to match the CIFAR10 format
        im = (np.array(image))
        r = np.append(r, im[:,:,0].flatten(),axis=0)
        g = np.append(g, im[:,:,1].flatten(),axis=0)
        b = np.append(b, im[:,:,2].flatten(),axis=0)
        if index <= 10:
            temp = [9]
        elif index <= 22:
            temp = [5]
        elif index <= 40:
            temp = [1]
        else:
            temp = [0]
        label = np.append(label, temp)
        print index,temp
        index = index + 1
    out = np.array(list(label) + list(r) + list(g) + list(b),np.uint8)
    out.tofile('/home/student/Downloads/images/gc_image_cifar10_format.bin')

    
def main(argv=None):  # pylint: disable=unused-argument
  build_gc_bin()

if __name__ == '__main__':
  tf.app.run()
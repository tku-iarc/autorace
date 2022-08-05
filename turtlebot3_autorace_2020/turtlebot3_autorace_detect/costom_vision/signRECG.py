#!/usr/bin/env python
# -*- coding: utf-8 -*-+
import tensorflow as tf
import numpy as np
import cv2


###載入模型
# Import the TF graph
graph_def = tf.GraphDef()
with tf.gfile.FastGFile("model.pb", 'rb') as f:
    graph_def.ParseFromString(f.read())
    tf.import_graph_def(graph_def, name='')
###載入標籤
# Create a list of labels.
labels = []
with open("labels.txt", 'rt') as lf:
    for l in lf:
        labels.append(l.strip())


def oprate(image):
    ###協助程式函式
    def crop_center(img,cropx,cropy):
        h, w = img.shape[:2]
        startx = w//2-(cropx//2)
        starty = h//2-(cropy//2)
        return img[starty:starty+cropy, startx:startx+cropx]

    def resize_down_to_1600_max_dim(image):
        h, w = image.shape[:2]
        if (h < 1600 and w < 1600):
            return image

        new_size = (1600 * w // h, 1600) if (h > w) else (1600, 1600 * h // w)
        return cv2.resize(image, new_size, interpolation = cv2.INTER_LINEAR)

    def resize_to_256_square(image):
        h, w = image.shape[:2]
        return cv2.resize(image, (256, 256), interpolation = cv2.INTER_LINEAR)

    ###處理維度 >1600 的影像
    # If the image has either w or h greater than 1600 we resize it down respecting
    # aspect ratio such that the largest dimension is 1600
    image = resize_down_to_1600_max_dim(image)


    ###裁剪中央最大的正方形
    # We next get the largest center square
    h, w = image.shape[:2]
    min_dim = min(w,h)
    max_square_image = crop_center(image, min_dim, min_dim)


    ###將大小往下調整為 256x256
    # Resize that square down to 256x256
    augmented_image = resize_to_256_square(max_square_image)


    ###裁剪模型特定輸入大小的中心
    # The compact models have a network size of 227x227, the model requires this size.
    network_input_size = 224

    # Crop the center for the specified network_input_Size
    augmented_image = crop_center(augmented_image, network_input_size, network_input_size)


    ###預測影像

    # These names are part of the model and cannot be changed.
    output_layer = 'loss:0'
    input_node = 'Placeholder:0'

    with tf.Session() as sess:
        prob_tensor = sess.graph.get_tensor_by_name(output_layer)
        predictions, = sess.run(prob_tensor, {input_node: [augmented_image] })


        ###View the results
        # Print the highest probability label

        # Or you can print out all of the results mapping labels to probabilities.
        label_index = 0
        for p in predictions:
            truncated_probablity = np.float64(round(p,8))
            #print (labels[label_index],":%4f "%truncated_probablity,end=" | ")
            print (labels[label_index],":%4f "%truncated_probablity)
            label_index += 1
        highest_probability_index = np.argmax(predictions)
        print('Classified as:' + labels[highest_probability_index])
        return labels[highest_probability_index]
### MAIN ###
if __name__ == '__main__':
    #cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)  # 啟用鏡頭
    cap = cv2.VideoCapture(0)  # 啟用鏡頭
    while (True):
        ret, frame = cap.read()  # 讀取相機畫面
        cv2.imshow("show",frame) #顯示相機畫面
        res = oprate(frame)
        print(res)
        #if cv2.waitKey(1) & 0xFF == ord('a'):
        #    res = oprate(frame)
        #    print(res)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # 按下按鍵關閉程式
            break
    cap.release()  # 關閉相機

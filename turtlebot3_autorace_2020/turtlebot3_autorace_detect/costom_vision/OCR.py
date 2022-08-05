import pytesseract
import numpy as np
import cv2

net = cv2.dnn.readNet("frozen_east_text_detection.pb")

def text_detect(img):
    def non_max_suppression(boxes, overlapThresh):
        # if there are no boxes, return an empty list
        if len(boxes) == 0:
            return []

        # initialize the list of picked indexes
        pick = []

        # grab the coordinates of the bounding boxes
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y2)
        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(i)
            suppress = [last]
            # loop over all indexes in the indexes list
            for pos in range(0, last):
                # grab the current index
                j = idxs[pos]

                # find the largest (x, y) coordinates for the start of
                # the bounding box and the smallest (x, y) coordinates
                # for the end of the bounding box
                xx1 = max(x1[i], x1[j])
                yy1 = max(y1[i], y1[j])
                xx2 = min(x2[i], x2[j])
                yy2 = min(y2[i], y2[j])

                # compute the width and height of the bounding box
                w = max(0, xx2 - xx1 + 1)
                h = max(0, yy2 - yy1 + 1)

                # compute the ratio of overlap between the computed
                # bounding box and the bounding box in the area list
                overlap = float(w * h) / area[j]

                # if there is sufficient overlap, suppress the
                # current bounding box
                if np.any(overlap > overlapThresh):
                    suppress.append(pos)

            # delete all indexes from the index list that are in the
            # suppression list
            idxs = np.delete(idxs, suppress)

            # return only the bounding boxes that were picked
        return boxes[pick]
    (H, W) = img.shape[:2]

    blob = cv2.dnn.blobFromImage(img, 1.0, (320, 320), (123.68, 116.78, 103.94), True, False)
    layerNames = ["feature_fusion/Conv_7/Sigmoid","feature_fusion/concat_3"]

    blob = cv2.dnn.blobFromImage(img, 1.0, (W, H),(123.68, 116.78, 103.94), swapRB=True, crop=False)

    net.setInput(blob)
    try:
        (scores, geometry) = net.forward(layerNames)
    except:
        return False

    (numRows, numCols) = scores.shape[2:4]
    rects = []
    confidences = []
    for y in range(0, numRows):
        scoresData = scores[0, 0, y]
        xData0 = geometry[0, 0, y]
        xData1 = geometry[0, 1, y]
        xData2 = geometry[0, 2, y]
        xData3 = geometry[0, 3, y]
        anglesData = geometry[0, 4, y]

        for x in range(0, numCols):
            # if our score does not have sufficient probability, ignore it
            if scoresData[x] < 0.5:
                continue

            # compute the offset factor as our resulting feature maps will
            # be 4x smaller than the input image
            (offsetX, offsetY) = (x * 4.0, y * 4.0)

            # extract the rotation angle for the prediction and then
            # compute the sin and cosine
            angle = anglesData[x]
            cos = np.cos(angle)
            sin = np.sin(angle)

            # use the geometry volume to derive the width and height of
            # the bounding box
            h = xData0[x] + xData2[x]
            w = xData1[x] + xData3[x]

            # compute both the starting and ending (x, y)-coordinates for
            # the text prediction bounding box
            endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
            endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
            startX = int(endX - w)
            startY = int(endY - h)

            # add the bounding box coordinates and probability score to
            # our respective lists
            rects.append((startX, startY, endX, endY))
            confidences.append(scoresData[x])

    boxes = non_max_suppression(np.array(rects),confidences)

    return boxes



def OCR(image):
    # 處理輸出字串
    def readText(str):
        lines = ''.join(e for e in str if e.isalnum())
        #print("origin result:", lines)
        if lines.upper().find("LEFT") is not -1:
            # print("LEFT")
            return "left"
        elif lines.upper().find("RIGHT") is not -1:
            # print("RIGHT")
            return "right"
        elif lines.upper().find("STOP") is not -1:
            # print("STOP")
            return "stop"
        else:
            # print("other")
            return "other"
    config = ("-l eng --oem 1 --psm 7")
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # gray = cv2.threshold(gray, 0, 255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    # qgray = cv2.medianBlur(gray, 3)
    boxes = text_detect(image)
    if boxes is False:return False
    res = ""

    # # loop over the bounding boxes
    # for (startX, startY, endX, endY) in boxes:
    #     crop_image = image[startY:endY, startX:endX]
    #
    #     #cv2.imshow("crop",crop_image)
    #
    #     #cv2.imwrite("tmp.jpg", crop_image)
    #     try:
    #         res += pytesseract.image_to_string(crop_image, config=config)
    #     except:
    #         break
    # return res

    (startX, startY, endX, endY)=(0,0,0,0)
    if len(boxes) is not 0:
        (startX, startY, endX, endY) = boxes[0]
    crop_image = image[startY:endY, startX:endX]
    try:
        res = pytesseract.image_to_string(crop_image, config=config)
    except:
        pass
    #print(readText(res))
    return readText(res)

    
if __name__ == '__main__':
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)  # 啟用鏡頭
    # cap.set(3, 320)  # 設定畫面大小
    # cap.set(4, 240)

    while(True):
        ret,frame = cap.read() #讀取相機畫面
        cv2.imshow("show",frame) #顯示相機畫面

        if cv2.waitKey(1) & 0xFF == ord('a'):
            res = OCR(frame)
            print(res)
        if cv2.waitKey(1) & 0xFF == ord('q'): #按下按鍵關閉程式
            break
    cv2.destroyAllWindows()  # 關閉顯示視窗 有imshow才需要打
    cap.release()  # 關閉相機

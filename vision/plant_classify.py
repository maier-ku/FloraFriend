import numpy as np
import cv2
import os
import sys
import torch
from PIL import Image
import pathlib
from transformers import GLPNFeatureExtractor, GLPNForDepthEstimation

gen_path = pathlib.Path.cwd()
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

yolo_modelpath = "best.pt"

if __name__ == '__main__':
    yolo = torch.hub.load('yolov5','custom', path=yolo_modelpath,source='local')
    yolo.conf = 0.2  #confidence threshold

    feature_extractor = GLPNFeatureExtractor.from_pretrained("vinvino02/glpn-nyu")
    depth_model = GLPNForDepthEstimation.from_pretrained("vinvino02/glpn-nyu")

    # define a video capture object
    cam = cv2.VideoCapture(0)

    while True:

        # Capture the video frame by frame
        ret, frame = cam.read()
        frame = frame[:, :, [2,1,0]]
        frame = Image.fromarray(frame)
        frame = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)


        # Plant detection
        result = yolo(frame)

        # # Display prediction results
        cv2.imshow('PotPlant', np.squeeze(result.render()))
        result = result.xyxy[0].tolist()

        if result:
            result = result[0]
            centre_x = result[0] + (result[2] - result[0])
            centre_y = result[1] + (result[3] - result[1])
            print(result[0:4])

        pixel_values = feature_extractor(image, return_tensors="pt").pixel_values


        # the 'q' button is the quitting button
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    cam.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
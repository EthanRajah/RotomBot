#!/usr/bin/env python3

from .darknet import load_network, make_image, copy_image_from_bytes, detect_image, free_image, draw_boxes
import cv2
import numpy as np
import os
# import matplotlib.pyplot as plt

class Darknet:
    def __init__(self, config_file, weights_file, datafile,  threshold=0.3):
        
        self.threshold = threshold
        self.image_width = 416
        self.image_height = 416
        self.network, self.class_names, _ = self.load_yolo(
            config_file=config_file,
            weights_file=weights_file,
            data_file=datafile
        )
        self.class_colors = {'crack': (0, 255, 0), 'no_crack': (255, 0, 0)}  # Example colors for classes

    def load_yolo(self, config_file=None, weights_file=None, data_file=None):
        
        network, class_names, class_colors = load_network(
            config_file,
            data_file,
            weights_file,
            batch_size=1,
        )
        return network, class_names, class_colors
    
    def run_inference(self, image, process_red_box, image_save_path=None):
        if process_red_box:
            image = self.extract_inner_content_v3(image, None)
            if image is None:
                Exception("No red frame detected or inner rectangle is too small after applying margins")
                return None, None

        darknet_image = make_image(self.image_width, self.image_height, 3)

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, 
                                   (self.image_width, self.image_height),
                                   interpolation=cv2.INTER_LINEAR
                                   )

        copy_image_from_bytes(darknet_image, image_resized.tobytes())
        detections = detect_image(self.network, 
                                          self.class_names, 
                                          darknet_image, 
                                          thresh=self.threshold)
        free_image(darknet_image)
        
        crack_flag = False
        for label, confidence, bbox in detections:
            if label == 'crack':
               crack_flag =  True
        #save the inferenced_image
        if image_save_path is not None:
            print("Drawing boxes")
            image = draw_boxes(detections, image_resized, self.class_colors)
            print("Converting colour")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            print("Writing...")
            # cv2.imwrite(image_save_path, image)
            print("Saving Images: Written")
            return crack_flag, image
        else:
            return crack_flag, None
        



    def extract_inner_content_v3(self, img, output_path = None):

        # Create a grayscale copy for finding inner content
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply thresholding to get a binary image
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Find red frame using color thresholding
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 + mask2
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if red_contours:
            red_contour = max(red_contours, key=cv2.contourArea)
            red_rect = cv2.boundingRect(red_contour)
            x_red, y_red, w_red, h_red = red_rect

            # Instead of finding inner contours, apply a more aggressive margin to the red frame
            # This ensures we stay well clear of any red border

            # Calculate margin as percentage of frame size (more aggressive)
            margin_percent = 0.05  # 12% margin (adjust as needed)
            margin_x = int(w_red * margin_percent)
            margin_y = int(h_red * margin_percent)

            # Ensure minimum margin is at least 20 pixels
            margin_x = max(margin_x, 20)
            margin_y = max(margin_y, 20)

            # Define the inner rectangle
            x_inner = x_red + margin_x
            y_inner = y_red + margin_y
            w_inner = w_red - 2 * margin_x
            h_inner = h_red - 2 * margin_y

            # Make sure the dimensions are valid
            if w_inner > 0 and h_inner > 0:
                # Extract the content
                content = img[y_inner:y_inner + h_inner, x_inner:x_inner + w_inner]

                # Save the extracted content
                if output_path is not None:
                    cv2.imwrite(output_path, content)

                # For visualization
                # result = img.copy()
                # cv2.rectangle(result, (x_red, y_red), (x_red + w_red, y_red + h_red), (0, 0, 255), 2)  # Red frame
                # cv2.rectangle(result, (x_inner, y_inner), (x_inner + w_inner, y_inner + h_inner), (0, 255, 0),
                #             2)  # Inner crop

                # Show results
                # plt.figure(figsize=(15, 8))
                # plt.subplot(131), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), plt.title('Original')
                # plt.subplot(132), plt.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB)), plt.title('Detected Regions')
                # plt.subplot(133), plt.imshow(cv2.cvtColor(content, cv2.COLOR_BGR2RGB)), plt.title('Extracted Content')
                # plt.tight_layout()
                # plt.show()

                return content
            else:
                print("Inner rectangle is too small after applying margins")
                return None
        else:
            print("No red frame detected")
            return None
        
    
if __name__ == "__main__":
    # Load the network
    config_file = '/home/jetson/ros2_ws/src/image_processor/image_processor/darknet_dir/training/yolov4-tiny-custom.cfg'
    weights_file = '/home/jetson/ros2_ws/src/image_processor/image_processor/darknet_dir/training/yolov4-tiny-custom_3000.weights'
    data_file = '/home/jetson/ros2_ws/src/image_processor/image_processor/darknet_dir/training/obj.data'

    
    net = Darknet(config_file, weights_file, data_file, threshold=0.10)
    
    
    # net.run_inference(cv2.imread('/home/epyc-ubuntu/yolo/darknet/IMG_5315.jpg'), process_red_box=False, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/IMG_5315.jpg')
    # net.run_inference(cv2.imread('/home/epyc-ubuntu/yolo/darknet/IMG_5316.jpg'),process_red_box=False, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/IMG_5316.jpg')
    # net.run_inference(cv2.imread('/home/epyc-ubuntu/yolo/darknet/IMG_5316_c.jpg'),process_red_box=False, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/IMG_5316_c.jpg')
    # net.run_inference(cv2.imread('/home/epyc-ubuntu/yolo/darknet/laptop_crack_img.png'),process_red_box=False, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/laptop_crack_img.png')
    result , image_inference = net.run_inference(cv2.imread('/home/jetson/Downloads/B/image_20250414-142032_0.png'),process_red_box=True, image_save_path='/home/jetson/Downloads/A/testinference_image_20250411-101154_0_inference.png')
    cv2.imwrite('/home/jetson/Downloads/B/image_20250414-142032_0_manual.png', image_inference)
    # net.run_inference(cv2.imread('/home/epyc-ubuntu/yolo/darknet/IMG_5324.jpg'),process_red_box=True, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/IMG_5324.jpg')
    # net.run_inference(cv2.imread('/home/epyc-ubuntu/yolo/darknet/IMG_5326.jpg'),process_red_box=True, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/IMG_5326.jpg')
    
    # paths = '/home/epyc-ubuntu/yolo/darknet/training/train.txt'
    # image_paths = [path for path in open(paths).read().splitlines() if path.endswith('.jpg') or path.endswith('.png') or path.endswith('.jpeg')]
    
    # for line in paths:
    #     path]
    
    # paths = ['/home/epyc-ubuntu/yolo/darknet/training/obj/CRACK500_20160328_152511_1921_361.jpg',
    #          '/home/epyc-ubuntu/yolo/darknet/training/obj/CRACK500_20160308_073532_1921_721.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/Volker_DSC01692_316_1370_932_1128.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/DeepCrack_11114.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/Rissbilder_for_Florian_9S6A2886_166_145_3216_3619.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/Rissbilder_for_Florian_9S6A2808_860_1782_2496_2905.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/CRACK500_20160222_165937_1281_721.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/Volker_DSC01680_13_1111_1794_1378.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/CRACK500_20160222_115833_1281_721.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/CRACK500_20160329_094025_1281_361.jpg',
    #         '/home/epyc-ubuntu/yolo/darknet/training/obj/Volker_DSC01650_491_62_1631_1742.jpg']
    # for path in image_paths[:100]:
    #     net.run_inference(cv2.imread(path), process_red_box=False, image_save_path='/home/epyc-ubuntu/yolo/darknet/testing_out/' + path.split('/')[-1])
   
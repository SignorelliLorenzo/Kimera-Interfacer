#!/usr/bin/env python3
import rospy
import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image as SensorImage

class PILBridge:
    @staticmethod
    def numpy_to_rosimg(array, frame_id="camera", encoding="rgb8", stamp=None):
        """
        Convert a NumPy array to a ROS Image message using PIL
        Args:
            array: NumPy array (H,W,3) or (H,W) or (H,W,4)
            frame_id: ROS frame ID for the image header
            encoding: ROS image encoding (e.g., "rgb8", "bgr8", "mono8", "32FC1", "16UC1")
                     If None, will try to infer from array shape/dtype
            stamp: ROS Time for the header (default: current time)
        Returns:
            sensor_msgs.msg.Image
        """
        if stamp is None:
            stamp = rospy.Time.now()
            
        # Convert array to PIL Image
        pil_img = PILImage.fromarray(array)
        
        # Create ROS Image message
        img_msg = SensorImage()
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = frame_id
        img_msg.height = pil_img.height
        img_msg.width = pil_img.width
        
        # Determine encoding if not specified
        if encoding is None:
            if array.ndim == 2:  # Grayscale
                encoding = "mono8"
            elif array.shape[2] == 3:  # Color
                encoding = "rgb8"
            else:
                raise ValueError("Cannot infer encoding from array shape")
        
        img_msg.encoding = encoding
        
        # Set step size based on encoding
        if encoding in ["rgb8", "bgr8"]:
            img_msg.step = pil_img.width * 3
        elif encoding == "mono8":
            img_msg.step = pil_img.width
        elif encoding == "32FC1":
            img_msg.step = pil_img.width * 4
        elif encoding == "16UC1":
            img_msg.step = pil_img.width * 2  # 16 bits = 2 bytes per pixel
        else:
            raise ValueError(f"Unsupported encoding: {encoding}")
        
        img_msg.is_bigendian = 0
        img_msg.data = np.array(pil_img).tobytes()
        
        return img_msg

    @staticmethod
    def rosimg_to_numpy(img_msg):
        """
        Convert a ROS Image message to a NumPy array using PIL
        Args:
            img_msg: sensor_msgs.msg.Image
        Returns:
            NumPy array
        """
        # Convert ROS Image to PIL Image
        pil_img = PILImage.frombytes(
            mode=PILBridge._ros_encoding_to_pil_mode(img_msg.encoding),
            size=(img_msg.width, img_msg.height),
            data=img_msg.data,
            decoder_name='raw'
        )
        
        return np.array(pil_img)

    @staticmethod
    def _ros_encoding_to_pil_mode(encoding):
        """
        Map ROS image encoding to PIL mode
        """
        encoding_map = {
            "rgb8": "RGB",
            "bgr8": "RGB",  # Note: channel order will need separate handling
            "mono8": "L",
            "32FC1": "F",
            "rgba8": "RGBA",
            "bgra8": "RGBA",  # Note: channel order will need separate handling
            "16UC1": "I;16",  # This is how PIL handles 16-bit unsigned images
        }
        
        if encoding not in encoding_map:
            raise ValueError(f"Unsupported ROS encoding: {encoding}")
        
        return encoding_map[encoding]
